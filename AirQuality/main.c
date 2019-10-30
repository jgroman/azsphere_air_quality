/***************************************************************************//**
* @file    main.c
* @version 1.0.0
*
* @brief Air Quality Monitor.
*
* @author Jaroslav Groman
*
* @date
*
*******************************************************************************/

#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>

#include "applibs_versions.h"   // API struct versions to use for applibs APIs
#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/i2c.h>

#include <azureiot/iothub_device_client_ll.h>

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

// Using a single-thread event loop pattern based on Epoll and timerfd
#include "epoll_timerfd_utilities.h"

// Azure IoT utilities
#include "azure_iot_utilities.h"

// This application Azure IoT configuration
#include "azure_iot_settings.h"

// Referenced libraries
#include "lib_ccs811.h"
#include "lib_hdc1000.h"
#include "lib_u8g2.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define I2C_ISU             PROJECT_ISU2_I2C
#define I2C_ADDR_OLED       (0x3C)

#define OLED_ROTATION       U8G2_R1 // Display is rotated 90 degrees clockwise
#define OLED_LINE_LENGTH    16      // Max number of chars on display line

#define JSON_BUFFER_SIZE    128     // JSON buffer for Azure uplod

/*******************************************************************************
* External variables
*******************************************************************************/

// TODO: Refactor to use function instead of extern var
extern IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle;

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

/** @brief Application termination handler.
 *
 * Signal handler for termination requests. This handler must be
 * async-signal-safe.
 *
 * @param signal_number	
 *
 */
static void
termination_handler(int signal_number);

/**
 * @brief Button1 press handler
 */
static void
button1_press_handler(void);

/**
 * @brief CCS811 DATA READY interrupt handler
 */
static void
ccs811_interrupt_handler(void);

/**
 * @brief Show measured values on OLED display
 */
static void
display_measurements(void);

/**
 * @brief Timer event handler for polling button states
 */
static void
button_timer_event_handler(EventData *event_data);

/**
 * @brief Timer event handler for polling CCS811 interrupt pin
 */
static void
ccs811_int_timer_event_handler(EventData *event_data);

/**
 * @brief Timer event handler for uploading data to Azure
 */
static void
upload_timer_event_handler(EventData *event_data);

/**
 * @brief Azure upload handler
 */
static void
azure_upload_handler(void);

/**
 * @brief Initialize signal handlers.
 *
 * Set up SIGTERM termination handler.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_handlers(void);

/**
 * @brief Initialize peripherals.
 *
 * Initialize all peripherals used by this project.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_peripherals(I2C_InterfaceId isu_id);

/**
 * @brief Close all peripherals and handlers
 */
static void
close_peripherals_and_handlers(void);

/*******************************************************************************
* Global variables
*******************************************************************************/

// Period how often will be data uploaded to Azure
static const struct timespec AZURE_UPLOAD_PERIOD = { 1 * 60, 0 };

// Termination state flag
static volatile sig_atomic_t gb_is_termination_requested = false;

// File descriptors
static int g_fd_epoll = -1;                 // Epoll
static int g_fd_i2c = -1;                   // I2C
static int g_fd_poll_timer_button = -1;     // Button1 poll timer
static int g_fd_poll_timer_ccs811_int = -1; // CCS811 interrupt pin poll timer
static int g_fd_poll_timer_upload = -1;     // Azure upload poll timer
static int g_fd_gpio_button1 = -1;          // Button1 GPIO
static int g_fd_gpio_ccs811_int = -1;       // CCS811 interrupt pin GPIO

// Button1 state storage
static GPIO_Value_Type g_state_button1 = GPIO_Value_High;

// CCS811 interrupt pin state storage
static GPIO_Value_Type g_state_ccs811_int = GPIO_Value_High;

// Event handler data
static EventData g_event_data_button = {        // Button state poll timer
    .eventHandler = &button_timer_event_handler
};
static EventData g_event_data_ccs811_int = {    // CCS811 int pin poll timer
    .eventHandler = &ccs811_int_timer_event_handler
};
static EventData g_event_data_poll_upload = {   // Azure upload timer
    .eventHandler = &upload_timer_event_handler
};

static hdc1000_t *gp_hdc;       // HDC1000 sensor data pointer
static ccs811_t *gp_ccs;        // CCS811 sensor data pointer
static u8g2_t g_u8g2;           // OLED device descriptor for u8g2

// Current data from sensors
static double g_temperature, g_humidity;
static int16_t g_eco2, g_tvoc;

// Print buffer for outputting data to display
static char g_print_buffer[OLED_LINE_LENGTH + 1];

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static bool versionStringSent = false;
#endif

/*******************************************************************************
* Function definitions
*******************************************************************************/

int 
main(int argc, char *argv[])
{
    gb_is_termination_requested = false;

	// Initialize handlers
	if (init_handlers() != 0)
	{
        // Failed to initialize handlers
        gb_is_termination_requested = true;
	}

	// Initialize peripherals
	if (!gb_is_termination_requested)
	{
		if (init_peripherals(I2C_ISU) != 0)
		{
            // Failed to initialize peripherals
            gb_is_termination_requested = true;
		}
	}

	// Main program
    if (!gb_is_termination_requested) 
    {
        // All handlers and peripherals are initialized properly at this point

        u8g2_ClearDisplay(&g_u8g2);

        // Initialize CCS811 measurement mode and enable interrupt
		ccs811_set_mode(gp_ccs, CCS811_MODE_10S);
        ccs811_enable_interrupt(gp_ccs, true);

        // Show measurement display while waiting for the first data
        display_measurements();

		// Main program loop
        while (!gb_is_termination_requested)
        {
            // Handle timers
			if (WaitForEventAndCallHandler(g_fd_epoll) != 0) 
            {
                // Timer event polling failed
                gb_is_termination_requested = true;
            }

#           if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
            // Setup the IoT Hub client.
            // Notes:
            // - it is safe to call this function even if the client has already
            //   been set up, as in this case it would have no effect
            // - a failure to setup the client is a fatal error.
            if (!AzureIoT_SetupClient(AZURE_CONNECTION_STRING)) 
            {
                Log_Debug("ERROR: Failed to set up IoT Hub client\n");
                gb_is_termination_requested = true;
            }
#           endif 

#           if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
            if (iothubClientHandle != NULL && !versionStringSent) 
            {
                checkAndUpdateDeviceTwin("versionString", argv[1], TYPE_STRING, false);
                versionStringSent = true;
            }

            // AzureIoT_DoPeriodicTasks() needs to be called frequently in order
            // to keep active data flow to the Azure IoT Hub
            AzureIoT_DoPeriodicTasks();
#           endif
        }

        u8g2_ClearDisplay(&g_u8g2);
        }

    // Clean up and shutdown
    close_peripherals_and_handlers();
}

/*******************************************************************************
* Private function definitions
*******************************************************************************/

static void
button1_press_handler(void)
{
    Log_Debug("Button1 pressed.\n");
    //gb_is_termination_requested = true;
}

static void
ccs811_interrupt_handler(void)
{
    // Read temperature and humidity from HDC1000
	g_temperature = hdc1000_get_temp(gp_hdc);
	g_humidity = hdc1000_get_humi(gp_hdc);

    Log_Debug("Temperature [degC]: %f, Humidity [percRH]: %f\n",
        g_temperature, g_humidity);

    // Feed environmental data to CCS811
    bool set_result = ccs811_set_environmental_data(gp_ccs,
        (float)g_temperature, (float)g_humidity);

    if (set_result)
    {
        // Reading CCS811 result will reset /INT pin.
        if (!ccs811_get_results(gp_ccs, &g_tvoc, &g_eco2, 0, 0)) 
        {
            Log_Debug("Could not read measurement from CCS811.\n");
            gb_is_termination_requested = true;
        }
        else
        {
            Log_Debug("CCS811 Sensor: TVOC %d ppb, eCO2 %d ppm\n", g_tvoc, g_eco2);

            // Output data on display
            display_measurements();
        }
    }
}

static void
display_measurements(void)
{
    u8g2_ClearBuffer(&g_u8g2);

    //u8g2_ClearDisplay(&g_u8g2);
    u8g2_SetFont(&g_u8g2, u8g2_font_helvB08_tf);

    lib_u8g2_DrawCenteredStr(&g_u8g2, 11, "eCO2 [ppm]");
    lib_u8g2_DrawCenteredStr(&g_u8g2, 56, "TVOC [ppb]");
    lib_u8g2_DrawCenteredStr(&g_u8g2, 101, "Humidity [%]");

    u8g2_SetFont(&g_u8g2, u8g2_font_crox4tb_tn);

    // Print eCO2 value
    if (g_eco2 > 0)
    {
        sprintf(g_print_buffer, "%d", g_eco2);
    }
    else
    {
        sprintf(g_print_buffer, "...");
    }
    lib_u8g2_DrawCenteredStr(&g_u8g2, 32, g_print_buffer);

    // Print TVOC value
    if (g_eco2 > 0)
    {
        // TVOC value is valid only after eCO2 measurement is valid
        sprintf(g_print_buffer, "%d", g_tvoc);
    }
    else
    {
        sprintf(g_print_buffer, "...");
    }
    lib_u8g2_DrawCenteredStr(&g_u8g2, 77, g_print_buffer);

    // Print humidity value
    if (g_humidity > 0)
    {
        sprintf(g_print_buffer, "%.1f", g_humidity);
    }
    else
    {
        sprintf(g_print_buffer, "...");
    }
    lib_u8g2_DrawCenteredStr(&g_u8g2, 123, g_print_buffer);

    u8g2_SendBuffer(&g_u8g2);

}

static void
termination_handler(int signal_number)
{
    gb_is_termination_requested = true;
}

static void
button_timer_event_handler(EventData *event_data)
{
    // Consume timer event
    if (ConsumeTimerFdEvent(g_fd_poll_timer_button) != 0) {
        gb_is_termination_requested = true;
        return;
    }

    // Check for a button press
    GPIO_Value_Type new_g_state_button1;
    int result = GPIO_GetValue(g_fd_gpio_button1, &new_g_state_button1);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", 
            strerror(errno), errno);
        gb_is_termination_requested = true;
        return;
    }

    if (new_g_state_button1 != g_state_button1)
    {
        if (new_g_state_button1 == GPIO_Value_Low)
        {
            button1_press_handler();
        }
        g_state_button1 = new_g_state_button1;
    }
}

static void
ccs811_int_timer_event_handler(EventData *event_data)
{
    // Consume timer event
    if (ConsumeTimerFdEvent(g_fd_poll_timer_ccs811_int) != 0) {
        gb_is_termination_requested = true;
        return;
    }

    // Check for interrupt signal state change
    GPIO_Value_Type new_g_state_ccs811_int;

    int result = GPIO_GetValue(g_fd_gpio_ccs811_int, &new_g_state_ccs811_int);
    if (result != 0) {
        Log_Debug("ERROR: Could not read CCS811 interrupt GPIO: %s (%d).\n",
            strerror(errno), errno);
        gb_is_termination_requested = true;
        return;
    }

    if (new_g_state_ccs811_int != g_state_ccs811_int)
    {
        if (new_g_state_ccs811_int == GPIO_Value_Low)
        {
            // CCS811 /INT pin is asserted. New measurement is available.
            ccs811_interrupt_handler();
        }
        g_state_ccs811_int = new_g_state_ccs811_int;
    }
}

static void
upload_timer_event_handler(EventData *event_data)
{
    bool b_is_all_ok = true;

    // Consume timer event
    if (ConsumeTimerFdEvent(g_fd_poll_timer_upload) != 0)
    {
        // Failed to consume timer event
        gb_is_termination_requested = true;
        b_is_all_ok = false;
    }

    if (b_is_all_ok)
    {
        // Request measurement data from all sources
        azure_upload_handler();
    }

    return;
}

static void
azure_upload_handler(void)
{
#   if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
    char *p_buffer_json;

    if ((p_buffer_json = malloc(JSON_BUFFER_SIZE)) == NULL)
    {
        Log_Debug("ERROR: not enough memory for upload buffer.\n");
    }
    else
    {
        // Construct Azure upload message
        snprintf(p_buffer_json, JSON_BUFFER_SIZE,
            "{\"eco2\":\"%d\", \"tvoc\":\"%d\", "
            "\"temperature\":\"%.1f\", \"humidity\":\"%.1f\"}",
            g_eco2, g_tvoc, g_temperature, g_humidity);

        Log_Debug("Uploading to Azure: %s\n", p_buffer_json);
        AzureIoT_SendMessage(p_buffer_json);
        free(p_buffer_json);
    }
#   endif

    return;
}

static int
init_handlers(void)
{
    int result = -1;

    Log_Debug("Init Handlers\n");
    
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = termination_handler;
    result = sigaction(SIGTERM, &action, NULL);
    if (result != 0) {
        Log_Debug("ERROR: %s - sigaction: errno=%d (%s)\n", 
            __FUNCTION__, errno, strerror(errno));
    }

    g_fd_epoll = CreateEpollFd();
    if (g_fd_epoll < 0) {
        result = -1;
    }

    // Create poll timer for Azure upload
    if (result != -1)
    {
        g_fd_poll_timer_upload = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &AZURE_UPLOAD_PERIOD, &g_event_data_poll_upload, EPOLLIN);
        if (g_fd_poll_timer_upload < 0)
        {
            // Failed to create Azure upload poll timer
            Log_Debug("ERROR: Could not create poll timer: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    return result;
}

static int
init_peripherals(I2C_InterfaceId isu_id)
{
    int result = -1;

    // Initialize I2C
    Log_Debug("Init I2C\n");
    g_fd_i2c = I2CMaster_Open(isu_id);
    if (g_fd_i2c < 0) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", 
            errno, strerror(errno));
    }
    else
    {
        result = I2CMaster_SetBusSpeed(g_fd_i2c, I2C_BUS_SPEED_STANDARD);
        if (result != 0) {
            Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", 
                errno, strerror(errno));
        }
        else 
        {
            result = I2CMaster_SetTimeout(g_fd_i2c, 100);
            if (result != 0) {
                Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", 
                    errno, strerror(errno));
            }
        }
    }

    // Initialize HDC1000 Click board
    // Default sensor I2C address, not using DRDYn signal
    if (result != -1)
    {
        Log_Debug("Init HDC1000\n");
        gp_hdc = hdc1000_open(g_fd_i2c, HDC1000_I2C_ADDR, -1);
        if (!gp_hdc)
        {
            Log_Debug("ERROR: Cannot initialize HDC1000 sensor.\n");
            result = -1;
        }
    }

    // Initialize Air Quality 3 Click board (CCS811 sensor)
    // Default sensor I2C address, located in  Socket1
    if (result != -1)
    {
        Log_Debug("Init CCS811\n");
        gp_ccs = ccs811_open(g_fd_i2c, CCS811_I2C_ADDRESS_1, SK_SOCKET1_CS_GPIO);
        if (!gp_ccs)
        {
            Log_Debug("ERROR: Cannot initialize CCS811 sensor.\n");
            result = -1;
        }
    }

    // Initialize Air Quality 3 Click board interrupt GPIO
    // Set development kit Socket 1 & 2 INT pin as Input
    if (result != -1)
    {
        Log_Debug("Opening PROJECT_SOCKET12_INT as input.\n");
        g_fd_gpio_ccs811_int = GPIO_OpenAsInput(PROJECT_SOCKET12_INT);
        if (g_fd_gpio_ccs811_int < 0) {
            Log_Debug("ERROR: Could not open GPIO: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // Initialize 128x64 SSD1306 OLED
    if (result != -1)
    {
        Log_Debug("Initializing OLED display.\n");

        // Set lib_u8g2 I2C interface file descriptor and device address
        lib_u8g2_set_i2c(g_fd_i2c, I2C_ADDR_OLED);

        // Set display type and callbacks
        u8g2_Setup_ssd1306_i2c_128x64_noname_f(&g_u8g2, OLED_ROTATION,
            lib_u8g2_byte_i2c, lib_u8g2_custom_cb);

        // Initialize display descriptor
        u8g2_InitDisplay(&g_u8g2);

        // Wake up display
        u8g2_SetPowerSave(&g_u8g2, 0);
    }

    // Initialize development kit button GPIO
    // Open button 1 GPIO as input
    if (result != -1)
    {
        Log_Debug("Opening PROJECT_BUTTON_1 as input.\n");
        g_fd_gpio_button1 = GPIO_OpenAsInput(PROJECT_BUTTON_1);
        if (g_fd_gpio_button1 < 0) {
            Log_Debug("ERROR: Could not open button GPIO: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // Create timer for button press check
    if (result != -1)
    {
        struct timespec button_press_check_period = { 0, 1000000 };
        g_fd_poll_timer_button = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &button_press_check_period, &g_event_data_button, EPOLLIN);
        if (g_fd_poll_timer_button < 0)
        {
            Log_Debug("ERROR: Could not create button poll timer: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // Create timer for CCS811 interrupt signal check
    if (result != -1)
    {
        struct timespec ccs811_int_check_period = { 0, 250000000 };
        g_fd_poll_timer_ccs811_int = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &ccs811_int_check_period, &g_event_data_ccs811_int, EPOLLIN);
        if (g_fd_poll_timer_ccs811_int < 0)
        {
            Log_Debug("ERROR: Could not create interrupt poll timer: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    return result;
}

static void
close_peripherals_and_handlers(void)
{
    // Close CCS811 sensor
    Log_Debug("Close CCS811\n");
    if (gp_ccs)
    {
        ccs811_close(gp_ccs);
    }

    // Close HDC1000 sensor
    Log_Debug("Close HDC1000\n");
    if (gp_hdc)
    {
        hdc1000_close(gp_hdc);
    }

    // Close I2C
    CloseFdAndPrintError(g_fd_i2c, "I2C");

    // Close CCS811 interrupt GPIO fd
    CloseFdAndPrintError(g_fd_gpio_ccs811_int, "CSS811 INT GPIO");

    // Close button1 GPIO fd
    CloseFdAndPrintError(g_fd_gpio_button1, "Button1 GPIO");

    // Close Epoll fd
    CloseFdAndPrintError(g_fd_epoll, "Epoll");
}

/* [] END OF FILE */
