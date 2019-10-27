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

#define I2C_ADDR_OLED       (0x3C)
#define JSON_BUFFER_SIZE    128

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

/** @brief Button1 press handler
 *
 */
static void
button1_press_handler(void);

static void
ccs811_interrupt_handler(void);

/** @brief Timer event handler for polling button states
 *
 */
static void
button_timer_event_handler(EventData *event_data);

/** @brief Timer event handler for polling CCS811 interrupt pin
 *
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

/** @brief Initialize signal handlers.
 *
 * Set up SIGTERM termination handler.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_handlers(void);

/** @brief Initialize peripherals.
 *
 * Initialize all peripherals used by this project.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_peripherals(I2C_InterfaceId isu_id);

/**
 *
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
static int i2c_fd = -1;                     
static int epoll_fd = -1;

static int button_poll_timer_fd = -1;
static int button1_gpio_fd = -1;
static GPIO_Value_Type button1_state = GPIO_Value_High;
static EventData button_event_data = {          // Event handler data
    .eventHandler = &button_timer_event_handler // Populate only this field
};

static int ccs811_int_poll_timer_fd = -1;
static int ccs811_int_gpio_fd = -1;
static GPIO_Value_Type ccs811_int_state = GPIO_Value_High;
static EventData ccs811_int_event_data = {
    .eventHandler = &ccs811_int_timer_event_handler
};

static int g_fd_poll_timer_upload = -1;
static EventData g_event_data_poll_upload = {        // Azure upload Event data
    .eventHandler = &upload_timer_event_handler
};

static hdc1000_t *gp_hdc;                   // HDC1000 sensor data pointer
static ccs811_t *gp_ccs;                    // CCS811 sensor data pointer
static u8x8_t gp_u8x8;                      // OLED control structure

// Current data from sensors
static double g_temperature, g_humidity;
static int16_t g_eco2, g_tvoc;

#define OLED_LINE_LENGTH    16
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
		gb_is_termination_requested = true;
	}

	// Initialize peripherals
	if (!gb_is_termination_requested)
	{
		if (init_peripherals(PROJECT_ISU2_I2C) != 0)
		{
			gb_is_termination_requested = true;
		}
	}

	// Initialize OLED
	if (!gb_is_termination_requested)
	{
		u8x8_ClearDisplay(&gp_u8x8);
		u8x8_SetFont(&gp_u8x8, u8x8_font_amstrad_cpc_extended_f);

		snprintf(g_print_buffer, 16, ".. STARTING ..");
		u8x8_DrawString(&gp_u8x8, 0, 0, g_print_buffer);
	}

	// Main program
    if (!gb_is_termination_requested) 
    {
		// Initialize CCS811 measurement mode and enable interrupt
		ccs811_set_mode(gp_ccs, CCS811_MODE_10S);
        ccs811_enable_interrupt(gp_ccs, true);

        Log_Debug("Waiting for events\n");

		// Main program loop
        while (!gb_is_termination_requested)
        {
            // Handle timers
			if (WaitForEventAndCallHandler(epoll_fd) != 0) 
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
        Log_Debug("Not waiting for event anymore\n");

        u8x8_ClearDisplay(&gp_u8x8);
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
    gb_is_termination_requested = true;
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

            // Output data on OLED
            snprintf(g_print_buffer, 16, "eCO2: %d ppm         ", g_eco2);
            u8x8_DrawString(&gp_u8x8, 0, 0, g_print_buffer);
            snprintf(g_print_buffer, 16, "TVOC: %d ppb         ", g_tvoc);
            u8x8_DrawString(&gp_u8x8, 0, 2, g_print_buffer);
            snprintf(g_print_buffer, 16, "Temp: %.1f C         ", g_temperature);
            u8x8_DrawString(&gp_u8x8, 0, 4, g_print_buffer);
            snprintf(g_print_buffer, 16, "Humi: %.1f RH        ", g_humidity);
            u8x8_DrawString(&gp_u8x8, 0, 6, g_print_buffer);
        }
    }
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
    if (ConsumeTimerFdEvent(button_poll_timer_fd) != 0) {
        gb_is_termination_requested = true;
        return;
    }

    // Check for a button press
    GPIO_Value_Type new_button1_state;
    int result = GPIO_GetValue(button1_gpio_fd, &new_button1_state);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", 
            strerror(errno), errno);
        gb_is_termination_requested = true;
        return;
    }

    if (new_button1_state != button1_state)
    {
        if (new_button1_state == GPIO_Value_Low)
        {
            button1_press_handler();
        }
        button1_state = new_button1_state;
    }
}

static void
ccs811_int_timer_event_handler(EventData *event_data)
{
    // Consume timer event
    if (ConsumeTimerFdEvent(ccs811_int_poll_timer_fd) != 0) {
        gb_is_termination_requested = true;
        return;
    }

    // Check for interrupt signal state change
    GPIO_Value_Type new_ccs811_int_state;

    int result = GPIO_GetValue(ccs811_int_gpio_fd, &new_ccs811_int_state);
    if (result != 0) {
        Log_Debug("ERROR: Could not read CCS811 interrupt GPIO: %s (%d).\n",
            strerror(errno), errno);
        gb_is_termination_requested = true;
        return;
    }

    if (new_ccs811_int_state != ccs811_int_state)
    {
        if (new_ccs811_int_state == GPIO_Value_Low)
        {
            // CCS811 /INT pin is asserted. New measurement is available.
            ccs811_interrupt_handler();
        }
        ccs811_int_state = new_ccs811_int_state;
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

    epoll_fd = CreateEpollFd();
    if (epoll_fd < 0) {
        result = -1;
    }

    // Create poll timer for Azure upload
    if (result != -1)
    {
        g_fd_poll_timer_upload = CreateTimerFdAndAddToEpoll(epoll_fd,
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
    i2c_fd = I2CMaster_Open(isu_id);
    if (i2c_fd < 0) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", 
            errno, strerror(errno));
    }
    else
    {
        result = I2CMaster_SetBusSpeed(i2c_fd, I2C_BUS_SPEED_STANDARD);
        if (result != 0) {
            Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", 
                errno, strerror(errno));
        }
        else 
        {
            result = I2CMaster_SetTimeout(i2c_fd, 100);
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
        gp_hdc = hdc1000_open(i2c_fd, HDC1000_I2C_ADDR, -1);
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
        gp_ccs = ccs811_open(i2c_fd, CCS811_I2C_ADDRESS_1, SK_SOCKET1_CS_GPIO);
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
        ccs811_int_gpio_fd = GPIO_OpenAsInput(PROJECT_SOCKET12_INT);
        if (ccs811_int_gpio_fd < 0) {
            Log_Debug("ERROR: Could not open GPIO: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // Initialize 128x64 SSD1306 OLED
    if (result != -1)
    {
        Log_Debug("Initializing OLED display.\n");

        // Setup u8x8 display type and custom callbacks
        u8x8_Setup(&gp_u8x8, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_i2c,
            lib_u8g2_byte_i2c, lib_u8g2_custom_cb);

        // Set OLED display I2C interface file descriptor and address
        lib_u8g2_set_i2c(i2c_fd, I2C_ADDR_OLED);

        // Initialize display descriptor
        u8x8_InitDisplay(&gp_u8x8);

        // Wake up display
        u8x8_SetPowerSave(&gp_u8x8, 0);
    }

    // Initialize development kit button GPIO
    // Open button 1 GPIO as input
    if (result != -1)
    {
        Log_Debug("Opening PROJECT_BUTTON_1 as input.\n");
        button1_gpio_fd = GPIO_OpenAsInput(PROJECT_BUTTON_1);
        if (button1_gpio_fd < 0) {
            Log_Debug("ERROR: Could not open button GPIO: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // Create timer for button press check
    if (result != -1)
    {
        struct timespec button_press_check_period = { 0, 1000000 };
        button_poll_timer_fd = CreateTimerFdAndAddToEpoll(epoll_fd,
            &button_press_check_period, &button_event_data, EPOLLIN);
        if (button_poll_timer_fd < 0)
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
        ccs811_int_poll_timer_fd = CreateTimerFdAndAddToEpoll(epoll_fd,
            &ccs811_int_check_period, &ccs811_int_event_data, EPOLLIN);
        if (ccs811_int_poll_timer_fd < 0)
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
    CloseFdAndPrintError(i2c_fd, "I2C");

    // Close CCS811 interrupt GPIO fd
    CloseFdAndPrintError(ccs811_int_gpio_fd, "CSS811 INT GPIO");

    // Close button1 GPIO fd
    CloseFdAndPrintError(button1_gpio_fd, "Button1 GPIO");

    // Close Epoll fd
    CloseFdAndPrintError(epoll_fd, "Epoll");
}

/* [] END OF FILE */
