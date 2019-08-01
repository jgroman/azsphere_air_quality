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

#include "applibs_versions.h"   // API struct versions to use for applibs APIs
#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/i2c.h>

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

// Using a single-thread event loop pattern based on Epoll and timerfd
#include "epoll_timerfd_utilities.h"

#include "LibCcs811.h"
#include "LibHdc1000.h"
#include "LibOledSsd1306.h"

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

#define OLED_LINE_LENGTH    16

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

static hdc1000_t *p_hdc;                    // HDC1000 sensor data pointer
static ccs811_t *p_ccs;                     // CCS811 sensor data pointer
static u8x8_t u8x8;                         // OLED control structure

static char print_buffer[OLED_LINE_LENGTH + 1];

/*******************************************************************************
* Function definitions
*******************************************************************************/

int 
main(void)
{
    gb_is_termination_requested = true;

    if (init_handlers() == 0)
    {
        if (init_peripherals(PROJECT_ISU2_I2C) == 0) 
        {
            // Handlers and peripherals are initialized
            gb_is_termination_requested = false;
        }
    }

    // Setup OLED
    u8x8_InitDisplay(&u8x8);
    u8x8_SetPowerSave(&u8x8, 0);
    u8x8_ClearDisplay(&u8x8);
    u8x8_SetFont(&u8x8, u8x8_font_amstrad_cpc_extended_f);

    snprintf(print_buffer, 16, " ... STARTING ...");
    u8x8_DrawString(&u8x8, 0, 0, print_buffer);


    if (!gb_is_termination_requested) 
    {
        // Main application

        ccs811_set_mode(p_ccs, CCS811_MODE_10S);
        ccs811_enable_interrupt(p_ccs, true);

        Log_Debug("Waiting for event\n");
        while (!gb_is_termination_requested)
        {
            if (WaitForEventAndCallHandler(epoll_fd) != 0) 
            {
                gb_is_termination_requested = true;
            }
        }
        Log_Debug("Not waiting for event anymore\n");
    }

    u8x8_ClearDisplay(&u8x8);

    close_peripherals_and_handlers();
}

/*******************************************************************************
* Private function definitions
*******************************************************************************/

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
            Log_Debug("Button1 pressed.\n");
            gb_is_termination_requested = true;
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
            // Reading CCS811 result will reset /INT pin.

            double temperature;
            double humidity;

            // Read temperature and humidity from HDC1000
            temperature = hdc1000_get_temp(p_hdc);
            humidity = hdc1000_get_humi(p_hdc);

            Log_Debug("Temperature [degC]: %f, Humidity [percRH]: %f\n",
                temperature, humidity);

            // Feed environmental data to CCS811
            bool set_result = ccs811_set_environmental_data(p_ccs, 
                (float)temperature, (float)humidity);

            if (set_result)
            {
                uint16_t tvoc;
                uint16_t eco2;

                if (ccs811_get_results(p_ccs, &tvoc, &eco2, 0, 0)) {
                    Log_Debug("CCS811 Sensor interrupt: TVOC %d ppb, eCO2 %d ppm\n",
                        tvoc, eco2);

                    // Output data on OLED
                    snprintf(print_buffer, 16, "Tmp: %.1f C         ", temperature);
                    u8x8_DrawString(&u8x8, 0, 0, print_buffer);
                    snprintf(print_buffer, 16, "Hum: %.1f RH         ", humidity);
                    u8x8_DrawString(&u8x8, 0, 2, print_buffer);
                    snprintf(print_buffer, 16, "eCO2: %d ppm         ", eco2);
                    u8x8_DrawString(&u8x8, 0, 4, print_buffer);
                    snprintf(print_buffer, 16, "TVOC: %d ppb         ", tvoc);
                    u8x8_DrawString(&u8x8, 0, 6, print_buffer);

                }
                else
                {
                    Log_Debug("Could not read measurement from CCS811.\n");
                    gb_is_termination_requested = true;
                }
            }
        }
        ccs811_int_state = new_ccs811_int_state;
    }

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
        p_hdc = hdc1000_open(i2c_fd, HDC1000_I2C_ADDR, -1);
        if (!p_hdc)
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
        p_ccs = ccs811_open(i2c_fd, CCS811_I2C_ADDRESS_1, SK_SOCKET1_CS_GPIO);
        if (!p_ccs)
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

    // Initialize 128x64 OLED
    if (result != -1)
    {
        u8x8_Setup(&u8x8, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_i2c, 
            u8x8_byte_i2c, mt3620_gpio_and_delay_cb);
        u8x8_SetI2CAddress(&u8x8, 0x3C);
        set_oled_i2c_fd(i2c_fd);
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
    if (p_ccs)
    {
        ccs811_close(p_ccs);
    }

    // Close HDC1000 sensor
    Log_Debug("Close HDC1000\n");
    if (p_hdc)
    {
        hdc1000_close(p_hdc);
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
