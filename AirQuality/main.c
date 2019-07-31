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

/**
 *
 */
static void
button_timer_event_handler(EventData *event_data);

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

// Termination state flag
static volatile sig_atomic_t gb_is_termination_requested = false;
static int i2c_fd = -1;                     // I2C file descriptor
static int epoll_fd = -1;                   // Epoll file descriptor
static int button_poll_timer_fd = -1;
static int button1_gpio_fd = -1;
static hdc1000_t *p_hdc;                    // HDC1000 sensor data pointer
static ccs811_t *p_ccs;                     // CCS811 sensor data pointer

static GPIO_Value_Type button1_state = GPIO_Value_High;


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

    if (!gb_is_termination_requested) 
    {
        // Main application

        Log_Debug("Waiting for event\n");
        while (!gb_is_termination_requested)
        {
            if (WaitForEventAndCallHandler(epoll_fd) != 0) 
            {
                gb_is_termination_requested = true;
            }
        }
        Log_Debug("Not waiting for event\n");

        struct timespec sleepTime;
        sleepTime.tv_sec = 1;
        sleepTime.tv_nsec = 0;

        double ddata;
        uint16_t tvoc;
        uint16_t eco2;

        ccs811_set_mode(p_ccs, CCS811_MODE_10S);
        nanosleep(&sleepTime, NULL);

        for (int meas = 0; meas < 300; meas++)
        {
            // HDC1000
            ddata = hdc1000_get_temp(p_hdc);
            Log_Debug("Temperature [degC]: %f\n", ddata);

            ddata = hdc1000_get_humi(p_hdc);
            Log_Debug("Humidity [percRH]: %f\n", ddata);

            // CCS811
            if (ccs811_get_results(p_ccs, &tvoc, &eco2, 0, 0)) {
                Log_Debug("CCS811 Sensor periodic: TVOC %d ppb, eCO2 %d ppm\n",
                    tvoc, eco2);
            }
            else
            {
                Log_Debug("No results\n");
            }

            nanosleep(&sleepTime, NULL);
        }

    }

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
        Log_Debug("Cannot consume time event.\n");
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
        }
        button1_state = new_button1_state;
    }

}

// Event handler data. Only the event handler field needs to be populated.
static EventData button_event_data = {
    .eventHandler = &button_timer_event_handler
};


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
    // Using default sensor I2C address and not using DRDYn signal
    Log_Debug("Init HDC1000\n");
    p_hdc = hdc1000_open(i2c_fd, HDC1000_I2C_ADDR, -1);
    if (!p_hdc)
    {
        Log_Debug("ERROR: Cannot initialize HDC1000 sensor.\n");
        result = -1;
    }

    // Initialize Air Quality 3 Click board (CCS811 sensor)
    // Using default sensor I2C address and Socket1 signals
    Log_Debug("Init CCS811\n");
    p_ccs = ccs811_open(i2c_fd, CCS811_I2C_ADDRESS_1, SK_SOCKET1_CS_GPIO);
    if (!p_ccs)
    {
        Log_Debug("ERROR: Cannot initialize CCS811 sensor.\n");
        result = -1;
    }

    // Initialize Air Quality 3 Click board interrupt GPIO

    // Initialize 128x64 OLED

    // Initialize development kit button GPIO
    // Open button GPIO as input
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
    Log_Debug("Close I2C\n");
    if (i2c_fd > 0)
    {
        close(i2c_fd);
    }

    // Close button1 GPIO
    CloseFdAndPrintError(button1_gpio_fd, "Button1 GPIO");

    // Close Epoll
    CloseFdAndPrintError(epoll_fd, "Epoll");
}

/* [] END OF FILE */
