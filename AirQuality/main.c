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

/** @brief Initialize signal handlers.
 *
 * Set up SIGTERM termination handler.
 *
 * @return 0 on success, errno otherwise.
 */
static int
init_handlers(void);

/**
 *
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
static int i2c_fd = -1;     // I2C file descriptor

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

static int
init_handlers(void)
{
    int result = -1;
    
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = termination_handler;
    result = sigaction(SIGTERM, &action, NULL);
    if (result != 0) {
        Log_Debug("ERROR: %s - sigaction: errno=%d (%s)\n", 
            __FUNCTION__, errno, strerror(errno));
    }

    return result;
}

static int
init_peripherals(I2C_InterfaceId isu_id)
{
    int result = -1;

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
    return result;
}

static void
close_peripherals_and_handlers(void)
{
    if (i2c_fd > 0)
    {
        close(i2c_fd);
    }
}

/* [] END OF FILE */
