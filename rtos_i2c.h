/*
 * rtos_i2c.h
 *
 *  Created on: 13 oct 2019
 *      Author: Luis Fernando
 */

#ifndef RTOS_I2C_H_
#define RTOS_I2C_H_

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "FREErtos.h"
#include "event_groups.h"
#include "semphr.h"

#define I2C_BAUDRATE (100000U)

#define I2C_SLAVE_ADDR (0x00U)

#endif /* RTOS_I2C_H_ */
