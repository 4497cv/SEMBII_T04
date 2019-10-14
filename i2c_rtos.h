/*
 * i2c_rtoc.h
 *
 *  Created on: Oct 13, 2019
 *      Author: LuisFernando
 */

#ifndef I2C_RTOS_H_
#define I2C_RTOS_H_


#include "gpio.h"

#include "FREErtos.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
/*Events and semaphore*/
#include "event_groups.h"
#include "semphr.h"

#define NUMBER_OF_SERIAL_PORTS (3U)

#define I2C_WRITE_DELAY (120000000U)

#define I2C_DEVICE_ADDRESS (0U)
#define I2C_SUBADDRESS (1U)
#define I2C_DATA_SIZE (1U)

typedef struct
{
	uint8_t is_init_flag;
	SemaphoreHandle_t mutex_sda;
	SemaphoreHandle_t mutex_scl;
	SemaphoreHandle_t sda_sem;
	SemaphoreHandle_t scl_sem;
}rtos_i2c_hanlde_t;

typedef enum
{
	rtos_i2c_portA,
	rtos_i2c_portB,
	rtos_i2c_portC,
	rtos_i2c_portD,
	rtos_i2c_portE
} rtos_i2c_port_t;
typedef enum
{
	rtos_i2c_0,
	rtos_i2c_1,
	rtos_i2c_2
} rtos_i2c_number_t;

typedef enum
{
	rtos_i2c_fail,
	rtos_i2c_success
} rtos_i2c_flag_t;

typedef struct
{
	uint32_t  baudrate;
	rtos_i2c_number_t i2c_number;
	rtos_i2c_port_t port;
	uint8_t sda_pin;
	uint8_t scl_pin;
	uint8_t pin_mux;
}rtos_i2c_config_t;



SemaphoreHandle_t g_mutex;

static rtos_i2c_hanlde_t i2c_handles[NUMBER_OF_SERIAL_PORTS] = {0};

volatile uint8_t g_master_send_complete_flag = FALSE;

static inline void enable_port_clock(rtos_i2c_port_t);

static inline I2C_Type * get_i2c_base(rtos_i2c_number_t);

static inline PORT_Type * get_port_base(rtos_i2c_port_t);

static void i2c_master_callback(I2C_Type *base,
								i2c_master_handle_t *handle,
								status_t status,
								void * userData);

#endif /* I2C_RTOS_H_ */
