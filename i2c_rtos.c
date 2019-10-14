/*
 * i2c_rtos.c
 *
 *  Created on: Oct 13, 2019
 *      Author: LuisFernando
 */

#include "i2c_rtos.h"

#define HANDEL

volatile uint8_t g_master_complete_flag = FALSE;

i2c_master_handle_t g_master_handle;
SemaphoreHandle_t g_i2c_mutex_send;


static inline void enable_port_clock(rtos_i2c_port_t port)
{
	switch(port)
	{
	case rtos_i2c_portA:
		CLOCK_EnableClock(kCLOCK_PortA);
		break;
	case rtos_i2c_portB:
		CLOCK_EnableClock(kCLOCK_PortB);
		break;
	case rtos_i2c_portC:
		CLOCK_EnableClock(kCLOCK_PortC);
		break;
	case rtos_i2c_portD:
		CLOCK_EnableClock(kCLOCK_PortD);
		break;
	case rtos_i2c_portE:
		CLOCK_EnableClock(kCLOCK_PortE);
		break;
	default:
		break;
	}
}

static inline I2C_Type * get_i2c_base(rtos_i2c_number_t rtos_number)
{
	I2C_Type * retval = I2C0;
	switch(rtos_number)
	{
	case rtos_i2c_0:
		retval = I2C0;
		break;
	case rtos_i2c_1:
		retval = I2C1;
		break;
	case rtos_i2c_2:
		retval = I2C2;
		break;
	default:
		break;
	}
	return retval;
}

static inline PORT_Type * get_port_base(rtos_i2c_port_t port)
{
	PORT_Type * port_base = PORTA;
	switch(port)
	{
	case rtos_i2c_portA:
		port_base = PORTA;
		break;
	case rtos_i2c_portB:
		port_base = PORTB;
		break;
	case rtos_i2c_portC:
		port_base = PORTC;
		break;
	case rtos_i2c_portD:
		port_base = PORTD;
		break;
	case rtos_i2c_portE:
		port_base = PORTE;
		break;
	default:
		break;
	}
	return port_base;
}

static void i2c_master_callback(I2C_Type *base,
								i2c_master_handle_t *handle,
								status_t status,
								void * userData)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(kStatus_I2C_Idle == status)
	{
		if(I2C0 == base)
		{
			xSemaphoreGiveFromISR(i2c_handles[rtos_i2c_0].sda_sem,&xHigherPriorityTaskWoken);
		}
		else if(I2C1 == base)
		{
			xSemaphoreGiveFromISR(i2c_handles[rtos_i2c_1].sda_sem,&xHigherPriorityTaskWoken);
		}
		else if(I2C2 == base)
		{
			xSemaphoreGiveFromISR(i2c_handles[rtos_i2c_2].sda_sem,&xHigherPriorityTaskWoken);
		}
		else
		{
			/*
			 * Do Nothing
			 */
		}
	}
}

rtos_i2c_flag_t rtos_i2c_init(rtos_i2c_config_t config)
{
	rtos_i2c_flag_t retval = rtos_i2c_fail;
	i2c_master_config_t i2c_master_config;

	if(config.i2c_number < NUMBER_OF_SERIAL_PORTS)
	{
		if(!i2c_handles[config.i2c_number].is_init_flag)
		{
			i2c_handles[config.i2c_number].mutex_sda = xSemaphoreCreateMutex();
			i2c_handles[config.i2c_number].mutex_scl = xSemaphoreCreateMutex();

			i2c_handles[config.i2c_number].sda_sem = xSemaphoreCreateBinary();
			i2c_handles[config.i2c_number].scl_sem = xSemaphoreCreateBinary();

			enable_port_clock(config.port);
			PORT_SetPinMux(get_port_base(config.port), config.sda_pin, config.pin_mux);
			PORT_SetPinMux(get_port_base(config.port), config.scl_pin, config.pin_mux);

			I2C_MasterGetDefaultConfig(&i2c_master_config);

			i2c_master_config.baudRate_Bps = config.baudrate;
			i2c_master_config.enableMaster = TRUE;

			if(rtos_i2c_0 == config.i2c_number)
			{
				I2C_MasterInit(get_i2c_base(rtos_i2c_0),&i2c_master_config, CLOCK_GetFreq(kCLOCK_BusClk));
				NVIC_SetPriority(I2C0_IRQn,5);
			}
			else if(rtos_i2c_1 == config.i2c_number)
			{
				I2C_MasterInit(get_i2c_base(rtos_i2c_0),&i2c_master_config, CLOCK_GetFreq(kCLOCK_BusClk));
				NVIC_SetPriority(I2C1_IRQn,5);
			}
			else if(rtos_i2c_2 == config.i2c_number)
			{
				I2C_MasterInit(get_i2c_base(rtos_i2c_0),&i2c_master_config, CLOCK_GetFreq(kCLOCK_BusClk));
				NVIC_SetPriority(I2C2_IRQn,5);
			}
			else
			{
				/*
				 * Do Nothing
				 */
			}

			i2c_handles[config.i2c_number].is_init_flag = TRUE;
			retval = rtos_i2c_success;
		}
	}
	return retval;
}

rtos_i2c_flag_t rtos_i2c_send(rtos_i2c_config_t config, uint16_t address, uint8_t *buffer)
{
	rtos_i2c_flag_t flag = rtos_i2c_fail;
	uint8_t *buffer_temp = buffer;
	i2c_master_transfer_t i2c_x_transfer;


	if(i2c_handles[config.i2c_number].is_init_flag)
	{
#ifdef HANDEL
		switch(config.i2c_number)
		{
		case rtos_i2c_0:
			I2C_MasterTransferCreateHandle(I2C0, &g_master_handle,i2c_master_callback, NULL);
			break;
		case rtos_i2c_1:
			I2C_MasterTransferCreateHandle(I2C1, &g_master_handle,i2c_master_callback, NULL);
			break;
		case rtos_i2c_2:
			I2C_MasterTransferCreateHandle(I2C2, &g_master_handle,i2c_master_callback, NULL);
			break;
		default:
			break;
		}
#endif
		/**/
		xSemaphoreTake(i2c_handles[config.i2c_number].mutex_sda,portMAX_DELAY);
		/*I2C protocol*/
		i2c_x_transfer.slaveAddress = I2C_DEVICE_ADDRESS;
		i2c_x_transfer.direction = kI2C_Write;
		i2c_x_transfer.subaddress = address;
		i2c_x_transfer.subaddressSize = I2C_SUBADDRESS;
		i2c_x_transfer.data = buffer_temp;
		i2c_x_transfer.dataSize = I2C_DATA_SIZE;
		i2c_x_transfer.flags = kI2C_TransferDefaultFlag;

		switch(config.i2c_number)
		{
		case rtos_i2c_0:
			I2C_MasterTransferNonBlocking(
										I2C0,
										&g_master_handle,
										NULL);
			break;
		case rtos_i2c_1:
			I2C_MasterTransferNonBlocking(
										I2C1,
										&g_master_handle,
										NULL);
			break;
		case rtos_i2c_2:
			I2C_MasterTransferNonBlocking(
										I2C2,
										&g_master_handle,
										NULL);
			break;
		default:
			break;
		}

		/**/
		xSemaphoreTake(i2c_handles[config.i2c_number].sda_sem,portMAX_DELAY);
		/**/
		xSemaphoreGive(i2c_handles[config.i2c_number].mutex_sda);

		flag = rtos_i2c_success;

	}

	return flag;
}

uint8_t rtos_i2c_recieve(rtos_i2c_config_t config, uint16_t address)
{
	uint8_t buffer = 0;
	i2c_master_transfer_t i2c_x_transfer;

	if(i2c_handles[config.i2c_number].is_init_flag)
	{
#ifdef HANDEL
		if(i2c_handles[config.i2c_number].is_init_flag)
		{

			switch(config.i2c_number)
			{
			case rtos_i2c_0:
				I2C_MasterTransferCreateHandle(I2C0, &g_master_handle,i2c_master_callback, NULL);
				break;
			case rtos_i2c_1:
				I2C_MasterTransferCreateHandle(I2C1, &g_master_handle,i2c_master_callback, NULL);
				break;
			case rtos_i2c_2:
				I2C_MasterTransferCreateHandle(I2C2, &g_master_handle,i2c_master_callback, NULL);
				break;
			default:
				break;
			}
		}
#endif
		/**/
		xSemaphoreTake(i2c_handles[config.i2c_number].mutex_sda,portMAX_DELAY);
		/*I2C protocol*/
		i2c_x_transfer.slaveAddress = I2C_DEVICE_ADDRESS;
		i2c_x_transfer.direction = kI2C_Write;
		i2c_x_transfer.subaddress = address;
		i2c_x_transfer.subaddressSize = I2C_SUBADDRESS;
		i2c_x_transfer.data = &buffer;
		i2c_x_transfer.dataSize = I2C_DATA_SIZE;
		i2c_x_transfer.flags = kI2C_TransferDefaultFlag;

		switch(config.i2c_number)
		{
		case rtos_i2c_0:
			I2C_MasterTransferNonBlocking(I2C0, &g_master_handle,&i2c_x_transfer);
			break;
		case rtos_i2c_1:
			I2C_MasterTransferNonBlocking(I2C1, &g_master_handle,&i2c_x_transfer);
			break;
		case rtos_i2c_2:
			I2C_MasterTransferNonBlocking(I2C2, &g_master_handle,&i2c_x_transfer);
			break;
		default:
			break;
		}

		/**/
		xSemaphoreTake(i2c_handles[config.i2c_number].sda_sem,portMAX_DELAY);
		/**/
		xSemaphoreGive(i2c_handles[config.i2c_number].mutex_sda);
	}
	return buffer;
}
