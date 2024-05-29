/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include <fcntl.h> // open()
#include <unistd.h> // close()
#include <time.h> // clock_gettime()

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>

#include "platform.h"
#include "types.h"
#include "vl53l5cx_api.h"

#define VL53L5CX_ERROR_GPIO_SET_FAIL	-1
#define VL53L5CX_COMMS_ERROR		-2
#define VL53L5CX_ERROR_TIME_OUT		-3

#define SUPPRESS_UNUSED_WARNING(x) \
	((void) (x))

#define VL53L5CX_COMMS_CHUNK_SIZE  1024

#define LOG 				printf

#ifndef STMVL53L5CX_KERNEL
static uint8_t i2c_buffer[VL53L5CX_COMMS_CHUNK_SIZE];
#else
struct comms_struct {
	uint16_t   len;
	uint16_t   reg_address;
	uint8_t    write_not_read;
	uint8_t    padding[3]; /* 64bits alignment */
	uint64_t   bufptr;
};
#endif

#define ST_TOF_IOCTL_TRANSFER           _IOWR('a',0x1, struct comms_struct)
#define ST_TOF_IOCTL_WAIT_FOR_INTERRUPT	_IO('a',0x2)

int32_t vl53l5cx_comms_init(VL53L5CX_Platform * p_platform)
{

#ifdef STMVL53L5CX_KERNEL
	p_platform->fd = open("/dev/stmvl53l5cx", O_RDONLY);
	if (p_platform->fd == -1) {
		LOG("Failed to open /dev/stmvl53l5cx\n");
		return VL53L5CX_COMMS_ERROR;
	}
#else
	/* Create sensor at default i2c address */
	p_platform->address = 0x52;
	p_platform->fd = open("/dev/i2c-1", O_RDONLY);
	if (p_platform->fd == -1) {
		LOG("Failed to open /dev/i2c-1\n");
		return VL53L5CX_COMMS_ERROR;
	}

	if (ioctl(p_platform->fd, I2C_SLAVE, p_platform->address) <0) {
		LOG("Could not speak to the device on the i2c bus\n");
		return VL53L5CX_COMMS_ERROR;
	}
#endif

	LOG("Opened ST TOF Dev = %d\n", p_platform->fd);

	return 0;
}

int32_t vl53l5cx_comms_close(VL53L5CX_Platform * p_platform)
{
	close(p_platform->fd);
	return 0;
}

int32_t write_read_multi(
		int fd,
		uint16_t i2c_address,
		uint16_t reg_address,
		uint8_t *pdata,
		uint32_t count,
		int write_not_read)
{
#ifdef STMVL53L5CX_KERNEL
	struct comms_struct cs;

	cs.len = count;
	cs.reg_address = reg_address;
	cs.bufptr = (uint64_t)(uintptr_t)pdata;
	cs.write_not_read = write_not_read;

	if (ioctl(fd, ST_TOF_IOCTL_TRANSFER, &cs) < 0)
		return VL53L5CX_COMMS_ERROR;
#else

	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];

	uint32_t data_size = 0;
	uint32_t position = 0;

	if (write_not_read) {
		do {
			data_size = (count - position) > (VL53L5CX_COMMS_CHUNK_SIZE-2) ? (VL53L5CX_COMMS_CHUNK_SIZE-2) : (count - position);

			memcpy(&i2c_buffer[2], &pdata[position], data_size);

			i2c_buffer[0] = (reg_address + position) >> 8;
			i2c_buffer[1] = (reg_address + position) & 0xFF;

			messages[0].addr = i2c_address >> 1;
			messages[0].flags = 0; //I2C_M_WR;
			messages[0].len = data_size + 2;
			messages[0].buf = i2c_buffer;

			packets.msgs = messages;
			packets.nmsgs = 1;

			if (ioctl(fd, I2C_RDWR, &packets) < 0)
				return VL53L5CX_COMMS_ERROR;
			position +=  data_size;

		} while (position < count);
	}

	else {
		do {
			data_size = (count - position) > VL53L5CX_COMMS_CHUNK_SIZE ? VL53L5CX_COMMS_CHUNK_SIZE : (count - position);

			i2c_buffer[0] = (reg_address + position) >> 8;
			i2c_buffer[1] = (reg_address + position) & 0xFF;

			messages[0].addr = i2c_address >> 1;
			messages[0].flags = 0; //I2C_M_WR;
			messages[0].len = 2;
			messages[0].buf = i2c_buffer;

			messages[1].addr = i2c_address >> 1;
			messages[1].flags = I2C_M_RD;
			messages[1].len = data_size;
			messages[1].buf = pdata + position;

			packets.msgs = messages;
			packets.nmsgs = 2;

			if (ioctl(fd, I2C_RDWR, &packets) < 0)
				return VL53L5CX_COMMS_ERROR;

			position += data_size;

		} while (position < count);
	}

#endif
	return 0;
}

int32_t write_multi(
		int fd,
		uint16_t i2c_address,
		uint16_t reg_address,
		uint8_t *pdata,
		uint32_t count)
{
	return(write_read_multi(fd, i2c_address, reg_address, pdata, count, 1));
}

int32_t read_multi(
		int fd,
		uint16_t i2c_address,
		uint16_t reg_address,
		uint8_t *pdata,
		uint32_t count)
{
	return(write_read_multi(fd, i2c_address, reg_address, pdata, count, 0));
}

uint8_t RdByte(
		VL53L5CX_Platform * p_platform,
		uint16_t reg_address,
		uint8_t *p_value)
{
	return(read_multi(p_platform->fd, p_platform->address, reg_address, p_value, 1));
}

uint8_t WrByte(
		VL53L5CX_Platform * p_platform,
		uint16_t reg_address,
		uint8_t value)
{
	return(write_multi(p_platform->fd, p_platform->address, reg_address, &value, 1));
}

uint8_t RdMulti(
		VL53L5CX_Platform * p_platform,
		uint16_t reg_address,
		uint8_t *p_values,
		uint32_t size)
{
	return(read_multi(p_platform->fd, p_platform->address, reg_address, p_values, size));
}

uint8_t WrMulti(
		VL53L5CX_Platform * p_platform,
		uint16_t reg_address,
		uint8_t *p_values,
		uint32_t size)
{
	return(write_multi(p_platform->fd, p_platform->address, reg_address, p_values, size));
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t WaitMs(
		VL53L5CX_Platform * p_platform,
		uint32_t time_ms)
{
	usleep(time_ms*1000);
	return 0;
}

uint8_t wait_for_dataready(VL53L5CX_Platform *p_platform)
{
#ifdef STMVL53L5CX_KERNEL
	if (ioctl(p_platform->fd, ST_TOF_IOCTL_WAIT_FOR_INTERRUPT) < 0)
		return 0;
#else
	VL53L5CX_Configuration * p_dev = (VL53L5CX_Configuration *)(p_platform - offsetof(VL53L5CX_Configuration, platform));
	uint8_t isReady = 0;
	do {
		WaitMs(p_platform, 5);
		vl53l5cx_check_data_ready(p_dev, &isReady);		
	} while (isReady == 0);
#endif
	return 1;
}