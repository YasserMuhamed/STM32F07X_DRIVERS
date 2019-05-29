#ifndef __SPI_MAIN_H
#define __SPI_MAIN_H

#include"stm32f407xx.h"
#include"gpio.h"
#include "spi.h"


#define SPI_MISO_PIN_14	14
#define SPI_MISO_PIN_15	15
#define CMD_LENGTH			16
#define DATA_LENGTH			4

#define CMD_MASTER_READ			((uint16_t)0x1234)
#define CMD_MASTER_WRITE		((uint16_t)0x5678)
#define CMD_LEN							2
#define SPI_ACK_BYTES				0xD5E5

#define SPIx_IRQn 			SPI2_IRQn
#define SPIx_IRQHandler	SPI2_IRQHandler


#define GPIO_BUTTON_PIN		0
#define GPIO_BUTTON_PORT	GPIOA

#define GPIOB_PIN_13			13






#endif