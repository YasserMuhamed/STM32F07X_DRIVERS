#ifndef __HAL_UART_DRIVER_H
#define __HAL_UART_DRIVER_H 

#include"stm32f407xx.h"
#include"led.h"
typedef enum 
{
	HAL_UART_STATE_RESET			=0x00,
	HAL_UART_STATE_READY			=0x01,
	HAL_UART_STATE_BUSY				=0x02,
	HAL_UART_STATE_BUSY_TX		=0x12,
	HAL_UART_STATE_BUSY_RX		=0x22,
	HAL_UART_STATE_BUSY_TX_RX	=0x32,
}hal_uart_state_t;

/* UART possible error codes */

#define HAL_UART_ERROR_NONE				  ((uint32_t)0x00000000)/*! <No error */
#define HAL_UART_ERROR_PE					  ((uint32_t)0x00000001)/*! <Parity error */
#define HAL_UART_ERROR_NE					  ((uint32_t)0x00000002)/*! <Noise error */
#define HAL_UART_ERROR_FE					  ((uint32_t)0x00000004)/*! <Frame error */
#define HAL_UART_ERROR_ORE					((uint32_t)0x00000008)/*! <Overrun error */
#define HAL_UART_ERROR_DMA					((uint32_t)0x00000010)/*! <DMA transfer error */


/*Different USART and UART peripheral base address */

#define USART_1 USART1
#define USART_2 USART2
#define USART_3 USART3
#define USART_4 USART4
#define USART_5 USART5
#define USART_6 USART6

/* Macros to enable the clocks for various UART */

#define _HAL_RCC_USART1_CLK_ENABLE()			(RCC->APB2ENR |=(1<<4))
#define _HAL_RCC_USART2_CLK_ENABLE()			(RCC->APB1ENR |=(1<<17))
#define _HAL_RCC_USART3_CLK_ENABLE()			(RCC->APB1ENR |=(1<<18))
#define _HAL_RCC_UART4_CLK_ENABLE()			  (RCC->APB1ENR |=(1<<19))
#define _HAL_RCC_UART5_CLK_ENABLE()			  (RCC->APB1ENR |=(1<<20))
#define _HAL_RCC_USART6_CLK_ENABLE()			(RCC->APB2ENR |=(1<<5))


/* Bit definitions for USART_SR 	register */

#define USART_REG_SR_TXE_FLAG							  ((uint32_t)(1<<7))
#define USART_REG_SR_TC_FLAG							  ((uint32_t)(1<<6))
#define USART_REG_SR_RXNE_FLAG							((uint32_t)(1<<5))
#define USART_REG_SR_IDLE_FLAG							((uint32_t)(1<<4))
#define USART_REG_SR_ORE_FLAG							  ((uint32_t)(1<<3))
#define USART_REG_SR_NE_FLAG						  	((uint32_t)(1<<2))
#define USART_REG_SR_FE_FLAG						  	((uint32_t)(1<<1))
#define USART_REG_SR_PE_FLAG							  ((uint32_t)(1<<0))

/*Bit definition for USART_BRR register*/

#define USART_REG_BRR_MANTISSA						  	((uint32_t)(1<<1))
#define USART_REG_BRR_FRACTION							  ((uint32_t)(1<<0))

/* Bit definition for USART_CR1   register */ 

#define USART_REG_CR1_OVERS										((uint32_t)(1<<15))
#define USART_OVERS_ENABLE										1
#define USART_OVER16_ENABLE										0

#define USART_REG_CR1_USART_EN								((uint32_t)(1<<13))
#define USART_REG_CR1_UART_WL								  ((uint32_t)(1<<12))
#define USART_WL_1S8B													0
#define USART_WL_1S9B													1


#define USART_REG_CR1_TXE_INT_EN							((uint32_t)(1<<7))
#define USART_REG_CR1_TCIE_INT_EN							((uint32_t)(1<<6))
#define USART_REG_CR1_RXNE_INT_EN							((uint32_t)(1<<5))
#define USART_REG_CR1_PEIE_INT_EN							((uint32_t)(1<<8))
#define USART_REG_CR1_TE											((uint32_t)(1<<3))
#define USART_REG_CR1_RE											((uint32_t)(1<<2))

/*  Bit definition for USART_CR3 register   */

#define USART_REG_CR3_ERR_INT_EN							((uint32_t)(1<<0))
#define UART_STOPBITS_1												((uint32_t)0x00)
#define UART_STOPBITS_HALF										((uint32_t)0x01)
#define USART_REG_CR2_STOP_BITS								12
#define USART_STOP_BITS_2											2
#define USART_STOP_BITS_HALF									1
#define USART_BAUD_9600												9600
#define USART_BAUD_115200											115200
typedef struct 
{
	uint32_t BaudRate;
	uint32_t WordLength;
	uint32_t StopBits;
	uint32_t Parity;
	uint32_t Mode;
	uint32_t OverSampling;
}
uart_init_t;

typedef void(*TX_COMP_CB_t)(void);
typedef void(*RX_COMP_CB_t)(void);
typedef struct 
{
	USART_TypeDef 					*Instance;
	uart_init_t							Init;
	uint8_t									*pTxBuffPtr;
	uint16_t								TxXferSize;
	uint16_t								TxXferCount;
	uint8_t									*pRxBuffPtr;
	uint16_t								RxXferSize;
	uint16_t								RxXferCount;
	hal_uart_state_t				rx_state;
	hal_uart_state_t				tx_state;
	uint32_t								ErrorCode;
	TX_COMP_CB_t						*tx_cmp_cb;/* Application call back when tx completed*/
	RX_COMP_CB_t 						*rx_cmp_cb;/* Application call back when rx completed */
}uart_handle_t;

void hal_uart_init(uart_handle_t *handle);
void hal_uart_tx(uart_handle_t *handle,uint8_t *buffer,uint32_t len);
void hal_uart_rx(uart_handle_t *handle,uint8_t *buffer,uint32_t len);
void hal_uart_handle_interrupt(uart_handle_t *huart);


#endif 