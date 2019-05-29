#ifndef __I2C_H 
#define __I2C_H

#include"stm32f407xx.h"
#include<stdint.h>
#include"led.h"

#define I2C_REG_CR1_POS														((uint32_t)1<<11)
#define I2C_REG_CR1_ACK														((uint32_t)1<<10)
#define I2C_ACK_ENABLE														1
#define I2C_ACK_DISABLE														0

#define I2C_REG_CR1_STOP_GEN											((uint32_t)1<<9)
#define I2C_REG_CR1_START_GEN											((uint32_t)1<<8)


#define I2C_REG_CR1_NOSTRETCH											((uint32_t)1<<7)
#define I2C_ENABLE_CLK_STRETCH										0
#define I2C_DISABLE_CLK_STRETCH										1

#define I2C_REG_CR1_ENABLE_I2C										((uint32_t)1<<0)

#define I2C_REG_CR2_BUF_INT_ENABLE								((uint32_t)(1<<10))
#define I2C_REG_CR2_EVT_INT_ENABLE								((uint32_t)(1<<9))
#define I2C_REG_CR2_ERR_INT_ENABLE								((uint32_t)(1<<8))



#define I2C_PERIPHERAL_CLK_FREQ_2MHZ							((uint32_t)2)
#define I2C_PERIPHERAL_CLK_FREQ_3MHZ							((uint32_t)3)
#define I2C_PERIPHERAL_CLK_FREQ_4MHZ							((uint32_t)4)
#define I2C_PERIPHERAL_CLK_FREQ_5MHZ							((uint32_t)5)
#define I2C_PERIPHERAL_CLK_FREQ_6MHZ							((uint32_t)6)
#define I2C_PERIPHERAL_CLK_FREQ_7MHZ							((uint32_t)7)
#define I2C_PERIPHERAL_CLK_FREQ_8MHZ							((uint32_t)8)
#define I2C_PERIPHERAL_CLK_FREQ_9MHZ							((uint32_t)9)
#define I2C_PERIPHERAL_CLK_FREQ_10MHZ							((uint32_t)10)
#define I2C_PERIPHERAL_CLK_FREQ_11MHZ							((uint32_t)11)
#define I2C_PERIPHERAL_CLK_FREQ_12MHZ							((uint32_t)12)
#define I2C_PERIPHERAL_CLK_FREQ_13MHZ							((uint32_t)13)
#define I2C_PERIPHERAL_CLK_FREQ_14MHZ							((uint32_t)14)
#define I2C_PERIPHERAL_CLK_FREQ_15MHZ							((uint32_t)15)
#define I2C_PERIPHERAL_CLK_FREQ_16MHZ							((uint32_t)16)
	
#define I2C_REG_OAR1_ADDRMODE											((uint32_t)1<<15)
#define I2C_ADDRMODE_7BIT														0
#define I2C_ADDRMODE_10BIT													1

#define I2C_REG_OAR1_14TH_BIT											((uint32_t)1<<14)
#define I2C_REG_OAR1_7BIT_ADDRESS_POS							1

#define I2C_REG_DR_READ														((uint32_t)1<<7)


#define I2C_REG_SR1_TIMEOUT_FLAG							((uint32_t)1<<14)
#define I2C_REG_SR1_OVR_FLAG									((uint32_t)1<<11)
#define I2C_REG_SR1_AF_FAILURE_FLAG						((uint32_t)1<<10)
#define I2C_REG_SR1_ARLO_FLAG									((uint32_t)1<<9)
#define I2C_REG_SR1_BUS_ERROR_FLAG						((uint32_t)1<<8)
#define I2C_REG_SR1_TXE_FLAG									((uint32_t)1<<7)
#define I2C_REG_SR1_RXNE_FLAG									((uint32_t)1<<6)
#define I2C_REG_SR1_STOP_DETECTION_FLAG				((uint32_t)1<<4)
#define I2C_REG_SR1_BTF_FLAG									((uint32_t)1<<2)
#define I2C_REG_SR1_ADDR_FLAG									((uint32_t)1<<1)
#define I2C_REG_SR1_ADDR_SENT_FLAG						((uint32_t)1<<1)
#define I2C_REG_SR1_SB_FLAG										((uint32_t)1<<0)

#define I2C_REG_SR2_BUS_BUSY_FLAG							((uint32_t)1<<1)
#define I2C_BUS_IS_BUSY												1
#define I2C_BUS_IS_FREE												0

#define I2C_REG_SR2_MSL_FLAG									((uint32_t)1<<0)
#define I2C_MASTER_MODE												1
#define I2C_SLAVE_MODE												0

#define I2C_REG_SR2_TRA_FLAG									((uint32_t)1<<2)
#define I2C_RX_MODE														0
#define I2C_TX_MODE														1

#define I2C_REG_CCR_ENABE_FM									((uint32_t)1<<15)
#define I2C_ENABLE_SM													0
#define I2C_ENABLE_FM													1

#define I2C_REG_CCR_DUTY											((uint32_t)1<<14)
#define I2C_FM_DUTY_16BY9											1
#define I2C_FM_DUTY_2													0

#define I2C_1 																I2C1
#define I2C_2 																I2C2
#define I2C_3 																I2C3

/* Macros to Enable clock for different I2C Periperals*/

#define _HAL_RCC_I2C1_CLK_ENABLE()						(RCC->APB1ENR|=(1<<21))
#define _HAL_RCC_I2C2_CLK_ENABLE()						(RCC->APB1ENR|=(1<<22))
#define _HAL_RCC_I2C3_CLK_ENABLE()						(RCC->APB1ENR|=(1<<23))

										/* DATA STRUCTURE USED BY I2C*/
typedef enum 
{
	HAL_I2C_STATE_RESET										=0x00,
	HAL_I2C_STATE_READY										=0x01,
	HAL_I2C_STATE_BUSY										=0x02,
	HAL_I2C_STATE_BUSY_TX									=0x03,
	HAL_I2C_STATE_BUSY_RX									=0x04,
	HAL_I2C_STATE_ERROR										=0x05
}hal_i2c_state_t;

#define HAL_I2C_ERROR_NONE								((uint32_t)0x00000000)
#define HAL_I2C_ERROR_BERR								((uint32_t)0x00000001)
#define HAL_I2C_ERROR_ARLO								((uint32_t)0x00000002)
#define HAL_I2C_ERROR_AF									((uint32_t)0x00000004)
#define HAL_I2C_ERROR_OVR									((uint32_t)0x00000008)
#define HAL_I2C_ERROR_DMA									((uint32_t)0x00000010)
#define HAL_I2C_ERROR_TIMEOUT							((uint32_t)0x00000020)


typedef struct 
{
	uint32_t ClockSpeed;
	uint32_t DutyCycle;
	uint32_t OwnAddress1;
	uint32_t AddressingMode;
	uint32_t DualAddressMode;
	uint32_t OwnAddress2;
	uint32_t GeneralCallMode;
	uint32_t NoStretchMode;
	uint32_t ack_enable;
}i2c_init_t;

typedef struct 
{
	I2C_TypeDef				*Instance;
	i2c_init_t				Init;
	uint8_t 					*pBuffer;
	uint32_t					XferSize;
	uint32_t					XferCount;
	hal_i2c_state_t		State;
	uint32_t 					ErrorCode;
	
}
i2c_handle_t;

#define RESET				0
#define SET 				!RESET

static void 		hal_i2c_send_addr_first(I2C_TypeDef *i2cx,uint8_t slave_address,uint8_t read);
void static 		clear_addr_flag(I2C_TypeDef *i2cx);
static void 		i2c_wait_until_addr_set(I2C_TypeDef *i2cx);
static void 		i2c_wait_until_sb_set(I2C_TypeDef *i2cx);
static uint32_t is_bus_busy(I2C_TypeDef *i2cx);
static void 		hal_i2c_configure_buffer_interrupt(I2C_TypeDef *i2cx,uint32_t enable);
static void 		hal_i2c_configure_error_interrupt(I2C_TypeDef *i2cx,uint32_t enable);
static void 		hal_i2c_configure_evt_interrupt(I2C_TypeDef *i2cx,uint32_t enable);
static void 		hal_i2c_generate_start_condition(I2C_TypeDef *i2cx);
static void			hal_i2c_generate_stop_condition(I2C_TypeDef *i2cx);
static void 		hal_i2c_configure_ccr(I2C_TypeDef *i2cx,uint32_t pclk,uint32_t clkspeed,uint32_t duty_cycle);
static void 		hal_i2c_rise_time_configuration(I2C_TypeDef *i2cx,uint32_t pclk,uint32_t clkspeed);
static void 		hal_i2c_clk_init(I2C_TypeDef *i2cx,uint32_t clkspeed,uint32_t duty_cycle);
static void 		hal_i2c_set_fm_duty_cycle(I2C_TypeDef *i2cx,uint32_t duty_cycle);
static void 		hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx,uint32_t adr_mode);
static void 		hal_i2c_enable_peripheral(I2C_TypeDef *i2cx);
static void 		hal_i2c_disable_peripheral(I2C_TypeDef *i2cx);
static void 		hal_i2c_manage_clock_stretch(I2C_TypeDef *i2cx,uint32_t no_stretch);
static void 		hal_i2c_set_own_address1(I2C_TypeDef *i2cx,uint32_t own_address);
void 						hal_i2c_init(i2c_handle_t *handle);
void 						hal_i2c_master_tx(i2c_handle_t *handle,uint8_t slave_address,uint8_t *buffer,uint32_t len);
void 						hal_i2c_master_rx(i2c_handle_t *handle,uint8_t slave_addr,uint8_t *buffer,uint32_t len);
void 						hal_i2c_slave_tx(i2c_handle_t *handle,uint8_t *buffer,uint32_t len);
void					  hal_i2c_slave_rx(i2c_handle_t *handle,uint8_t *buffer,uint32_t len);
static void 		hal_i2c_master_handle_TXE_interrupt(i2c_handle_t	*hi2c);
static void 		hal_i2c_master_tx_handle_btf(i2c_handle_t	*hi2c);
static void 		hal_i2c_slave_rx_complt( i2c_handle_t  *hi2c);
static void 		hal_i2c_clear_stop_flag(i2c_handle_t *hi2c);
static void 		hal_i2c_clear_stop_flag(i2c_handle_t *hi2c);
static void 		hal_i2c_slave_handle_stop_condition(i2c_handle_t *hi2c);
static void 		hal_i2c_slave_tx_handle_btf(i2c_handle_t *hi2c);
static void		  hal_i2c_slave_handle_RXNE_interrupt(i2c_handle_t *hi2c);
static void 		hal_i2c_slave_rx_handle_btf(i2c_handle_t *hi2c);
void 						hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c);
void 						hal_i2c_error_cb(i2c_handle_t *I2cHandle);
static void 		hal_i2c_slave_handle_ack_failure(i2c_handle_t *hi2c);
void 						hal_i2c_init(i2c_handle_t *handle);

#endif