#include<stdint.h>
#include"led.h"
#include"i2c.h"
#include"gpio.h"


#define GPIOB_PIN_6			6

#define GPIOB_PIN_9			9
#define I2C1_SCL_LINE		GPIOB_PIN_6
#define I2C1_SDA_LINE		GPIOB_PIN_9

/* Definition for I2CX'S NVIC */

#define I2CX_EV_IRQn							I2C1_EV_IRQn
#define I2CX_EV_IRQHandler				I2C1_EV_IRQHandler
#define I2CX_ER_IRQn							I2C1_ER_IRQn
#define I2CX_ER_IRQHandler				I2C1_ER_IRQHandler

/* Definition for I2CX'S NVIC */ 

#define EXTIX_IRQn							EXTI0_IRQn
#define EXTIX_IRQHandler				EXTI0_IRQHandler

/*Definition for EXTIX's NVIC */

#define EXTIX_IRQn							EXTI0_IRQn
#define EXTIX_IRQHandler				EXTI0_IRQHandler

/*USER button GPIO*/

#define GPIO_BUTTON_PIN					0
#define ALT_FUN_4								0x04
#define GPIO_PIN_AF4_I2C123			ALT_FUN_4

#define SLAVE_ADDRESS						0x05
#define MASTER_WRITE_CMD				1
#define SLAVE_ADDRESS_WRITE			0x000
#define WRITE_LEN								0x8
#define READ_LEN								8
#define MASTER_READ_CMD					1