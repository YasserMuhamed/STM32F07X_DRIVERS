#ifndef __GPIO_H
#define __GPIO_H
#include<stdint.h>
#include"stm32f407xx.h"

/*GPIO Mode settings values */ 

#define GPIO_PIN_INPUT_MODE						((uint32_t)0x00)
#define GPIO_PIN_OUTPUT_MODE					((uint32_t)0x01)
#define GPIO_PIN_ALT_FUN_MODE					((uint32_t)0x02)

/*GPIO OP  type selection values */ 
#define GPIO_PIN_OP_TYPE_PUSHPULL			((uint32_t)0x00)
#define GPIO_PIN_OP_TYPE_OPEN_DRAIN		((uint32_t)0x01)

/*GPIO Speed type selection values */
#define GPIO_PIN_SPEED_LOW						((uint32_t)0x00)
#define GPIO_PIN_SPEED_MEDIUM					((uint32_t)0x01)
#define GPIO_PIN_SPEED_HIGH						((uint32_t)0x02)
#define GPIO_PIN_SPEED_VERY_HIGH			((uint32_t)0x03)

/*GPIO pull  up/down selection values */ 
#define GPIO_PIN_NO_PULL_PUSH					((uint32_t)0x00)
#define GPIO_PIN_PULL_UP     					((uint32_t)0x01)
#define GPIO_PIN_PULL_DOWN				  	((uint32_t)0x11)

/*GPIO PORT ADDRESS*/

#define GPIO_PORT_A 									GPIOA
#define GPIO_PORT_B 									GPIOB
#define GPIO_PORT_C 									GPIOC
#define GPIO_PORT_D 									GPIOD
#define GPIO_PORT_E 									GPIOE
#define GPIO_PORT_F 									GPIOF
#define GPIO_PORT_G 									GPIOG
#define GPIO_PORT_H 									GPIOH
#define GPIO_PORT_I 									GPIOI

/* SPI MACROS*/

#define SPI_CLK_PIN 									0x15
#define GPIO_PIN_AF5_SPI2 						0x05

/*Macros to enable clock for different GPIO PORTS in RCC register*/
#define _HAL_RCC_GPIOA_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<0))
#define _HAL_RCC_GPIOB_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<1))
#define _HAL_RCC_GPIOC_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<2))
#define _HAL_RCC_GPIOD_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<3))
#define _HAL_RCC_GPIOE_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<4))
#define _HAL_RCC_GPIOF_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<5))
#define _HAL_RCC_GPIOG_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<6))
#define _HAL_RCC_GPIOH_CLK_ENABLE() 	(RCC->AHB1ENR|=(1<<7))


/** DATA STRUCTURE FOR GPIO PIN INITIALIZATION **/

typedef struct 
{
	uint32_t pin;/*Specifies the GPIO  pins to be configured*/
	uint32_t mode;/*Specifies the operating mode for the selected pin*/
	uint32_t op_type;/*Specifies the output type for the selected pin*/
	uint32_t pull;/*Specifies the pull up or pull down activation for selected pin */
	uint32_t speed;/*Specifies the speed for the selected pin*/
	uint32_t alternate;/*Specifies the alternate function value ,
											if mode is set for alt function mode*/
}GPIO_PIN_CONF_T;

typedef enum 
{
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE
}int_edge_sel_t;
static void 			hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t mode);
static void 			hal_gpio_configure_pin_otype(GPIO_TypeDef* GPIOx,uint16_t pin_no,uint32_t op_type);
static void 			hal_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t speed);
static void 			hal_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t pupd);
void 							hal_gpio_init(GPIO_TypeDef *GPIOx,GPIO_PIN_CONF_T* gpio_pin_conf);
void 							hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint8_t val);
uint8_t 					hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no);
void 							hal_gpio_configure_interrupt(uint16_t pin_no,int_edge_sel_t edge_sel);
void 							hal_gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);
void 							hal_gpio_clear_interrupt(uint16_t pin);
void 							hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t alt_fun_value);

#endif 
