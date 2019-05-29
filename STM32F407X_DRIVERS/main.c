#include"led.h"


int main(void)
{

	uint32_t i;
	/* Initialize the LEDs */
	led_init();
	/* Enable the clock for the GPIOA PORT */
	_HAL_RCC_GPIOA_CLK_ENABLE();
	/* Configure the button interrupt as falling edge */
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN,INT_FALLING_EDGE);
	/* Enable the interrupt on EXTI0 line */
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);
	
	while(1)
	{
		led_turn_on(GPIOD,LED_ORANGE);
		led_turn_off(GPIOD,LED_BLUE);
		
		for (i=0;i<500000;i++);
		
		led_turn_off(GPIOD,LED_ORANGE);
		led_turn_off(GPIOD,LED_BLUE);
		
		for (i=0;i<500000;i++);

	}
	
	return 0 ; 
}


void EXTI0_IRQHandler(void)
{
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	/* Do your task here */
	led_toggle(GPIOD,LED_BLUE);
	led_toggle(GPIOD,LED_ORANGE);
	led_toggle(GPIOD,LED_RED);
	led_toggle(GPIOD,LED_GREEN);
	
}
