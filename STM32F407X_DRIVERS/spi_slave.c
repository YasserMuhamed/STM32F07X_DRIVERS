#include"spi_main.h"
#include"led.h"



uint8_t master_write_data[]={0xa,0xb,0xc,0xd};
uint8_t master_read_buffer [4];
spi_handle_t SpiHandle;
/* configure gpio for spi functionality */ 

void spi_gpio_init(void)
{
	GPIO_PIN_CONF_T spi_conf;

	_HAL_RCC_GPIOB_CLK_ENABLE();
	/* configure GPIOB PIN_13 for SPI CLK funcationality */
	spi_conf.pin=SPI_CLK_PIN;
	spi_conf.mode=GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type=GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull=GPIO_PIN_PULL_DOWN;
	spi_conf.speed=GPIO_PIN_SPEED_MEDIUM;
	
	hal_gpio_set_alt_function(GPIOB,SPI_CLK_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB,&spi_conf);
	
	/* configure GPIOB_PIN_14 for SPI MISO funcationality */
	spi_conf.pin=SPI_MISO_PIN_14;
	spi_conf.pull=GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MISO_PIN_14,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB,&spi_conf);
	
	/* configure GPIOB_PIN_15 for SPI MOSI funcationality */
	
	spi_conf.pin=SPI_MISO_PIN_15;
	spi_conf.pull=GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MISO_PIN_15,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB,&spi_conf);
	
}

int main()
{
	
	uint16_t ack_bytes=SPI_ACK_BYTES;
	uint8_t rcv_cmd[2];

	spi_gpio_init();
	/*To use LED */
	led_init();

	/* Configure USER Button interrupt*/
	_HAL_RCC_GPIOA_CLK_ENABLE();
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN,INT_FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);
	
	/* enable the clock for the SPI2*/
	_HAL_RCC_SPI2_CLK_ENABLE();
	
	/* Fill up the handle structure */
	SpiHandle.Instance=SPI_2;
	SpiHandle.Init.BaudRatePrescaler=SPI_REG_CR1_BR_PCLK_DIV_32;
	SpiHandle.Init.Direction=SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase=SPI_SECOND_CLOCK_TRANS;
	SpiHandle.Init.CLKPolarity=SPI_CPOL_LOW;
	SpiHandle.Init.DataSize=SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.FirstBit=SPI_TX_MSB_FIRST;
	SpiHandle.Init.NSS=SPI_SSM_ENABLE;
	SpiHandle.Init.Mode=SPI_SLAVE_MODE_SEL;
	SpiHandle.State=HAL_SPI_READY;
	
	hal_spi_init(&SpiHandle);
	NVIC_EnableIRQ(SPI2_IRQn);
	
	//hal_debug_uart_init(DEBUG_USART_BAUD_9600);
	
	//while(TestReady!=SET)
	//{
	//	led_toggle(GPIOD,LED_ORANGE);
	//	delay_gen();
	//}
	//led_turn_off(GPIOD,LED_ORANGE);
	
	
	while(1)
	{
		while(SpiHandle.State!=HAL_SPI_READY);
		
		

		/* first send the master write cmd to slave */
		hal_spi_master_rx(&SpiHandle,rcv_cmd,CMD_LENGTH);
		
		/* application can block here , or can do other task untill above tx finishes */
		while(SpiHandle.State!=HAL_SPI_READY);
		/* this dealy helps for the slave to be ready with the ACK bytes */
		master_cmd=(uint16_t)(rcv_cmd[1]<<8|rcv_cmd[0]);
		
		if(master_cmd==CMD_MASTER_WRITE || master_cmd==CMD_MASTER_READ)
		{
			hal_spi_master_tx(&SpiHandle,(uint8_t*)ack_bytes,ACK_LEN);
			while(SpiHandle.State!=HAL_SPI_READY);

		}
		else 
		{
			led_toggle(GPIOD,LED_RED);
		}
		
		if(master_cmd==CMD_MASTER_WRITE)
		{
			hal_spi_slave_rx(&SpiHandle,slave_rx_buffer,DATA_LENGTH);
			while(SpiHandle.State!=HAL_SPI_READY);
			if(Buffercmp(master_write_data,slave_rx_buffer,4))
			{
				led_toggle(GPIOD,LED_RED);
			}
			else 
			{
				led_toggle(GPIOD,LED_BLUE);
			}
		}
		/* read back the ACK bytes from the slave */
		hal_spi_master_rx(&SpiHandle,ack_buf,ACK_LEN);
		/* wait until ACK reception finishes */
		while(SpiHandle.State!=HAL_SPI_READY);
		/* did we rcv the valid ACK from slave */
		if(ack_buf[0]==0XE5&&ack_buf[1]==0XD5)
		{
			//correct ack 
			led_toggle(GPIOD,LED_GREEN);
		}
		else 
		{
			//invalide ack 
			assert_error ();
			memset(ack_buf,0,2);
		}
	}
	/* Now send the data stream */
	hal_spi_master_tx(&SpiHandle,master_write_data,DATA_LENGTH);
	while(SpiHandle.State!=HAL_SPI_READY);
	delay_gen();
	//read from slave 
	
	/* Master Read Slave */
	addrcmd[0]=(uint8_t)CMD_MASTER_READ;
	addrcmd[1]=(uint8_t)(CMD_MASTER_READ>>8);
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle,addrcmd,CMD_LENGTH);
	while(SpiHandle.State!=HAL_SPI_READY);
	/* this delay helps for the slave to be ready with the ACK bytes */
	
	delay_gen();
	
	/*read back the ACK bytes from the slave */
	hal_spi_master_rx(&SpiHandle,ack_buf_ACK_LEN);
	
	while(SpiHandle.State!=HAL_SPI_READY);
	if(ack_buf[0]==0xE5&&ack_buf[1]=0XD5)
	{
		//correct ack
		led_toggle(GPIOD,LED_GREEN);
		memset(ack_buf,0,2);
	}
	else 
	{
			//invalide ack 
			assert_error ();
			memset(ack_buf,0,2);
	}
	/* start receiving from the slave*/
	hal_spi_master_rx(&SpiHandle,master_read_buffer,DATA_LENGTH);
	while(SpiHandle.State!=HAL_SPI_READY);
	
	/* compare the data rcvd from slave , with what slave supposed to send */
	if(Buffercmp(master_read_buffer,slave_reply_data,DATA_LENGTH))
	{
		led_toggle(GPIOD,LED_RED);
	}
	else 
	{
		led_toggle(GPIOD,LED_BLUE);
	}
	return 0; 
}


void EXTI0_IRQHandler(void)
{
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	TestReady=SET;
}

void SPI2_IRQHandler(void)
{
	hal_spi_irq_handler(&SpiHandle);
}