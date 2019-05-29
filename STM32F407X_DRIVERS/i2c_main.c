#include"i2c_main.h"


#define I2C_MASTER_MODE_EN
/* BUFFERS to hold master tx and rx data */
	uint8_t master_tx_buffer[5]={0xa5,0x55,0xa5,0x55,0xb0}
	uint8_t master_rx_buffer[5];
	
	/* buffers to hold slave tx and rx data */
	uint8_t slave_tx_buffer[5]={0xa5,0x55,0xa5,0x55,0xc0};
	uint8_t slave_rx_buffer[5];
	
	uint8_t master_write_req;
	uint8_t master_read_req;
	
	/* Handle variable for he I2C device */
	
	i2c_handle_t i2c_handle;
	
	/* This is handle variable for uart, used when print the debug message over UART */
	
	//uart_handle_t uart_handle;
	int TestReady=0;
	/* systemCoreClock holds the clock value of the microcontroller */
	extern uint32_t SystemCoreClock;
	/* generates some delay */
	void delay_gen()
	{
		uint32_t cnt=500000;
		while(cnt--);
	}


/*Does A gpio initialization which are used for I2C funcationality*/

void i2c_gpio_init()
{
	
	GPIO_PIN_CONF_T i2c_scl,i2c_sda;
	
	/* enable the clock for GPIO port B */
	_HAL_RCC_GPIOD_CLK_ENABLE();
	/*configure the GPIO_PORT_B_PIN_6 for SCL functionality*/
	i2c_scl.pin=I2C1_SCL_LINE;
	i2c_scl.mode=GPIO_PIN_ALT_FUN_MODE;
	i2c_scl.op_type=GPIO_PIN_OP_TYPE_OPEN_DRAIN;
	i2c_scl.pull=GPIO_PIN_PULL_UP;
	i2c_scl.speed=GPIO_PIN_SPEED_HIGH;
	
	hal_gpio_set_alt_function(GPIOB,I2C1_SCL_LINE,GPIO_PIN_AF4_I2C123);
	hal_gpio_init(GPIOB,&i2c_scl);
	
	/*configure the GPIO_PORT_B_PIN_9 for SDA functionality */
	
	i2c_sda.pin=I2C1_SDA_LINE;
	i2c_sda.mode=GPIO_PIN_ALT_FUN_MODE;
	i2c_scl.op_type=GPIO_PIN_OP_TYPE_OPEN_DRAIN;
	i2c_scl.pull=GPIO_PIN_PULL_UP;
	i2c_scl.speed=GPIO_PIN_SPEED_HIGH;
	
	
}


/* interrupt handler for the EXTI interrupts */
void EXTIx_IRQHandler(void)
{
	/* In the ISR first clear out the stick interrupt 
	pending bit for this interrupt */
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	/* Do you task here */ 
	TestReady=SET;
}

void I2Cx_EV_IRQHandler(void)
{

	hal_i2c_handle_evt_interrupt(&i2c_handle);

}

void I2Cx_ER_IRQHandler(void)
{

	hal_i2c_handle_error_interrupt(&i2c_handle);

}

int main(void )
{
	uint32_t val1;
	uint32_t slave_data_len;
	/* initialize to get LED functionality */
	led_init();
	/* This function configure the GPIOs for I2C functionality */
	i2c_gpio_init();
	
	#ifdef I2C_MASTER_MODE_EN
	//button_init(); // lets use button, only if master
	#endif 
	/* enable the clock for the I2C1 */ 
	_HAL_RCC_I2C1_CLK_ENABLE();
	
	/* Initialize the I2C handle structure and pass to driver */
	
	i2c_handle.Instance=I2C_1;
	i2c_handle.Init.ack_enable=I2C_ACK_ENABLE;
	i2c_handle.Init.AddressingMode=I2C_ADDRMODE_7BIT;
	i2c_handle.Init.ClockSpeed=400000;
	i2c_handle.Init.DutyCycle=I2C_FM_DUTY_2;
	i2c_handle.Init.GeneralCallMode=0;
	i2c_handle.Init.NoStretchMode=I2C_ENABLE_CLK_STRETCH;
	i2c_handle.Init.OwnAddress1=SLAVE_ADDRESS;
	/* Enable the IRQs in the NVIC */
	NVIC_EnableIRQ(I2CX_ER_IRQn); // for error interrupts 
	NVIC_EnableIRQ(I2CX_EV_IRQn);//for event interrupts 
	
	/* call driver to do initialization */
	hal_i2c_init(&i2c_handle);
	i2c_handle.State=HAL_I2C_STATE_READY;
	
	#ifdef I2C_MASTER_MODE_EN
	while(TestReady!=SET)
	{
	led_toggle(GPIOD,LED_ORANGE);
	//LED3 (orange)
	delay_gen();
	}
	led_turn_off(GPIOD,LED_ORANGE);
	#endif 
	
	while(1)
	{
		//check for state ready 
		while(i2c_handle.State!=HAL_I2C_STATE_READY);
		#ifdef I2C_MASTER_MODE_EN 
		/* write to slave */ 
		/* first send the master write cmd to slave */
		master_write_req=MASTER_WRITE_CMD;
		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t *) & master_write_req,1);
		/*application can block here , or cna do other task until above tx finishes */
		while(i2c_handle.State!=HAL_I2C_STATE_READY);
		/*Now send the number of bytes to be written*/
		master_write_req=WRITE_LEN;
		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
		/* application can block here, or can do other task untill above tx finishes */
		while (i2c_handle.State!=HAL_I2C_STATE_READY);
		/*Now send the data stream */
		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t *)&master_write_req,1);
		/* application can block here or can do other task until above tx finishes */ 
		while(i2c_handle.State !=HAL_I2C_STATE_READY);
		/*Read from slave */ 
		/* send the master read cmd to slave */
		
		master_read_req=MASTER_READ_CMD;
		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t *)&master_write_req,1);
		
		while(i2c_handle.State!=HAL_I2C_STATE_READY);
		/*Now send the number of bytes to be read */
		master_read_req=READ_LEN;
		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
		while(i2c_handle.State!=HAL_I2C_STATE_READY);
		memset(master_rx_buffer,0,5);
		/* Now read the data stream */
		hal_i2c_master_rx(&i2c_handle,SLAVE_ADDRESS_WRITE,master_rx_buffer,READ_LEN);
		while(i2c_handle.State!=HAL_I2C_STATE_READY);
		/* compare what master rcvd against what slave supposed to send */
		if (1/*Buffercmp(slave_tx_buffer,master_rx_buffer,READ_LEN)*/)
		{
			/* Error !!! this is not what slave supposed to send */ 
			led_turn_on(GPIOD,LED_RED);
		}else 
		{
			led_toggle(GPIOD,LED_BLUE);
		}
		delay_gen();
		#else /*if device slave */
		
		/* first rcv the command from the master */
		hal_i2c_slave_rx(&i2c_handle,&slave_rcv_cmd,1);
		/* application can block here , or can do other task untill above rx finishes.*/
		while(i2c_handle.State!=HAL_I2C_STATE_READY);
		/* is it master write cmd ?? */ 
		if (slave_rcv_cmd==MASTER_WRITE_CMD)
		{
		//prepare to rcv from the master 
		// first rcv number of bytes to be written by master 
		hal_i2c_slave_rx(&i2c_handle,&slave_rcv_cmd,1);
		while (i2c_handle.State!=HAL_I2C_STATE_READY);
		memset(slave_rx_buffer,0,sizeof(slave_rx_buffer));
		/* receive data from the master */
		hal_i2c_slave_rx(&i2c_handle,slave_rx_buffer,slave_rcv_cmd);
		/*application can block here , or can do other task until above rx finishes  */
		while(i2c_handle.State!=HAL_I2C_STATE_READY);
		/* COMPARE what slave rcvd against what master supposed to send */
		if (Buffercmp(slave_rx_buffer,master_tx_buffer,READ_LEN))
		{
		//Error 
		led_turn_on(GPIOD,LED_RED);
	}
	else 
	{
		led_toggle(GPIOD,LED_BLUE);
	}
	/* IS IT MASTER WRITE CMD ??*/
	if (slave_rcv_cmd==MASTER_READ_CMD)
	{
	  //PREPARE to rcv from the master
			//first rcv number of bytes to be written by master 
			hal_i2c_slave_rx(&i2c_handle, &slave_rcv_cmd,1);
			while(i2c_handle.State !=HAL_I2C_STATE_READY);
			memset(slave_rx_buffer,0,sizeof(slave_rcv_buffer));
			/* receive data from the master */
			hal_i2c_slave_rx(&i2c_handle,slave_rx_buffer,slave_rcv_cmd);
			/*application can bock here , or can do other task until above rx finishes */
			while(i2c_handle.State!=HAL_I2C_STATE_READY);
			/*COMPARE what slave rcvd aainst wha master supposed to send */
			if(Buffercmp(slave_rx_buffer,master_tx_buffer,READ_LEN))
			{
			//ERROR 
			led_turn_on(GPIOD,LED_RED);
		}
		else 
		{
		led_toggle(GPIOD,LED_BLUE);
	}
	
	}
	/* IS IT MASTER READ CMD ?? */ 
	
	if(slave_rcv_cmd==MASTER_READ_CMD)
	{
	//prepare to send data to the master 
	// first rcv number of bytes to be written to master 
	hal_i2c_slave_rx(&i2c_handle,&slave_data_len,1);
	
	while (i2c_handle.State!=HAL_I2C_STATE_READY);
	/* send data to the master of slave_data_len*/
	hal_i2c_slave_tx(&i2c_handle,slave_tx_buffer,slave_data_len);
	while(i2c_handle.State!=HAL_I2C_STATE_READY);

}
#endif 	
	
	return 0 ; 
}

