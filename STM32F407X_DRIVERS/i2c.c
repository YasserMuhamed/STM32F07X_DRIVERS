#include "i2c.h"

static void hal_i2c_enable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1|=I2C_REG_CR1_ENABLE_I2C;
}
static void hal_i2c_clear_stop_flag(i2c_handle_t *hi2c)
{
	uint32_t tmpreg;
	tmpreg=hi2c->Instance->SR1; //reading from SR1
	hi2c->Instance->CR1|=I2C_REG_CR1_ENABLE_I2C;//writing to SR1
	
}
static void hal_i2c_disable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1&=~I2C_REG_CR1_ENABLE_I2C;
}

static void hal_i2c_manage_clock_stretch(I2C_TypeDef *i2cx,uint32_t no_stretch)
{
	if(no_stretch)
	{
		i2cx->CR1|=I2C_REG_CR1_NOSTRETCH;
	}
	else 
	{
		i2cx->CR1&=~I2C_REG_CR1_NOSTRETCH;
	}
}

static void hal_i2c_set_own_address1(I2C_TypeDef *i2cx,uint32_t own_address)
{
	i2cx->OAR1	&=	~(0x7f<<1);
	i2cx->OAR1	|=	 (own_address<<1);

}

static void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx,uint32_t adr_mode)
{
	if(adr_mode==I2C_ADDMODE_10BIT)
	{
		i2cx->OAR1|=I2C_REG_OAR1_ADDRMODE;
	}
	else 
	{
		i2cx->OAR1&=~I2C_REG_OAR1_ADDRMODE;
	}
}

static void hal_i2c_set_fm_duty_cycle(I2C_TypeDef *i2cx,uint32_t duty_cycle)
{
	if(duty_cycle==I2C_FM_DUTY_16BY9)
	{
		i2cx->CCR|=I2C_REG_CCR_DUTY;
	}
	else 
	{
		i2cx->CCR&=~I2C_REG_CCR_DUTY;
	}
}

static void hal_i2c_clk_init(I2C_TypeDef *i2cx,uint32_t clkspeed,uint32_t duty_cycle)
{
	uint32_t pclk=I2C_PERIPHERAL_CLK_FREQ_10MHZ;
	i2cx->CR2&=~(0x3F);
	i2cx->CR2|=(pclk&0x3F);
	hal_i2c_configure_ccr(i2cx,pclk,clkspeed,duty_cycle);
	hal_i2c_rise_time_configuration(i2cx,pclk,clkspeed);
}
static void hal_i2c_generate_start_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1|=I2C_REG_CR1_START_GEN;
}
static void hal_i2c_generate_stop_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1|=I2C_REG_CR1_STOP_GEN;
}

static void hal_i2c_configure_evt_interrupt(I2C_TypeDef *i2cx,uint32_t enable)
{
	if(enable)
	{
		i2cx->CR2|=I2C_REG_CR2_EVT_INT_ENABLE;
	}
	else
	{
		i2cx->CR2&=~I2C_REG_CR2_EVT_INT_ENABLE;
	}
}
static void hal_i2c_configure_error_interrupt(I2C_TypeDef *i2cx,uint32_t enable)
{
	if(enable)
	{
		i2cx->CR2|=I2C_REG_CR2_ERR_INT_ENABLE;
	}
	else
	{
		i2cx->CR2&=~I2C_REG_CR2_ERR_INT_ENABLE;
	}
}

static void hal_i2c_configure_buffer_interrupt(I2C_TypeDef *i2cx,uint32_t enable)
{
	if(enable)
	{
		i2cx->CR2|=I2C_REG_CR2_BUF_INT_ENABLE;
	}
	else
	{
		i2cx->CR2&=~I2C_REG_CR2_BUF_INT_ENABLE;
	}
}

static uint32_t is_bus_busy(I2C_TypeDef *i2cx)
{
	if(i2cx->SR2&I2C_REG_SR2_BUS_BUSY_FLAG)
		return 1;
	else 
		return 0;
}
static void i2c_wait_until_sb_set(I2C_TypeDef *i2cx)
{
	while(!(i2cx->SR1&I2C_REG_SR1_SB_FLAG));
}
static void i2c_wait_until_addr_set(I2C_TypeDef *i2cx)
{
	while(!(i2cx->SR1&I2C_REG_SR1_ADDR_SENT_FLAG));
}
void hal_i2c_init(i2c_handle_t *handle)
{
	
	//helper functions...
}

void static clear_addr_flag(I2C_TypeDef *i2cx)
{
	uint32_t tmpreg;
	/* clear ADDR flag */
	tmpreg=i2cx->SR1;		//read SR1
	tmpreg=i2cx->SR2;	 //read  SR2
}
static void hal_i2c_send_addr_first(I2C_TypeDef *i2cx,uint8_t slave_address,uint8_t read)
{
	i2cx->DR&=~0xff;
	i2cx->DR|=slave_address;
	if(read)
	i2cx->DR|=I2C_REG_DR_READ;
	else
	i2cx->DR&=~I2C_REG_DR_READ;

}
void hal_i2c_master_tx(i2c_handle_t *handle,uint8_t slave_address,uint8_t *buffer,uint32_t len)
{
	/*populate the handle with tx buffer pointer and length information */
	handle->pBuffer=buffer;
	handle->XferCount=len;
	handle->XferSize=len;
	handle->State=HAL_I2C_STATE_BUSY_TX;
	/* make sure that I2C is enabled */
	hal_i2c_enable_peripheral(handle->Instance);
	
	/* first lets generate the start condition by using our helper function */
	hal_i2c_generate_start_condition(handle->Instance);
	/* waiit till sb is set */
	i2c_wait_until_sb_set(handle->Instance);
	/* address phase : send the 8 bit slave address first (8th bit is r/w bit ) */
	hal_i2c_send_addr_first(handle->Instance,slave_address,0);
	
	i2c_wait_until_addr_set(handle->Instance);
	clear_addr_flag(handle->Instance);
	
	hal_i2c_configure_buffer_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);
	
}

void hal_i2c_master_rx(i2c_handle_t *handle,uint8_t slave_addr,uint8_t *buffer,uint32_t len)
{
	/*Populate the handle with rx buffer pointer and length information*/
	handle->pBuffer=buffer;
	handle->XferCount=len;
	handle->XferSize=len;
	/* make state is busy in RX */
	handle->State=HAL_I2C_STATE_BUSY_RX;
	/* enable the I2C peripheral */
	hal_i2c_enable_peripheral(handle->Instance);
	/*make sure that POS bit is disabled */
	handle->Instance->CR1&=~I2C_CR1_POS;
	/*make sure that ACKing is enabled */
	handle->Instance->CR1|=I2C_CR1_ACK;
	/* first lets generate the start condition by using our helper function*/
	hal_i2c_generate_start_condition(handle->Instance);
	/* wait till sb is set */
	i2c_wait_until_sb_set(handle->Instance);
	//send the slave address
	hal_i2c_send_addr_first(handle->Instance,slave_addr,1);
	//wait until ADDR=1, that means Address phase is completed successfully 
	i2c_wait_until_addr_set(handle->Instance);
	//clear the ADDR flag which is set 
	clear_addr_flag(handle->Instance);
	//Enable the buff, err , event interrupts 
	hal_i2c_configure_buffer_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);

}

void hal_i2c_slave_tx(i2c_handle_t *handle,uint8_t *buffer,uint32_t len)
{
	/*Populate the handle with tx buffer pointer and length information*/
	handle->pBuffer=buffer;
	handle->XferCount=len;
	handle->XferSize=len;
	
	/* make state is busy in TX */
	handle->State=HAL_I2C_STATE_BUSY_TX;
	/*make sure that POS bit is disabled */
	handle->Instance->CR1&=~I2C_CR1_POS;
	/* enable the I2C peripheral */
	hal_i2c_enable_peripheral(handle->Instance);

	/*make sure that ACKing is enabled */
	handle->Instance->CR1|=I2C_CR1_ACK;

	//Enable the buff, err , event interrupts 
	hal_i2c_configure_buffer_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);

}

void hal_i2c_slave_rx(i2c_handle_t *handle,uint8_t *buffer,uint32_t len)
{
	/*Populate the handle with rx buffer pointer and length information*/
	handle->pBuffer=buffer;
	handle->XferCount=len;
	handle->XferSize=len;
	/* make state is busy in RX */
	handle->State=HAL_I2C_STATE_BUSY_RX;
		/*make sure that POS bit is disabled */
	handle->Instance->CR1&=~I2C_CR1_POS;
	/* enable the I2C peripheral */
	hal_i2c_enable_peripheral(handle->Instance);

	/*make sure that ACKing is enabled */
	handle->Instance->CR1|=I2C_CR1_ACK;

	//Enable the buff, err , event interrupts 
	hal_i2c_configure_buffer_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);

}

static void hal_i2c_master_handle_TXE_interrupt(i2c_handle_t	*hi2c)
{
	/* Write data to DR */
	hi2c->Instance->DR=(*hi2c->pBuffer++);
	hi2c->XferCount--;
	if(hi2c->XferCount==0)
	{
		/* Disable BUF interrupt */
		hi2c->Instance->CR2&=~I2C_REG_CR2_BUF_INT_ENABLE;
	}
}
static void hal_i2c_master_tx_complt(i2c_handle_t	*hi2c)
{
	/* call application callback here if needed */
}

static void hal_i2c_master_tx_handle_btf(i2c_handle_t	*hi2c)
{
	if(hi2c->XferCount!=0)
	{
		/* Write data to DR */
		hi2c->Instance->DR=(*hi2c->pBuffer++);
		hi2c->XferCount--;
	}
	else 
	{
		/* Disable EVT, BUF and ERR interrupts */
		hi2c->Instance->CR2&=~I2C_REG_CR2_EVT_INT_ENABLE;
		hi2c->Instance->CR2&=~I2C_REG_CR2_BUF_INT_ENABLE;
		hi2c->Instance->CR2&=~I2C_REG_CR2_ERR_INT_ENABLE;
		/* Generate Stop */ 
		hi2c->Instance->CR1|=I2C_REG_CR1_STOP_GEN;
		//SINCE ALL THE BYTES ARE SENT , MAKE STATE = READY
		hi2c->State=HAL_I2C_STATE_READY;
		//in this function , you can all application call back function 
		hal_i2c_master_tx_complt(hi2c);
	}
}

static void hal_i2c_slave_rx_complt( i2c_handle_t  *hi2c)
{
	/* call application callback here if needed */
	
}

static void hal_i2c_slave_handle_stop_condition(i2c_handle_t *hi2c)
{
	hi2c->Instance->CR2&=~I2C_REG_CR2_EVT_INT_ENABLE;
	hi2c->Instance->CR2&=~I2C_REG_CR2_BUF_INT_ENABLE;
	hi2c->Instance->CR2&=~I2C_REG_CR2_ERR_INT_ENABLE;
	
	/* Clear stop flag */
	hal_i2c_clear_stop_flag(hi2c);
	
	/* Disable Aknowledge */
	hi2c->Instance->CR1 &=~I2C_CR1_ACK;
	hi2c->State=HAL_I2C_STATE_READY;
	hal_i2c_slave_rx_complt(hi2c);
}

static void hal_i2c_slave_tx_handle_btf(i2c_handle_t *hi2c)
{
	if(hi2c->XferCount!=0)
	{
		/* Write data to DR */
		hi2c->Instance->DR=(*hi2c->pBuffer++);
		hi2c->XferCount--;
		
	}
}

static void hal_i2c_slave_rx_handle_btf(i2c_handle_t *hi2c)
{
	if(hi2c->XferCount!=0)
	{
		/* Write data to DR */
		(*hi2c->pBuffer++)=hi2c->Instance->DR;
	  hi2c->XferCount--;
		
	}
}
static void hal_i2c_slave_handle_RXNE_interrupt(i2c_handle_t *hi2c)
{
	if(hi2c->XferCount!=0)
	{
		/* Read data from DR */
		(*hi2c->pBuffer++)=hi2c->Instance->DR;
		hi2c->XferCount--;
	}
}


/* Slave mode selected */ 
/*else 
{*/
	/*if we are here , then event interrupt occured for slave .
	lets check , why this event interrupt occured by checking the status register	*/
	/*temp1=(hi2c->Instance->SR1&I2C_REG_SR1_ADDR_FLAG);
	temp2=(hi2c->Instance->CR2&I2C_REG_CR2_EVT_INT_ENABLES);
	temp3=(hi2c->Instance->SR1&I2C_REG_SR1_STOP_DETECTION_FLAG);
	temp4=(hi2c->Instance->SR2&I2C_REG_SR2_TRA_FLAG);*/
	/*ADDR set : slave address matched */
/*	if((tmp1)&&(tmp2))
	{
		led_turn_on(GPIOD,LED_GREEN);*/
		/*until you clear ADDR flag, I2C clock will be stretched */
	//	hal_i2c_clear_addr_flag(hi2c);
	//}*/
	/* STOP set */
/*	else if ((tmp3)&&(tmp2))
	{
		hal_i2c_master_rx_handle_btf(hi2c);
	}
}
}
*/



/* slave mode selected */ 

//else 
//{
	/*if we are here , then event interrupt occured for slave . 
	lets check, why this event interrupt occured by checking the status register */ 
	/*
	temp1=(hi2c->Instance->SR1&I2C_REG_SR1_ADDR_FLAG);
	temp2=(hi2c->Instance->CR2&I2C_REG_CR2_EVT_INT_ENABLES);
	temp3=(hi2c->Instance->SR1&I2C_REG_SR1_STOP_DETECTION_FLAG);
	temp4=(hi2c->Instance->SR2&I2C_REG_SR2_TRA_FLAG);
	*/
	/* ADDR set : Slave address matched */
/*
	if((tmp1)&&(tmp2))
	{
		led_turn_on(GPIOD,LED_GREEN);
*/
		/* until you clear ADDR flag, I2C clock will be stretched */
		/*
		hal_i2c_clear_addr_flag(hi2c);
	}
	*/
	/*STOP FLAG SET */
	/*
	else if ((tmp3)&&(tmp2))
	{
		hal_i2c_slave_handle_stop_condition(hi2c);
	}*/
	/* I2C in mode Transmitter */
	/*
	else if (tmp4)
	{
		temp1=(hi2c->Instance->SR1&I2C_REG_SR1_TXE_FLAG);
		temp2=(hi2c->Instance->CR2&I2C_REG_CR2_BUF_INT_ENABLE);
		temp3=(hi2c->Instance->SR1&I2C_REG_SR1_BTF_FLAG);
		temp4=(hi2c->Instance->CR2&I2C_REG_CR2_EVT_INT_ENABLE);
	*/
		/* TXE SET AND BTF RESET */
	/*
		if((tmp1)&&(tmp2)&& (!tmp3))
		{
			hal_i2c_slave_handle_TXE_interrupt(hi2c);
		}
	*/
		/*BTF set : looks like both tx buffer and shift register are empty*/
	/*
		else if ((tmp3)&& (tmp4))
		{
			hal_i2c_slave_tx_handle_btf(hi2c);
		}
	}*/
	/*I2C in mode Receiver */
	/*
	else 
	{
		temp1=(hi2c->Instance->SR1&I2C_REG_SR1_RXNE_FLAG);
		temp2=(hi2c->Instance->CR2&I2C_REG_CR2_BUF_INT_ENABLE);
		temp3=(hi2c->Instance->SR1&I2C_REG_SR1_BTF_FLAG);
		temp4=(hi2c->Instance->CR2&I2C_REG_CR2_EVT_INT_ENABLE);
	*/
		/* RXNE set and BTF reset */
	/*
		if((tmp1)&&(tmp2)&&(!tmp3))
		{
			hal_i2c_slave_handle_RXNE_interrupt(hi2c);
		}
	}
	
}
*/

void hal_i2c_error_cb(i2c_handle_t *I2cHandle)
{
	while(1)
	{
		led_toggle(GPIOD,LED_RED);
	}
}

static void hal_i2c_slave_handle_ack_failure(i2c_handle_t *hi2c)
{
	/* Disable EVT, BUF and ERR interrupt */
	hi2c->Instance->CR2&=~I2C_REG_CR2_EVT_INT_ENABLE;
	hi2c->Instance->CR2&=~I2C_REG_CR2_BUF_INT_ENABLE;
	hi2c->Instance->CR2&=~I2C_REG_CR2_ERR_INT_ENABLE;
	
	/* Clear AF flag */
	hi2c->Instance->SR1 &=~(I2C_REG_SR1_AF_FAILURE_FLAG);
	
	/* Disable Acknowledge */
	hi2c->Instance->CR1 &=~I2C_REG_CR1_ACK;
	hi2c->State=HAL_I2C_STATE_READY;
	//hal_i2c_slave_tx_complt(hi2c);

}
void hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c)
{
	uint32_t tmp1=0, tmp2=0, tmp3=0;
	tmp1=(hi2c->Instance->SR2&I2C_REG_SR1_BUS_ERROR_FLAG);
	tmp2=(hi2c->Instance->CR2&I2C_REG_CR2_ERR_INT_ENABLE);
	/* I2C BUS error interrupt occurred */
	if((tmp1)&& (tmp2))
	{
		hi2c->ErrorCode|=HAL_I2C_ERROR_BERR;
		/*Clear BERR flag */
		hi2c->Instance->SR1&=~(I2C_REG_SR1_ARLO_FLAG);
		
	}
	tmp1=(hi2c->Instance->SR1&I2C_REG_SR1_ARLO_FLAG);
	tmp2=(hi2c->Instance->CR2 &I2C_REG_CR2_ERR_INT_ENABLE);
	/* I2C Arbitration loss error interrupt occurred */
	if((tmp1)&& (tmp2))
	{
		hi2c->ErrorCode|=HAL_I2C_ERROR_ARLO;
		/*Clear ARLO flag*/
		hi2c->Instance->SR1&=~(I2C_REG_SR1_ARLO_FLAG);
	}
	tmp1=(hi2c->Instance->SR1&I2C_REG_SR1_AF_FAILURE_FLAG);
	tmp2=(hi2c->Instance->CR2&I2C_REG_CR2_ERR_INT_ENABLE);
	/* I2C Acknowledge failure error interrupt occured */
	if((tmp1)&&(tmp2))
	{
		tmp1=(hi2c->Instance->SR2&I2C_REG_SR2_MSL_FLAG);
		tmp2=hi2c->XferCount;
		tmp3=hi2c->State;
		if((!tmp1)&&(tmp2==0)&&(tmp3==HAL_I2C_STATE_BUSY_TX))
		{
			/* IF ACK failure happens for slave , then slave should assume that , master doesn't want more data so 
			it should stop sending more data to the master */
			hal_i2c_slave_handle_ack_failure(hi2c);
		}
		else 
		{
			/* if ack failure happens for master, then its an Error */
			hi2c->ErrorCode|=HAL_I2C_ERROR_AF;
			/* Clear AF flag*/
			hi2c->Instance->SR1&=~(I2C_REG_SR1_AF_FAILURE_FLAG);
		}
		
	}
	
	tmp1=(hi2c->Instance->SR1&I2C_REG_SR1_OVR_FLAG);
	tmp2=(hi2c->Instance->CR2&I2C_REG_CR2_ERR_INT_ENABLE);
	/* I2C Over-Run /Under-Run interrupt occured */
	if((tmp1)&&(tmp2))
	{
		hi2c->ErrorCode|=HAL_I2C_ERROR_OVR;
		/* Clear OVR FLAG*/
		hi2c->Instance->SR1 &=~(I2C_REG_SR1_OVR_FLAG);
	}
	if(hi2c->ErrorCode!=HAL_I2C_ERROR_NONE)
	{
		/*Disable Pos bit in I2c CR1 when error occured in Master/Mem Receive IT process*/
		hi2c->Instance->CR1&=~I2C_REG_CR1_POS;
		hal_i2c_error_cb(hi2c);
	}
		
}