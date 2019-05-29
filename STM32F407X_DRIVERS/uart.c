#include "uart.h"

void hal_uart_enable(USART_TypeDef *uartx)
{
	uartx->CR1 |=USART_REG_CR1_USART_EN;
}
void hal_uart_enable_disable_tx(USART_TypeDef *uartx,uint32_t te)
{
	if(te&USART_REG_CR1_TE)
	uartx->CR1 |=USART_REG_CR1_TE;
	else 
		uartx->CR1&=~USART_REG_CR1_TE;
}

void hal_uart_enable_disable_rx(USART_TypeDef *uartx,uint32_t re)
{
	if(re&USART_REG_CR1_RE)
	uartx->CR1 |=USART_REG_CR1_RE;
	else 
		uartx->CR1&=~USART_REG_CR1_RE;
}

void hal_uart_configure_over_sampling(USART_TypeDef *uartx,uint32_t over8)
{
	if (over8)
		uartx->CR1|=USART_REG_CR1_OVERS;
}

void hal_uart_configure_word_length(USART_TypeDef *uartx, uint32_t wl)
{
	if(wl)
	{
		uartx->CR1|=USART_REG_CR1_UART_WL; //9 data bits 
	}else 
	{
		uartx->CR1&=~USART_REG_CR1_UART_WL ; //8 data bits 
	}
	
}

void hal_uart_configure_stop_bits(USART_TypeDef *uartx,uint32_t nstop)
{
	//clear the 12th and 13th bits 
	uartx->CR2&=~(0x3<<USART_REG_CR2_STOP_BITS);
	if(nstop==USART_STOP_BITS_HALF)
	{
		uartx->CR2|=(0x01<<USART_REG_CR2_STOP_BITS);
	}else if (nstop==USART_STOP_BITS_2)
	{
		uartx->CR2|=(0x03<<USART_REG_CR2_STOP_BITS);
	}else 
	{
		uartx->CR2 |=(0x00<<USART_REG_CR2_STOP_BITS);
	}
}

void hal_uart_configure_rxne_interrupt(USART_TypeDef *uartx , uint32_t rxne_en)
{
	if(rxne_en)
		uartx->CR1|=USART_REG_CR1_RXNE_INT_EN;
	else 
		uartx->CR1 &=~USART_REG_CR1_RXNE_INT_EN;
}

void hal_uart_configure_txe_interrupt(USART_TypeDef *uartx,uint32_t txe_en)
{
	if(txe_en)
		uartx->CR1 |=USART_REG_CR1_TXE_INT_EN;
	else 
		uartx->CR1 &=~USART_REG_CR1_TXE_INT_EN;
}

void hal_uart_configure_error_interrupt(USART_TypeDef *uartx, uint32_t er_en )
{
	if(er_en)
		uartx->CR3 |=USART_REG_CR3_ERR_INT_EN;
	else 
		uartx->CR3 |=USART_REG_CR3_ERR_INT_EN;
}

void hal_uart_configure_parity_error_interrupt(USART_TypeDef *uartx, uint32_t pe_en)
{
		if(pe_en)
		uartx->CR1 |=USART_REG_CR1_PEIE_INT_EN;
	else 
		uartx->CR1 |=USART_REG_CR1_PEIE_INT_EN;
}
void hal_uart_set_baud_rate(USART_TypeDef *uartx,uint32_t baud)
{
	uint32_t val; 
	if(baud==USART_BAUD_9600)
	{
		val=0x683;
	}
	else if (baud==USART_BAUD_115200)
	{
		val=0x8A;
	}
	else 
	{
		val=0x0A;
	}
	uartx->BRR=val;
}

void hal_uart_init(uart_handle_t *uart_handle)
{
	// configure the word length 
	// configure the number of Stop bits 
	//configure the oversampling rate for the receive block 
	//set the baud rate 
	//enable the transmit block of the uart peripheral 
	//enable the receiver block of the uart peripheral 
	// enable the uart peripheral 
	
	uart_handle->tx_state=HAL_UART_STATE_READY;
	uart_handle->rx_state=HAL_UART_STATE_READY;
	uart_handle->ErrorCode=HAL_UART_ERROR_NONE;

	
	
}

void hal_uart_tx(uart_handle_t *uart_handle,uint8_t *buffer,uint32_t len)
{
	uart_handle->pTxBuffPtr=buffer;
	uart_handle->TxXferCount=len;
	uart_handle->TxXferSize=len;
	
	/* This handle is busy in doing the TX */ 
	uart_handle->tx_state=HAL_UART_STATE_BUSY_TX;
	/* Enable the UART Peripheral */ 
	hal_uart_enable(uart_handle->Instance);
	/* lets , enable the TXE interrupt */ 
	hal_uart_configure_txe_interrupt(uart_handle->Instance,1);
}

void hal_uart_rx(uart_handle_t *uart_handle,uint8_t *buffer,uint32_t len)
{
	uint32_t val;
	uart_handle->pRxBuffPtr=buffer;
	uart_handle->RxXferCount=len;
	uart_handle->RxXferSize=len;
	
	/* This handle is busy in doing the RX */ 
	uart_handle->rx_state=HAL_UART_STATE_BUSY_RX;
	/* Enable the UART Parity Error Interrupt */
	hal_uart_configure_parity_error_interrupt(uart_handle->Instance,1);
	/* Enable the UART  Errors Interrupt */
	hal_uart_configure_error_interrupt(uart_handle->Instance,1);
	val=uart_handle->Instance->DR;
	/*Enable the UART Data Registers not empty Interrupt */
	hal_uart_configure_rxne_interrupt(uart_handle->Instance,1);

	
}
void hal_uart_clear_error_flag(uart_handle_t *huart)
{
	uint32_t tmpreg=0x00;
	tmpreg=huart->Instance->SR;
	tmpreg=huart->Instance->DR;
}

void hal_uart_error_cb(uart_handle_t *huart)
{
	while(1)
	{
		led_turn_on(GPIOD,LED_RED);
	}
}
static void hal_uart_handle_TC_interrupt(uart_handle_t *huart)
{
	/*Disable the UART Transmit Complete Interrupt */ 
	huart->Instance->CR1 &=~USART_REG_CR1_TCIE_INT_EN; 
	huart->tx_state=HAL_UART_STATE_READY;
	/*Call the application callback */ 
	//if(huart->tx_cmp_cb)
		//huart->tx_cmp_cb(&huart->TxXferSize);
}
static void hal_uart_handle_TXE_interrupt(uart_handle_t *huart)
{
	uint32_t 		tmp1=0 ; 
	uint8_t 		val ; 
	tmp1=huart->tx_state;
	if(tmp1==HAL_UART_STATE_BUSY_TX)
	{
		val=(uint8_t)(*huart->pTxBuffPtr++&(uint8_t)0x00FF);
		huart->Instance->DR=val;
		if(--huart->TxXferCount==0)
		{
			/* Disable the UART TXE Interrupt */
			huart->Instance->CR1 &=~USART_REG_CR1_TXE_INT_EN;
			/* Enable the UART Transmit Complete Interrupt */
			huart->Instance->CR1|=USART_REG_CR1_TCIE_INT_EN;
		}
}
	}

void hal_uart_handle_RXNE_interrupt(uart_handle_t *huart)
{
	uint32_t tmp1=0; 
tmp1=huart->rx_state; 

if(tmp1==HAL_UART_STATE_BUSY_RX)
{
// is application using partity ??? 
if (huart->Init.Parity==UART_PARITY_NONE)
{
//no parity 
*huart->pRxBuffPtr++=(uint8_t)(huart->Instance->DR &(uint8_t 0x00FF);
}
else 
{
	// yes , don't read the most significant bit , because its a parity bit 
	*huart->pRxBuffPtr++=(uint8_t)(huart->Instance->DR &(uint8_t)0x007F);
	
}
/* are we done with the reception ??? */ 
if (--huart->RxXferCount==0)
{
	// yes , disable the RXNE interrupt 
	huart->Instance->CR1 &=~USART_REG_CR1_RXNE_INT_EN;
	/*Disable the uart parity error interrupt */ 
	huart->Instance->CR1&=~USART_REG_CR1_PEIE_INT_EN;
	/*Disable the 	uart Errors  interrupt*/
	huart->Instance->CR3&=~USART_REG_CR3_ERR_INT_EN;
	// make the state ready for this handle 
	huart->rx_state=HAL_UART_STATE_READY;
	/* call the application callback */
	//if(huart->rx_cmp_cb)
		//huart->rx_cmp_cb(&huart->RxXferSize);
}


void hal_uart_handle_interrupt(uart_handle_t *huart)
{
	uint32_t tmp1=0, tmp2=0 ; 
	tmp1=huart->Instance->SR&USART_REG_SR_PE_FLAG;
	tmp2=huart->Instance->CR1&USART_REG_CR1_PEIE_INT_EN;
	
	/*UART Parity error interrupt occured */
	
	if ((tmp1)&& (tmp2))
	{
		hal_uart_clear_error_flag(huart);
		huart->ErrorCode|=HAL_UART_ERROR_PE;
	}
	tmp1=huart->Instance->SR & USART_REG_SR_FE_FLAG;
	tmp2=huart->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
	if((tmp1)&& (tmp2))
	{
		hal_uart_clear_error_flag(huart);
		huart->ErrorCode|=HAL_UART_ERROR_FE;
	}
	tmp1=huart->Instance->SR & USART_REG_SR_RXNE_FLAG;
	tmp2=huart->Instance->CR3 & USART_REG_CR1_RXNE_INT_EN;
	/*Uart in mode Receiver */ 
	if((tmp1)&&(tmp2))
	{
		hal_uart_handle_RXNE_interrupt(huart);
	}
	tmp1=huart->Instance->SR & USART_REG_SR_TC_FLAG;
	tmp2=huart->Instance->CR3 & USART_REG_CR1_TCIE_INT_EN;
	/*Uart in mode Transmitter end */ 
	if((tmp1)&&(tmp2))
	{
		hal_uart_handle_TC_interrupt(huart);
	}
	if(huart->ErrorCode!=HAL_UART_ERROR_NONE)
	{
		huart->tx_state=HAL_UART_STATE_READY;
		huart->rx_state=HAL_UART_STATE_READY;
		hal_uart_error_cb(huart);
	}
		

}



