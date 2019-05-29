#include"spi.h"

static void hal_spi_enable(SPI_TypeDef *SPIx)
{
	if(!(SPIx->CR1&SPI_REG_CR1_SPE))
		SPIx->CR1|=SPI_REG_CR1_SPE;
}

static void hal_spi_disable(SPI_TypeDef *SPIx)
{
	SPIx->CR1&=~SPI_REG_CR1_SPE;
}

static void hal_spi_configure_phase_and_polarity(SPI_TypeDef *SPIx,uint32_t phase_value,uint32_t polarity_value)
{
	if(phase_value)
	{
		SPIx->CR1|=SPI_REG_CR1_CPHA;
	}
	else
	{
			SPIx->CR1&=~SPI_REG_CR1_CPHA;

	}
	if(polarity_value)
	{
		SPIx->CR1|=SPI_REG_CR1_CPOL;
	}
	else
	{
		SPIx->CR1&=~SPI_REG_CR1_CPOL;

	}

}

static void hal_spi_configure_device_mode(SPI_TypeDef *SPIx,uint32_t master)
{
		if(master)
	{
		SPIx->CR1|=SPI_REG_CR1_MSTR;
	}
	else 
	{
		SPIx->CR1 &=~SPI_REG_CR1_MSTR;
	}
}

static void hal_spi_configure_datasize_direction(SPI_TypeDef *SPIx,uint32_t datasize_16,uint32_t lsbfirst)
{
	if (datasize_16)
	{
		SPIx->CR1|=SPI_REG_CR1_DFF;
	}
	else 
	{
		SPIx->CR1&=~SPI_REG_CR1_DFF;
	}
	if(lsbfirst)
	{
		SPIx->CR1|=SPI_CR1_LSBFIRST;
	}
	else 
	{
		SPIx->CR1&=~SPI_CR1_LSBFIRST;
	}
}

static void hal_spi_configure_nss_master(SPI_TypeDef *SPIx,uint32_t ssm_enable)
{
	if(ssm_enable)
	{
		SPIx->CR1|=SPI_REG_CR1_SSM;
		SPIx->CR1|=SPI_REG_CR1_SSI;
	}
	else 
	{
		SPIx->CR1&=~SPI_REG_CR1_SSM;
	}
}

static void hal_spi_configure_nss_slave(SPI_TypeDef *SPIx,uint32_t ssm_enable)
{
	if(ssm_enable)
	{
		SPIx->CR1|=SPI_REG_CR1_SSM;
	}
	else 
	{
		SPIx->CR1&=~SPI_REG_CR1_SSM;
	}
}


static void hal_spi_tx_close_interrupt(spi_handle_t *hspi)
{
	hal_spi_disable_txe_interrupt(hspi->Instance);
	if(hspi->Init.Mode && (hspi->State!=HAL_SPI_STATE_BUSY_RX))
	{
		hspi->State=HAL_SPI_READY;
	}
}

void hal_spi_init(spi_handle_t *spi_handle)
{
	hal_spi_configure_phase_and_polarity(spi_handle->Instance,spi_handle->Init.CLKPhase,spi_handle->Init.CLKPolarity);
	hal_spi_configure_device_mode(spi_handle->Instance,spi_handle->Init.Mode);
	hal_spi_configure_datasize_direction(spi_handle->Instance,spi_handle->Init.DataSize,spi_handle->Init.FirstBit);
	hal_spi_configure_nss_slave(spi_handle->Instance,spi_handle->Init.NSS);
	
}
void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2|=SPI_REG_CR2_TXEIE_ENABLE;
}
void hal_spi_enable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2|=SPI_REG_CR2_RXNEIE_ENABLE;
}

void hal_spi_disable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2&=~SPI_REG_CR2_TXEIE_ENABLE;
}
void hal_spi_master_tx(spi_handle_t *spi_handle,uint8_t *buffer,uint32_t len)
{
	spi_handle->pTxBuffPtr=buffer;
	spi_handle->TxXferCount=len;
	spi_handle->TxXferSize=len;
	spi_handle->State=HAL_SPI_STATE_BUSY_TX;
	hal_spi_enable(spi_handle->Instance);
	hal_spi_enable_txe_interrupt(spi_handle->Instance);
}
void hal_spi_slave_rx(spi_handle_t *spi_handle,uint8_t *rcv_buffer,uint32_t len)
{
	uint32_t i =0 , val;
	
	/* Data will be rxed to rx_buffer */
	spi_handle->pRxBuffPtr=rcv_buffer;
	spi_handle->RxXferSize=len;
	spi_handle->RxXferCount=len;
	
	/* Driver is busy in RX */
	spi_handle->State=HAL_SPI_STATE_BUSY_RX;
	
	hal_spi_enable(spi_handle->Instance);

	/* Now enable  RXNE Interrupt */
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);
}
void hal_spi_master_rx(spi_handle_t *spi_handle,uint8_t *rx_buffer,uint32_t len)
{
	uint32_t i =0 , val;
	/* This is a dummy tx */
	spi_handle->pTxBuffPtr=rx_buffer;
	spi_handle->TxXferSize=len;
	spi_handle->TxXferCount=len;
	
	/* Data will be rxed to rx_buffer */
	spi_handle->pRxBuffPtr=rx_buffer;
	spi_handle->RxXferSize=len;
	spi_handle->RxXferCount=len;
	
	/* Driver is busy in RX */
	spi_handle->State=HAL_SPI_STATE_BUSY_RX;
	hal_spi_enable(spi_handle->Instance);
	
	val=spi_handle->Instance->DR;
	/* Now enable both TXE and RXNE Interrupt */
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);
	hal_spi_enable_txe_interrupt(spi_handle->Instance);

}
void hal_spi_slave_tx(spi_handle_t *spi_handle,uint8_t *tx_buffer,uint32_t len)
{
	spi_handle->pTxBuffPtr=tx_buffer;
	spi_handle->TxXferCount=len;
	spi_handle->TxXferSize=len;
	
	spi_handle->pRxBuffPtr=tx_buffer;
	spi_handle->RxXferCount=len;
	spi_handle->RxXferSize=len;
	
	spi_handle->State=HAL_SPI_STATE_BUSY_TX;
	
	hal_spi_enable(spi_handle->Instance);
	
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);
	hal_spi_enable_txe_interrupt(spi_handle->Instance);

}

void hal_spi_handle_rx_interrupt(spi_handle_t *hspi)
{
	if(hspi->Init.DataSize==SPI_8BIT_DF_ENABLE)
	{
		hspi->Instance->DR=(*hspi->pTxBuffPtr++);
		hspi->TxXferCount--; // we sent 1 byte 
	}
	/*Transmit data in 16 Bit mode */
	else 
	{
		hspi->Instance->DR=*((uint16_t*)hspi->pTxBuffPtr);
		hspi->pTxBuffPtr+=2;
		hspi->TxXferCount-=2;
	}
	if(hspi->TxXferCount==0)
	{
		hal_spi_tx_close_interrupt(hspi);
	}
}

void hal_i2c_spi_irq_handler(spi_handle_t *hspi)
{
	uint32_t tmp1=0,tmp2=0;
	/* Check to see RXNE is set in the status register*/
	tmp1=(hspi->Instance->SR&SPI_REG_SR_RXNE_FLAG);
	tmp2=(hspi->Instance->CR2&SPI_REG_CR2_RXNEIE_ENABLE);
	
	if((tmp1!=RESET)&&(tmp2!=RESET))
	{
		hal_spi_handle_rx_interrupt(hspi);
		return;
	}

}