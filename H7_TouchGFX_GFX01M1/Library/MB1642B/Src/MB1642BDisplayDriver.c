#include "stm32h7xx_hal.h"
#include "DCS.h"
#include "main.h"
#include "MB1642BDisplayDriver.h"
#include <assert.h>

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim6;

volatile uint16_t TE = 0;

//Signal TE interrupt to TouchGFX
void touchgfxSignalVSync(void);

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

static void Display_DCS_Send(uint8_t command)
{
	// Reset the nCS pin
	DISPLAY_CSX_GPIO_Port->BSRR = (uint32_t)DISPLAY_CSX_Pin << 16U;

	// ReSet the DCX pin
	DISPLAY_DCX_GPIO_Port->BSRR = (uint32_t)DISPLAY_DCX_Pin << 16U;

	// Send the command
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&command, 1, 500);

	// Set the DCX pin
	DISPLAY_DCX_GPIO_Port->BSRR = (uint32_t)DISPLAY_DCX_Pin;

	// Set the nCS
	DISPLAY_CSX_GPIO_Port->BSRR = (uint32_t)DISPLAY_CSX_Pin;
}

static void Display_DCS_Send_With_Data(uint8_t command, uint8_t* data, uint8_t size)
{
  // Reset the nCS pin
  DISPLAY_CSX_GPIO_Port->BSRR = DISPLAY_CSX_Pin << 16U;
  // ReSet the DCX pin
  DISPLAY_DCX_GPIO_Port->BSRR = DISPLAY_DCX_Pin << 16U;

  HAL_SPI_Transmit(&hspi1, (uint8_t *)&command, 1, 500);

  // Set the DCX pin to send data in data mode
  DISPLAY_DCX_GPIO_Port->BSRR = DISPLAY_DCX_Pin;

  HAL_SPI_Transmit(&hspi1, (uint8_t *)data, size, 500);

  // Wait until the bus is not busy before changing configuration
  while(((hspi1.Instance->SR) & SPI_FLAG_EOT) != RESET);

  // Set the nCS
  DISPLAY_CSX_GPIO_Port->BSRR = DISPLAY_CSX_Pin;
}

void MB1642BDisplayDriver_DisplayOn(void)
{
  // Display ON
  Display_DCS_Send(DCS_SET_DISPLAY_ON);
  HAL_Delay(100);
}

void Display_OFF(void)
{
  // Display OFF
  Display_DCS_Send(DCS_SET_DISPLAY_OFF);
  HAL_Delay(100);
}

static uint16_t old_x0=0xFFFF, old_x1=0xFFFF, old_y0=0xFFFF, old_y1=0xFFFF;

void Display_Set_Area(uint16_t x0, uint16_t y0,
                      uint16_t x1, uint16_t y1)
{
  uint8_t arguments[4];

  // Set columns, if changed
  if (x0 != old_x0 || x1 != old_x1)
  {
    arguments[0] = x0 >> 8;
    arguments[1] = x0 & 0xFF;
    arguments[2] = x1 >> 8;
    arguments[3] = x1 & 0xFF;
    Display_DCS_Send_With_Data(0x2A, arguments, 4);

    old_x0 = x0;
    old_x1 = x1;
  }

  // Set rows, if changed
  if (y0 != old_y0 || y1 != old_y1)
  {
    arguments[0] = y0 >> 8;
    arguments[1] = y0 & 0xFF;
    arguments[2] = y1 >> 8;
    arguments[3] = y1 & 0xFF;
    Display_DCS_Send_With_Data(0x2B, arguments, 4);

    old_y0 = y0;
    old_y1 = y1;
  }
}

volatile uint8_t IsTransmittingBlock_;
void Display_Bitmap(const uint16_t *bitmap, uint16_t posx, uint16_t posy, uint16_t sizex, uint16_t sizey)
{
	IsTransmittingBlock_ = 1;
	__HAL_SPI_ENABLE(&hspi1); // Enables SPI peripheral
	uint8_t command = DCS_WRITE_MEMORY_START;

	// Define the display area
	Display_Set_Area(posx, posy, posx+sizex-1, posy+sizey-1);


	// Reset the nCS pin
	DISPLAY_CSX_GPIO_Port->BSRR = (uint32_t)DISPLAY_CSX_Pin << 16U;

	// ReSet the DCX pin
	DISPLAY_DCX_GPIO_Port->BSRR = (uint32_t)DISPLAY_DCX_Pin << 16U;

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&command, 1, 500);

	// Wait until the bus is not busy before changing configuration
	while(((hspi1.Instance->SR) & SPI_FLAG_EOT) != RESET);

	DISPLAY_DCX_GPIO_Port->BSRR = DISPLAY_DCX_Pin;

	// Set the SPI in 16-bit mode to match endianess
	__HAL_SPI_DISABLE(&hspi1); // Disable SPI peripheral
	hspi1.Instance->CFG1 |= SPI_DATASIZE_16BIT;
	__HAL_SPI_ENABLE(&hspi1);

	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t* )bitmap, sizex*sizey);

	__HAL_SPI_DISABLE(&hspi1); // Disable SPI peripheral
	hspi1.Instance->CFG1 = SPI_DATASIZE_8BIT;
	__HAL_SPI_ENABLE(&hspi1);

//	// Disable spi peripherals
//	__HAL_SPI_DISABLE(&hspi1);
//	__HAL_DMA_DISABLE(&hdma_spi1_tx);
//
//	hspi1.TxXferCount = (hspi1.TxXferCount + (uint16_t) 1UL) >> 1UL;
//
//	/* Clear TXDMAEN bit*/
//	CLEAR_BIT(hspi1.Instance->CFG1, SPI_CFG1_TXDMAEN);
//
//	/* Clear all flags */
//	DMA_Base_Registers* tempDMA_reg = (DMA_Base_Registers *)hdma_spi1_tx.StreamBaseAddress;
//	tempDMA_reg->IFCR = 0x3FUL << (hdma_spi1_tx.StreamIndex & 0x1FU);
////
//    /* Configure DMA Stream data length */
//    ((DMA_Stream_TypeDef *)hdma_spi1_tx.Instance)->NDTR = sizex*sizey;
//
//    /* Configure DMA Stream destination address */
//    ((DMA_Stream_TypeDef *)hdma_spi1_tx.Instance)->PAR = (uint32_t)&hspi1.Instance->TXDR;
//
//    /* Configure DMA Stream source address */
//    ((DMA_Stream_TypeDef *)hdma_spi1_tx.Instance)->M0AR = (uint32_t)bitmap;
//
////	/* Disable the transfer half complete interrupt */
////	__HAL_DMA_DISABLE_IT(&hdma_spi1_tx, DMA_IT_HT);
////	/* Enable the transfer complete interrupt */
////	__HAL_DMA_ENABLE_IT(&hdma_spi1_tx, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME));
//
//	/* Enable the Peripherals */
//	__HAL_DMA_ENABLE(&hdma_spi1_tx);
//	__HAL_SPI_ENABLE(&hspi1);
//
//	/* Enable Tx DMA Request */
//	SET_BIT(hspi1.Instance->CFG1, SPI_CFG1_TXDMAEN);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);

  TE++;
  (&htim6)->Instance->CR1 &= ~(TIM_CR1_CEN);
  (&htim6)->Instance->CNT = 0;

  touchgfxSignalVSync();
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);
  (&htim6)->Instance->CR1 = (TIM_CR1_CEN);
}

void MB1642BDisplayDriver_DisplayInit(void)
{
  uint8_t arguments[4];
  __HAL_SPI_ENABLE(&hspi1);
  // Sleep out
  Display_DCS_Send(DCS_EXIT_SLEEP_MODE);
  HAL_Delay(100);

  // Display Normal mode
  Display_DCS_Send(DCS_ENTER_NORMAL_MODE);
  HAL_Delay(100);

  // MADCTL: Exchange RGB / BGR + Mirror X
  arguments[0] = 0x48; // 0x48
  Display_DCS_Send_With_Data(DCS_SET_ADDRESS_MODE, arguments, 1);
  HAL_Delay(100);

  // Pixel Format
  arguments[0] = 0x05; // RGB565
  Display_DCS_Send_With_Data(DCS_SET_PIXEL_FORMAT, arguments, 1);
  HAL_Delay(100);

  // Tearing effect line on
  arguments[0] = 0; //0x00;
  Display_DCS_Send_With_Data(DCS_SET_TEAR_ON, arguments, 1);
  HAL_Delay(100);

  // Tearing effect scan line
  arguments[0] = 0;
  arguments[1] = 0;
  Display_DCS_Send_With_Data(DCS_SET_TEAR_SCANLINE, arguments, 2);
  HAL_Delay(100);

}

void MB1642BDisplayDriver_DisplayReset(void)
{
  HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
}

void MB1642BDisplayDriver_Init(void)
{
  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

int touchgfxDisplayDriverTransmitActive(void)
{
  return IsTransmittingBlock_;
}

void touchgfxDisplayDriverTransmitBlock(const uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  Display_Bitmap((uint16_t*)pixels, x, y, w, h);
}

void MB1642BDisplayDriver_DMACallback(void)
{
//  /* Transfer Complete Interrupt management ***********************************/
//  if ((0U != (DMA1->ISR & (DMA_FLAG_TC1))) && (0U != (hdma_spi1_tx.Instance->CCR & DMA_IT_TC)))
//  {
//    /* Disable the transfer complete and error interrupt */
//    __HAL_DMA_DISABLE_IT(&hdma_spi1_tx, DMA_IT_TE | DMA_IT_TC);
//
//    /* Clear the transfer complete flag */
//    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_FLAG_TC1);
//
//    IsTransmittingBlock_ = 0;
//
//    // Wait until the bus is not busy before changing configuration
//    // SPI is busy in communication or Tx buffer is not empty
//    while(((hspi1.Instance->SR) & SPI_FLAG_BSY) != RESET) { }
//
    // Set the nCS
    DISPLAY_CSX_GPIO_Port->BSRR = DISPLAY_CSX_Pin;

    // Go back to 8-bit mode
    hspi1.Instance->CFG1 = SPI_DATASIZE_8BIT;

    // Signal Transfer Complete to TouchGFX
    DisplayDriver_TransferCompleteCallback();
//  }
//    /* Transfer Error Interrupt management **************************************/
//  else if ((0U != (DMA1->ISR & (DMA_FLAG_TC1))) && (0U != (hdma_spi1_tx.Instance->CCR & DMA_IT_TE)))
//  {
//    /* When a DMA transfer error occurs */
//    /* A hardware clear of its EN bits is performed */
//    /* Disable ALL DMA IT */
//    __HAL_DMA_DISABLE_IT(&hdma_spi1_tx, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
//
//    /* Clear all flags */
//    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_FLAG_GI1 );
//
//    assert(0);  // Halting program - Transfer Error Interrupt received.
//  }
}


int touchgfxDisplayDriverShouldTransferBlock(uint16_t bottom)
{
  //return (bottom < getCurrentLine());
  return (bottom < (TE > 0 ? 0xFFFF : ((__IO uint16_t)htim6.Instance->CNT)));
}

void test()
{
  uint16_t i, j;
  uint8_t color1 = 0xF0;
  uint8_t color2 = 0x0F;
  uint8_t cmd = 0x2C;

  Display_Set_Area(0, 0, 239, 319);

  HAL_GPIO_WritePin(DISPLAY_DCX_GPIO_Port, DISPLAY_DCX_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DISPLAY_CSX_GPIO_Port, DISPLAY_CSX_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&(cmd), 1, 500);

  // Reset the nCS pin
  // DISPLAY_CSX_GPIO_Port->BRR = DISPLAY_CSX_Pin;
  HAL_GPIO_WritePin(DISPLAY_CSX_GPIO_Port, DISPLAY_CSX_Pin, GPIO_PIN_RESET);
  // Set the DCX pin
  // DISPLAY_DCX_GPIO_Port->BRR = DISPLAY_DCX_Pin;
  HAL_GPIO_WritePin(DISPLAY_DCX_GPIO_Port, DISPLAY_DCX_Pin, GPIO_PIN_SET);

  while (color1 > 0x0F && color2 < 0xF0 ) {
	for(i = 0; i < 320; i++){
		for(j = 0; j < 240; j++){
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&(color1), 1, 500);
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&(color2), 1, 500);
		}
	 }

  	 color1 = color1 - 0x0A;
  	 color2 = color2 + 0x0A;
  	 HAL_Delay(100);
  }
  MB1642BDisplayDriver_DisplayOn();
}
