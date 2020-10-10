#include "UART.h"
#include "stm32h7xx.h"
#include <stdbool.h>
#include <string.h>

uint8_t tx_buffer[UART_BUF_SIZE] = {0};
uint8_t rx_buffer[UART_BUF_SIZE] = {0};;
uint8_t rx_Uart_buffer[UART_BUF_SIZE] = {0};;

volatile uint32_t rx_buffer_len = 0;
volatile bool  rx_buffer_received = false;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef* ghuart;

void UART_init() {
    HAL_UART_MspInit(ghuart);
    
    __HAL_DMA_ENABLE_IT(ghuart->hdmarx, DMA_IT_TC);
    __HAL_DMA_ENABLE(ghuart->hdmarx);
    __HAL_UART_ENABLE_IT(ghuart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(ghuart);
    HAL_UART_Receive_DMA(ghuart, rx_Uart_buffer, UART_BUF_SIZE);
}

void UART_Abort_Rx(UART_HandleTypeDef *huart) {
  if (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (huart->hdmarx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmarx->XferAbortCallback = NULL;

      if (HAL_DMA_Abort(huart->hdmarx) != HAL_OK)
      {
        if (HAL_DMA_GetError(huart->hdmarx) == HAL_DMA_ERROR_TIMEOUT)
        {
          /* Set error code to DMA */
          huart->ErrorCode = HAL_UART_ERROR_DMA;

          // return HAL_TIMEOUT;
        }
      }
    }
  }

  /* Reset Tx and Rx transfer counters */
  huart->TxXferCount = 0U;
  huart->RxXferCount = 0U;

  /* Clear the Error flags in the ICR register */
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

  /* Restore huart->gState and huart->RxState to Ready */
  huart->gState  = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;

  huart->ErrorCode = HAL_UART_ERROR_NONE;    
}

void checkReceiveData(void) {
    rx_buffer_len = UART_BUF_SIZE - READ_BIT(DMA1_Stream0->NDTR, DMA_SxNDT);
    UART_Abort_Rx(ghuart);

    if(rx_buffer_len != 0) {
        memcpy(rx_buffer, rx_Uart_buffer, rx_buffer_len);
    }

    HAL_UART_Transmit_DMA(ghuart, rx_buffer, rx_buffer_len);
}


void UART_IDLE_Handler(UART_HandleTypeDef *huart) {
    // following code is to get the data send before the idle 
    // interrupt is triggered, check manual 16.3.16
    uint32_t isrflags = READ_REG(huart->Instance->ISR);

    if((isrflags & USART_ISR_IDLE) != RESET) {
        HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_2);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        checkReceiveData();
    }


    HAL_UART_Receive_DMA(ghuart, rx_Uart_buffer, UART_BUF_SIZE);
}
