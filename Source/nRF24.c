/*
 * nRF24.c
 *
 *  Created on: Aug 20, 2021
 *      Author: janoko
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nRF24_conf.h"
#include "nRF24.h"


static void sendCommand(NRF24_HandlerTypedef *hnrf24, uint8_t command, uint8_t *pTxData, uint8_t *pRxData, uint16_t size);
static uint8_t writeBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size);
static uint8_t readBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size);


/**
  * @brief  Setup nrf24 feature.
  *
  * @note   This function setup feature and set nrf24 register (FEATURE) in NRF24_Init()
  *
  * @param  hnrf24 pointer of nrf24 handler
  * @param  feature feature that will be enabled
  *            @arg NRF24_FEATURE_EN_DPL: to enable feature dynamic payload length. 
  *            @arg NRF24_FEATURE_EN_ACK_PAY: to enable feature ack payload. 
  *            @arg NRF24_FEATURE_EN_DYN_ACK: to enable feature dynamic ack trasmit packet. 
  * 
  * @retval None
  */
void NRF24_SetupFeature(NRF24_HandlerTypedef *hnrf24, uint8_t feature)
{
  hnrf24->feature |= feature;
}

__weak void NRF24_Wait(uint32_t ms)
{
  HAL_Delay(ms);
}

__weak uint32_t NRF24_GetTick(void)
{
  return HAL_GetTick();
}

/**
  * @brief  Initialing nrf24.
  *
  * @note   This function will set some nrf24 register.
  *
  * @param  hnrf24 pointer of nrf24 handler
  * 
  * @retval None
  */
void NRF24_Init(NRF24_HandlerTypedef *hnrf24)
{
  uint8_t tmp_reg;
  NRF24_SetMode(hnrf24, NRF24_MODE_PWR_DOWN);

  // set channel and frequency
  if (hnrf24->channel == 0) {
    hnrf24->channel = 2;
  }
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_RF_CH, &(hnrf24->channel));

  tmp_reg = 0;
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_RF_CH, &(tmp_reg));

  // set data rate
  tmp_reg = 0;
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_RF_SETUP, &(tmp_reg));
  tmp_reg &= ~(NRF24_RF_SETUP_DATA_RATE);
  tmp_reg |= (hnrf24->datarate & NRF24_RF_SETUP_DATA_RATE);
  tmp_reg &= ~(NRF24_RF_SETUP_POWER);
  tmp_reg |= (hnrf24->power & NRF24_RF_SETUP_POWER);
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_RF_SETUP, &(tmp_reg));

  // TODO: set retransmitting
  if (hnrf24->retransmitDelay == 0) hnrf24->retransmitDelay = 0x10;
  tmp_reg = hnrf24->retransmitDelay & 0xF0;
  tmp_reg |= hnrf24->retransmitCount & 0x0F;
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_SETUP_RETR, &(tmp_reg));

  if (hnrf24->sendingTimeout == 0) hnrf24->sendingTimeout = 1000;

  // set feature
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_FEATURE, &(hnrf24->feature));  

  // reset interupt
  NRF24_ResetIrq(hnrf24, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

  // reset fifo
  NRF24_FlushTx(hnrf24);
  NRF24_FlushRx(hnrf24);

  // set to standby
  NRF24_SetMode(hnrf24, NRF24_MODE_STANDBY_1);
}


HAL_StatusTypeDef NRF24_Check(NRF24_HandlerTypedef *hnrf24)
{
  uint8_t wData[5] = "nrf24";
  uint8_t rData[5] = {0};

  NRF24_WriteRegister(hnrf24, NRF24_ADDR_TX_ADDR, wData);
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_TX_ADDR, &(rData[0]));
  if (strncmp((char*)rData, (char*)wData, 5) == 0) {
    return HAL_OK;
  }
  return HAL_ERROR;
}


/**
  * @brief  Set a pipe.
  *
  * @param  hnrf24 pointer of nrf24 handler
  * @param  pipeIndex pipe length is 6. select pipe to setting up
  * @param  setup set pipe to enable. (pipe 0 and 1 always enable)
  *            @arg NRF24_PIPE_ENABLE: to set pipe enable.
  *            @arg NRF24_PIPE_ENABLE | NRF24_PIPE_ENABLE_AUTO_ACK: to set pipe automatic send ack
  * @param  width set pipe width
  *
  * @retval None
  */
void NRF24_SetPipe(NRF24_HandlerTypedef *hnrf24,
                     uint8_t pipeIndex, uint8_t setup, uint8_t width, uint8_t *addr)
{
  uint8_t enablePipe;
  uint8_t enablePipeAutoAck;

  NRF24_ReadRegister(hnrf24, NRF24_ADDR_EN_RXADDR, &enablePipe);
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_EN_AA, &enablePipeAutoAck);

  // set up
  hnrf24->pipe[pipeIndex].setup = setup & 0xC0;

  if (addr != NULL) {
    for (uint8_t i = 0; i < 5; i++) {
      hnrf24->pipe[pipeIndex].address[i] = *addr;
      addr++;
    }
  }

  // set to nRF24
  if (pipeIndex > 1) {
    // set pipe enable, default pipe 0 and 1 always enable
    if (NRF24_IS(hnrf24->pipe[pipeIndex].setup, NRF24_PIPE_ENABLE))
      enablePipe |= 1 << pipeIndex;
    else
      enablePipe &= ~(1 << pipeIndex);
    NRF24_WriteRegister(hnrf24, NRF24_ADDR_EN_RXADDR, &enablePipe);

    // set pipe enable auto ack
    if (NRF24_IS(hnrf24->pipe[pipeIndex].setup, NRF24_PIPE_ENABLE_AUTO_ACK))
      enablePipeAutoAck |= 1 << pipeIndex;
    else
      enablePipeAutoAck &= ~(1 << pipeIndex);
    NRF24_WriteRegister(hnrf24, NRF24_ADDR_EN_AA, &enablePipeAutoAck);
  }

  // set pipe width
  if (!NRF24_IS(hnrf24->feature, NRF24_FEATURE_EN_DPL)) {
    width &= NRF24_PIPE_WIDTH;
    hnrf24->pipe[pipeIndex].setup |= width;
    if (width)
      NRF24_WriteRegister(hnrf24, (NRF24_ADDR_RX_PW_P0 + pipeIndex), &width);
  }

  // set pipe address
  if (addr != NULL) {
      NRF24_WriteRegister(hnrf24,
                          (NRF24_ADDR_RX_ADDR_P0 + pipeIndex),
                          &(hnrf24->pipe[pipeIndex].address[0]));
  }
}


void NRF24_SetTxAddress(NRF24_HandlerTypedef *hnrf24, uint8_t *addr)
{
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_TX_ADDR, addr);
}


void NRF24_SetMode(NRF24_HandlerTypedef *hnrf24, NRF24_Mode mode)
{
  uint8_t config_reg;
  if (hnrf24->mode == mode) return;

  NRF24_ReadRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);

  switch(mode){
  case NRF24_MODE_PWR_DOWN:
    if ((config_reg & 0x02) != 0) {
      config_reg &= ~((uint8_t) 0x02);
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_RESET);
    break;

  case NRF24_MODE_STANDBY_1:
    if ((config_reg & 0x02) == 0 || (config_reg & 0x01) != 0) {
      config_reg |= (uint8_t) 0x02;
      config_reg &= ~((uint8_t) 0x01);
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_RESET);
    break;

  case NRF24_MODE_TX:
    // set PRIM_RX to 0
    if ((config_reg & 0x01) != 0) {
      config_reg &= ~((uint8_t) 0x01);
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_SET);
    break;

  case NRF24_MODE_RX:
    // set PRIM_RX to 1
    if ((config_reg & 1) == 0) {
      config_reg |= (uint8_t) 1;
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_SET);
    break;
  }

  if (hnrf24->mode == NRF24_MODE_PWR_DOWN) NRF24_Wait(1);
  NRF24_Wait(1);
  hnrf24->mode = mode;
}


HAL_StatusTypeDef NRF24_SendData(NRF24_HandlerTypedef *hnrf24, uint8_t *data, uint8_t size, uint8_t withAcK)
{
  HAL_StatusTypeDef result = HAL_TIMEOUT;
  uint32_t tick = NRF24_GetTick();

  while ( NRF24_IS_STATUS(hnrf24, NRF24_STATUS_TRANSMITTING) ||
          NRF24_IS_STATUS(hnrf24, NRF24_STATUS_TRANSMITTED)
  ) {
    if (NRF24_GetTick() - tick > hnrf24->sendingTimeout) {
      return result;
    }
    NRF24_Wait(1);
  }

  NRF24_SET_STATUS(hnrf24, NRF24_STATUS_TRANSMITTING);
  NRF24_SetMode(hnrf24, NRF24_MODE_STANDBY_1);

  if (withAcK)
    NRF24_WritePayload(hnrf24, data, size);
  else
    NRF24_WritePayloadNoAck(hnrf24, data, size);
  NRF24_SetMode(hnrf24, NRF24_MODE_TX);

  // if interrupt is disable
  if (!NRF24_IS(hnrf24->irqEnabled, NRF24_IRQ_TX)) {
    NRF24_UNSET_STATUS(hnrf24, NRF24_STATUS_TRANSMITTING);
    return HAL_OK;
  }

  // wait interrupt
  while(1) {
    if (withAcK || !NRF24_IS(hnrf24->feature, NRF24_FEATURE_EN_DYN_ACK)){
      NRF24_SetMode(hnrf24, NRF24_MODE_RX);
    }
    if (NRF24_IS_STATUS(hnrf24, NRF24_STATUS_TRANSMITTING)) {
      if (NRF24_GetTick() - tick > hnrf24->sendingTimeout) {
        return result;
      }
      // retransmitting
      NRF24_SetMode(hnrf24, NRF24_MODE_TX);
      continue;
    }
    break;
  }

  if (NRF24_IS_STATUS(hnrf24, NRF24_STATUS_TRANSMITTED)) {
    NRF24_UNSET_STATUS(hnrf24, NRF24_STATUS_TRANSMITTED);
    result = HAL_OK;
  } else {
    result = HAL_ERROR;
  }
  return result;
}


void NRF24_SendDataNoAck(NRF24_HandlerTypedef *hnrf24, uint8_t *data, uint8_t size)
{
  NRF24_WritePayloadNoAck(hnrf24, data, size);
  NRF24_SetMode(hnrf24, NRF24_MODE_TX);
}


uint8_t NRF24_ReadData(NRF24_HandlerTypedef *hnrf24, uint8_t *data, uint8_t size)
{
  uint8_t readLen = 0;
  uint8_t maxTry = 3;
  while (size) {
    // try to read buffer
    uint8_t bufLen = readBuffer(&(hnrf24->buffer), data, size);
    
    readLen += bufLen;
    size -= bufLen;
    maxTry--;
    if (!maxTry) break;
    if (size) NRF24_Wait(10);
  }
  return readLen;
}

void NRF24_FlushData(NRF24_HandlerTypedef *hnrf24)
{
  hnrf24->buffer.length = 0;
  hnrf24->buffer.posRead = 0;
  hnrf24->buffer.posWrite = 0;
}


void NRF24_EnableIrq(NRF24_HandlerTypedef *hnrf24, NRF24_Irq irq)
{
  uint8_t config;
  hnrf24->irqEnabled |= irq;

  NRF24_ReadRegister(hnrf24, NRF24_ADDR_CONFIG, &config);
  config &= ~(hnrf24->irqEnabled & 0x70);
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config);
}


void NRF24_DisableIrq(NRF24_HandlerTypedef *hnrf24, NRF24_Irq irq)
{
  uint8_t config;
  hnrf24->irqEnabled &= ~(irq);

  NRF24_ReadRegister(hnrf24, NRF24_ADDR_CONFIG, &config);
  config &= ~(hnrf24->irqEnabled & 0x70);
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config);
}


NRF24_Irq NRF24_IrqHandler(NRF24_HandlerTypedef *hnrf24)
{
  uint8_t nrf_status;
  uint8_t tmpData[32];
  uint8_t payloadWidth;
  uint8_t pipeIndex;
  NRF24_Irq irq = 0;

  NRF24_ReadRegister(hnrf24, NRF24_ADDR_STATUS, &nrf_status);
  pipeIndex = (nrf_status & NRF24_STATUS_RX_P_NO);

  if (NRF24_IS(nrf_status, NRF24_STATUS_RX_DR) && pipeIndex != NRF24_STATUS_RX_FIFO_EMPTY) {
    // pipe index shift from NRF24_STATUS_RX_P_NO to get index number;
    pipeIndex = pipeIndex >> 1;

    // read payload
    payloadWidth = NRF24_ReadPayload(hnrf24, pipeIndex, tmpData);

    // reset interupt
    NRF24_ResetIrq(hnrf24, NRF24_IRQ_RX);
    writeBuffer(&(hnrf24->buffer), tmpData, payloadWidth);
    irq = NRF24_IRQ_RX;
  }

  if (NRF24_IS(nrf_status, NRF24_STATUS_TX_DS)) {
    NRF24_ResetIrq(hnrf24, NRF24_IRQ_TX);
    NRF24_FlushTx(hnrf24);
    NRF24_SET_STATUS(hnrf24, NRF24_STATUS_TRANSMITTED);
    NRF24_UNSET_STATUS(hnrf24, NRF24_STATUS_TRANSMITTING);
    irq = NRF24_IRQ_TX;
  }

  if (NRF24_IS(nrf_status, NRF24_STATUS_MAX_RT)) {
    NRF24_ResetIrq(hnrf24, NRF24_IRQ_MAX_RT);
//    NRF24_UNSET_STATUS(hnrf24, NRF24_STATUS_TRANSMITTING);
    irq = NRF24_IRQ_MAX_RT;
  }
  return irq;
}


void NRF24_ResetIrq(NRF24_HandlerTypedef *hnrf24, NRF24_Irq irq)
{
  uint8_t nrf_status;
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_STATUS, &nrf_status);
  nrf_status |= irq & 0x70;
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_STATUS, &nrf_status);
}


void NRF24_ReadRegister(NRF24_HandlerTypedef *hnrf24, uint8_t addr, uint8_t *result)
{
  uint8_t register_len = 1;
  if (  addr == NRF24_ADDR_TX_ADDR
    ||  addr == NRF24_ADDR_RX_ADDR_P0
    ||  addr == NRF24_ADDR_RX_ADDR_P1
  ){
    register_len = 5;
  }
  sendCommand(hnrf24, NRF24_CMD_R_REGISTER|addr, NULL, result, register_len);
}

void NRF24_WriteRegister(NRF24_HandlerTypedef *hnrf24, uint8_t addr, uint8_t *value)
{
  uint8_t register_len = 1;
  if (  addr == NRF24_ADDR_TX_ADDR
    ||  addr == NRF24_ADDR_RX_ADDR_P0
    ||  addr == NRF24_ADDR_RX_ADDR_P1
  ){
    register_len = 5;
  }
  sendCommand(hnrf24, NRF24_CMD_W_REGISTER|addr, value, NULL, register_len);
}

uint8_t NRF24_ReadPayload(NRF24_HandlerTypedef *hnrf24, uint8_t pipeIndex, uint8_t *result)
{
  uint8_t size;

  if (NRF24_IS(hnrf24->feature, NRF24_FEATURE_EN_DPL)) {
    // if nrf24 dynamic payload length feature enable
    size = NRF24_RecivedPayloadWidth(hnrf24);
  } 
  else {
    size = hnrf24->pipe[pipeIndex].setup & NRF24_PIPE_WIDTH;
  }
  sendCommand(hnrf24, NRF24_CMD_R_RX_PAYLOAD, NULL, result, size);
  return size;
}

uint8_t NRF24_RecivedPayloadWidth(NRF24_HandlerTypedef *hnrf24)
{
  uint8_t rxPayW = 0;
  sendCommand(hnrf24, NRF24_CMD_R_RX_PL_WID, NULL, &rxPayW, 1);
  return rxPayW;
}

void NRF24_WritePayload(NRF24_HandlerTypedef *hnrf24, uint8_t *value, uint8_t size)
{
  if (size > 32) size = 32;
  sendCommand(hnrf24, NRF24_CMD_W_TX_PAYLOAD, value, NULL, size);
}

void NRF24_WritePayloadNoAck(NRF24_HandlerTypedef *hnrf24, uint8_t *value, uint8_t size)
{
  if (size > 32) size = 32;
  sendCommand(hnrf24, NRF24_CMD_W_TX_PAYLOAD_NOACK, value, NULL, size);
}

void NRF24_FlushRx(NRF24_HandlerTypedef *hnrf24)
{
  sendCommand(hnrf24, NRF24_CMD_FLUSH_RX, NULL, NULL, 0);
}

void NRF24_FlushTx(NRF24_HandlerTypedef *hnrf24)
{
  sendCommand(hnrf24, NRF24_CMD_FLUSH_TX, NULL, NULL, 0);
}

void NRF24_WriteAck(NRF24_HandlerTypedef *hnrf24, uint8_t *value, uint8_t size)
{
  if (size > 32) size = 32;
  sendCommand(hnrf24, NRF24_CMD_W_ACK_PAYLOAD, value, NULL, size);
}

static void sendCommand(NRF24_HandlerTypedef *hnrf24, uint8_t command, uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
  uint8_t tmp_pRxData[1+size];
  uint8_t tmp_pTxData[1+size];
  tmp_pTxData[0] = command;

  // if transmit pointer data is not null
  for (uint16_t i = 1; i <= size; i++) {
    if (pTxData != NULL) {
      tmp_pTxData[i] = *pTxData;
      pTxData++;
    }
    else {
      tmp_pTxData[i] = 0;
    }
  }

  while (__HAL_SPI_GET_FLAG(hnrf24->hspi, SPI_FLAG_BSY)) {
    NRF24_Wait(1);
  };
  HAL_GPIO_WritePin((hnrf24->pin.CSN).GPIOx, (hnrf24->pin.CSN).GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hnrf24->hspi, tmp_pTxData, tmp_pRxData, 1 + size, HAL_MAX_DELAY);
  HAL_GPIO_WritePin((hnrf24->pin.CSN).GPIOx, (hnrf24->pin.CSN).GPIO_Pin, GPIO_PIN_SET);
  
  // if recive pointer data is not null
  if (pRxData != NULL && size != 0) {
    for (uint16_t i = 1; i <= size; i++) {
      *pRxData = tmp_pRxData[i];
      pRxData++;
    }
  }
  return;
}

static uint8_t writeBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size)
{
  uint8_t wroteSize = 0;
  while (buffer->length < NRF24_BUFFER_SIZE && size) {
    buffer->buf[buffer->posWrite] = *data;

    // shift posRead if overlap
    if (buffer->length != 0 && buffer->posWrite == buffer->posRead) {
      buffer->posRead++;
      if (buffer->posRead == NRF24_BUFFER_SIZE) buffer->posRead = 0;
    }

    data++;
    size--;
    wroteSize++;
    buffer->length++;
    buffer->posWrite++;
    if (buffer->posWrite == NRF24_BUFFER_SIZE) buffer->posWrite = 0;
  }
  return wroteSize;
}

static uint8_t readBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size)
{
  uint8_t readSize = 0;
  while (buffer->length && size) {
    *data = buffer->buf[buffer->posRead];
    data++;
    size--;
    readSize++;
    buffer->length--;
    buffer->posRead++;
    if (buffer->posRead == NRF24_BUFFER_SIZE) buffer->posRead = 0;
  }
  return readSize;
}
