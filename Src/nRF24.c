/*
 * nRF24.c
 *
 *  Created on: Aug 20, 2021
 *      Author: janoko
 */

#include <stdio.h>
#include <stdlib.h>
#include "nRF24.h"

static void sendCommand(NRF24_Handler *hnrf24, uint8_t command, uint8_t *pTxData, uint8_t *pRxData, uint16_t size);
static uint8_t writeBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size);
static uint8_t readBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size);


/**
  * @brief  Setup a pipe.
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
void NRF24_SetupPipe(NRF24_Handler *hnrf24, uint8_t pipeIndex, uint8_t setup, uint8_t width)
{
  hnrf24->pipe[pipeIndex].setup = setup & 0xC0;
  hnrf24->pipe[pipeIndex].setup |= width & NRF24_PIPE_WIDTH;
}


/**
  * @brief  Setup address of pipe selected.
  * 
  * @param  hnrf24 pointer of nrf24 handler
  * @param  pipeIndex pipe length is 6. select pipe to setting up
  * @param  address address bytes. for pipe 0 and 1, its max length is 5. and 1 for else 
  * 
  * @retval None
  */
void NRF24_SetupPipeAddress(NRF24_Handler *hnrf24, uint8_t pipeIndex, uint8_t *address)
{
  hnrf24->pipe[pipeIndex].address = address;
}


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
void NRF24_SetupFeature(NRF24_Handler *hnrf24, uint8_t feature)
{
  hnrf24->feature &= feature;
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
void NRF24_Init(NRF24_Handler *hnrf24)
{
  uint8_t tmp_reg;
  NRF24_SetMode(hnrf24, NRF24_MODE_PWR_DOWN);

  // set channel and frequency
  if(hnrf24->channel == 0){
    hnrf24->channel = 2;
  }
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_RF_CH, &(hnrf24->channel));

  // set data rate
  tmp_reg = 0;
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_RF_SETUP, &(tmp_reg));
  tmp_reg |= hnrf24->datarate;
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_RF_SETUP, &(tmp_reg));

  // set retransmitting
  if(hnrf24->retransmitMax == 0x00) hnrf24->retransmitMax = 0x03;  // default
  else if(hnrf24->retransmitMax & 0xF0) hnrf24->retransmitMax = 0;

  tmp_reg = hnrf24->retransmitMax & 0x0F;
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_RF_SETUP, &(tmp_reg));

  // set pipe
  uint8_t addr_rx_pw = NRF24_ADDR_RX_PW_P0;
  uint8_t addr_rx_addr = NRF24_ADDR_RX_ADDR_P0;
  uint8_t enablePipe = 3;
  uint8_t enablePipeAutoAck = 3;
  for(int i = 0; i < NRF24_PIPE_NUMBER; i++){
    // set pipe enable, default pipe 0 and 1 always enable
    if(i > 1 && NRF24_IS(hnrf24->pipe[i].setup, NRF24_PIPE_ENABLE)){
      enablePipe |= 1 << i;
    }

    // set pipe enable auto ack
    if(i > 1 && NRF24_IS(hnrf24->pipe[i].setup, NRF24_PIPE_ENABLE_AUTO_ACK)){
      enablePipeAutoAck |= 1 << i;
    }

    // set pipe width
    if(!NRF24_IS(hnrf24->feature, NRF24_FEATURE_EN_DPL)){
      uint8_t pipeRxWidth = hnrf24->pipe[i].setup & NRF24_PIPE_WIDTH;
      if(pipeRxWidth)
        NRF24_WriteRegister(hnrf24, addr_rx_pw, &pipeRxWidth);
    }

    // set pipe address
    if(hnrf24->pipe[i].address != NULL){
        NRF24_WriteRegister(hnrf24, addr_rx_addr, hnrf24->pipe[i].address);
    }

    addr_rx_pw++;
    addr_rx_addr++;
  }
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_EN_RXADDR, &enablePipe);
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_EN_AA, &enablePipeAutoAck);

  // set feature
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_FEATURE, &(hnrf24->feature));  

  // reset interupt
  NRF24_ResetInterupt(hnrf24, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

  // reset fifo
  NRF24_FlushTx(hnrf24);
  NRF24_FlushRx(hnrf24);

  // set to standby
  NRF24_SetMode(hnrf24, NRF24_MODE_STANDBY_1);
}


void NRF24_SetMode(NRF24_Handler *hnrf24, NRF24_Mode mode)
{
  uint8_t config_reg;
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);

  switch(mode){
  case NRF24_MODE_PWR_DOWN:
    if((config_reg & 2) != 0){
      config_reg &= ~((uint8_t) 2);
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_RESET);
    break;

  case NRF24_MODE_STANDBY_1:
    if((config_reg & 2) == 0){
      config_reg |= (uint8_t) 2;
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_RESET);
    break;

  case NRF24_MODE_TX:
    // set PRIM_RX to 0
    if((config_reg & 1) != 0){
      config_reg &= ~((uint8_t) 1);
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_SET);
    break;

  case NRF24_MODE_RX:
    // set PRIM_RX to 1
    if((config_reg & 1) == 0){
      config_reg |= (uint8_t) 1;
      NRF24_WriteRegister(hnrf24, NRF24_ADDR_CONFIG, &config_reg);
    }
    HAL_GPIO_WritePin((hnrf24->pin.CE).GPIOx, (hnrf24->pin.CE).GPIO_Pin, GPIO_PIN_SET);
    break;

  }
  HAL_Delay(2);
}

void NRF24_SendData(NRF24_Handler *hnrf24, uint8_t *data, uint8_t size)
{
  if(size != hnrf24->txPayloadWidth) size = hnrf24->txPayloadWidth;
  NRF24_WritePayload(hnrf24, data, size);
//  NRF24_FlushTx(hnrf24);
  NRF24_SetMode(hnrf24, NRF24_MODE_TX);
}

void NRF24_SendDataNoAck(NRF24_Handler *hnrf24, uint8_t *data, uint8_t size)
{
  if(size != hnrf24->txPayloadWidth) size = hnrf24->txPayloadWidth;
  NRF24_WritePayloadNoAck(hnrf24, data, size);
//  NRF24_FlushTx(hnrf24);
  NRF24_SetMode(hnrf24, NRF24_MODE_TX);
}

uint8_t NRF24_ReadData(NRF24_Handler *hnrf24, uint8_t *data, uint8_t size)
{
  uint8_t readLen = 0;
  uint8_t maxTry = 3;
  while(size){
    // try to read buffer
    uint8_t bufLen = readBuffer(&(hnrf24->buffer), data, size);
    
    readLen += bufLen;
    size -= bufLen;
    maxTry--;
    if(!maxTry) break;
    if(size) HAL_Delay(10);
  }
  return readLen;
}

void NRF24_RxInteruptHandler(NRF24_Handler *hnrf24)
{
  uint8_t nrf_status;
  uint8_t tmpData[32];
  uint8_t payloadWidth;
  uint8_t pipeIndex;

  NRF24_ReadRegister(hnrf24, NRF24_ADDR_STATUS, &nrf_status);
  pipeIndex = (nrf_status & NRF24_STATUS_RX_P_NO);
  if(NRF24_IS(nrf_status, NRF24_STATUS_RX_DR) && pipeIndex != NRF24_STATUS_RX_FIFO_EMPTY){
    // pipe index shift from NRF24_STATUS_RX_P_NO to get index number;
    pipeIndex = pipeIndex >> 1;

    // read payload
    payloadWidth = NRF24_ReadPayload(hnrf24, pipeIndex, tmpData);

    // reset interupt
    NRF24_ResetInterupt(hnrf24, NRF24_STATUS_RX_DR);
    writeBuffer(&(hnrf24->buffer), tmpData, payloadWidth);
  }
}

void NRF24_ResetInterupt(NRF24_Handler *hnrf24, uint8_t interupt_register)
{
  uint8_t nrf_status;
  NRF24_ReadRegister(hnrf24, NRF24_ADDR_STATUS, &nrf_status);
  nrf_status |= interupt_register;
  NRF24_WriteRegister(hnrf24, NRF24_ADDR_STATUS, &nrf_status);
}


void NRF24_ReadRegister(NRF24_Handler *hnrf24, uint8_t addr, uint8_t *result)
{
  uint8_t register_len = 1;
  if(   NRF24_IS(addr, NRF24_ADDR_TX_ADDR)
    ||  NRF24_IS(addr, NRF24_ADDR_RX_ADDR_P0)
    ||  NRF24_IS(addr, NRF24_ADDR_RX_ADDR_P1)
  ){
    register_len = 5;
  }
  sendCommand(hnrf24, NRF24_CMD_R_REGISTER|addr, NULL, result, register_len);
}

void NRF24_WriteRegister(NRF24_Handler *hnrf24, uint8_t addr, uint8_t *value)
{
  uint8_t register_len = 1;
  if(   NRF24_IS(addr, NRF24_ADDR_TX_ADDR)
    ||  NRF24_IS(addr, NRF24_ADDR_RX_ADDR_P0)
    ||  NRF24_IS(addr, NRF24_ADDR_RX_ADDR_P1)
  ){
    register_len = 5;
  }
  sendCommand(hnrf24, NRF24_CMD_W_REGISTER|addr, value, NULL, register_len);
}

uint8_t NRF24_ReadPayload(NRF24_Handler *hnrf24, uint8_t pipeIndex, uint8_t *result)
{
  uint8_t size;

  if(NRF24_IS(hnrf24->feature, NRF24_FEATURE_EN_DPL)){
    // if nrf24 dynamic payload length feature enable
    size = NRF24_RecivedPayloadWidth(hnrf24);
  } 
  else {
    size = hnrf24->pipe[pipeIndex].setup & NRF24_PIPE_WIDTH;
  }
  sendCommand(hnrf24, NRF24_CMD_R_RX_PAYLOAD, NULL, result, size);
  return size;
}

uint8_t NRF24_RecivedPayloadWidth(NRF24_Handler *hnrf24)
{
  uint8_t rxPayW = 0;
  sendCommand(hnrf24, NRF24_CMD_R_RX_PL_WID, NULL, &rxPayW, 1);
  return rxPayW;
}

void NRF24_WritePayload(NRF24_Handler *hnrf24, uint8_t *value, uint8_t size)
{
  if(size > 32) size = 32;
  sendCommand(hnrf24, NRF24_CMD_W_TX_PAYLOAD, value, NULL, size);
}

void NRF24_WritePayloadNoAck(NRF24_Handler *hnrf24, uint8_t *value, uint8_t size)
{
  if(size > 32) size = 32;
  sendCommand(hnrf24, NRF24_CMD_W_TX_PAYLOAD_NOACK, value, NULL, size);
}

void NRF24_FlushRx(NRF24_Handler *hnrf24)
{
  sendCommand(hnrf24, NRF24_CMD_FLUSH_RX, NULL, NULL, 0);
}

void NRF24_FlushTx(NRF24_Handler *hnrf24)
{
  sendCommand(hnrf24, NRF24_CMD_FLUSH_TX, NULL, NULL, 0);
}

void NRF24_WriteAck(NRF24_Handler *hnrf24, uint8_t *value, uint8_t size)
{
  if(size > 32) size = 32;
  sendCommand(hnrf24, NRF24_CMD_W_ACK_PAYLOAD, value, NULL, size);
}

static void sendCommand(NRF24_Handler *hnrf24, uint8_t command, uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
  uint8_t tmp_pRxData[1+size];
  uint8_t tmp_pTxData[1+size];
  tmp_pTxData[0] = command;

  // if transmit pointer data is not null
  for(int i = 1; i <= size; i++){
    if(pTxData != NULL){
      tmp_pTxData[i] = *pTxData;
      pTxData++;
    }
    else {
      tmp_pTxData[i] = 0;
    }
  }

  while (__HAL_SPI_GET_FLAG(hnrf24->hspi, SPI_FLAG_BSY)) {
    HAL_Delay(1);
  };
  HAL_GPIO_WritePin((hnrf24->pin.CSN).GPIOx, (hnrf24->pin.CSN).GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hnrf24->hspi, tmp_pTxData, tmp_pRxData, 1 + size, HAL_MAX_DELAY);
  HAL_GPIO_WritePin((hnrf24->pin.CSN).GPIOx, (hnrf24->pin.CSN).GPIO_Pin, GPIO_PIN_SET);
  
  // if recive pointer data is not null
  if(pRxData != NULL && size != 0){
    for(int i = 1; i <= size; i++){
      *pRxData = tmp_pRxData[i];
      pRxData++;
    }
  }
  return;
}

static uint8_t writeBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size)
{
  uint8_t wroteSize = 0;
  while(buffer->length < NRF24_BUFFER_SIZE && size){
    if(buffer->posWrite == NRF24_BUFFER_SIZE) buffer->posWrite = 0;
    buffer->buf[buffer->posWrite] = *data;
    data++;
    wroteSize++;
    buffer->length++;
    buffer->posWrite++;
    size--;
  }
  return wroteSize;
}

static uint8_t readBuffer(NRF24_Buffer *buffer, uint8_t *data, uint8_t size)
{
  uint8_t readSize = 0;
  while(buffer->length && size){
    if(buffer->posRead == NRF24_BUFFER_SIZE) buffer->posRead = 0;
    *data = buffer->buf[buffer->posRead];
    data++;
    readSize++;
    buffer->length--;
    buffer->posRead++;
    size--;
  }
  return readSize;
}
