/*
 * nRF24.h
 *
 *  Created on: Aug 20, 2021
 *      Author: janoko
 */

#ifndef DRIVER_INC_NRF24_H_
#define DRIVER_INC_NRF24_H_

#include "stm32f4xx_hal.h"
#include "nRF24_conf.h"

#define NRF24_PIPE_ENABLE 0x80
#define NRF24_PIPE_ENABLE_AUTO_ACK 0x40
#define NRF24_PIPE_WIDTH 0x3F

#define NRF24_STATUS_TRANSMITTING 0x01
#define NRF24_STATUS_TRANSMITTED 0x02

#define NRF24_CMD_R_REGISTER 0x00
#define NRF24_CMD_W_REGISTER 0x20
#define NRF24_CMD_R_RX_PAYLOAD 0x61
#define NRF24_CMD_W_TX_PAYLOAD 0xA0
#define NRF24_CMD_FLUSH_TX 0xE1
#define NRF24_CMD_FLUSH_RX 0xE2
#define NRF24_CMD_REUSE_TX_PL 0xE3
#define NRF24_CMD_R_RX_PL_WID 0x60
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24_CMD_NOP 0xFF

#define NRF24_ADDR_CONFIG 0x00
#define NRF24_ADDR_EN_AA 0x01
#define NRF24_ADDR_EN_RXADDR 0x02
#define NRF24_ADDR_SETUP_AW 0x03
#define NRF24_ADDR_SETUP_RETR 0x04
#define NRF24_ADDR_RF_CH 0x05
#define NRF24_ADDR_RF_SETUP 0x06
#define NRF24_ADDR_STATUS 0x07
#define NRF24_ADDR_OBSERVE_TX 0x08
#define NRF24_ADDR_RPD 0x09
#define NRF24_ADDR_RX_ADDR_P0 0x0A
#define NRF24_ADDR_RX_ADDR_P1 0x0B
#define NRF24_ADDR_RX_ADDR_P2 0x0C
#define NRF24_ADDR_RX_ADDR_P3 0x0D
#define NRF24_ADDR_RX_ADDR_P4 0x0E
#define NRF24_ADDR_RX_ADDR_P5 0x0F
#define NRF24_ADDR_TX_ADDR 0x10
#define NRF24_ADDR_RX_PW_P0 0x11
#define NRF24_ADDR_RX_PW_P1 0x12
#define NRF24_ADDR_RX_PW_P2 0x13
#define NRF24_ADDR_RX_PW_P3 0x14
#define NRF24_ADDR_RX_PW_P4 0x15
#define NRF24_ADDR_RX_PW_P5 0x16
#define NRF24_ADDR_FIFO_STATUS 0x17
#define NRF24_ADDR_DYNPD 0x1C
#define NRF24_ADDR_FEATURE 0x1D
#define NRF24_ADDR_MAX 0x1E

#define NRF24_RF_SETUP_DATA_RATE 0x28
#define NRF24_DATA_RATE_250Kbps 0x20
#define NRF24_DATA_RATE_1Mbps 0x00
#define NRF24_DATA_RATE_2Mbps 0x08

#define NRF24_RF_SETUP_POWER 0x06
#define NRF24_POWER_M18DBM 0x00
#define NRF24_POWER_M12DBM 0x02
#define NRF24_POWER_M6DBM 0x04
#define NRF24_POWER_0DBM 0x06

#define NRF24_STATUS_RX_DR 0x40
#define NRF24_STATUS_TX_DS 0x20
#define NRF24_STATUS_MAX_RT 0x10
#define NRF24_STATUS_RX_P_NO 0x0E
#define NRF24_STATUS_RX_FIFO_EMPTY 0x0E
#define NRF24_STATUS_TX_FULL 0x01

#define NRF24_IRQ_RX 0x40
#define NRF24_IRQ_TX 0x20
#define NRF24_IRQ_MAX_RT 0x10

#define NRF24_FEATURE_EN_DPL 0x04
#define NRF24_FEATURE_EN_ACK_PAY 0x02
#define NRF24_FEATURE_EN_DYN_ACK 0x01

#define NRF24_MODE_PWR_DOWN 0x00
#define NRF24_MODE_STANDBY_1 0x01
#define NRF24_MODE_STANDBY_2 0x02
#define NRF24_MODE_TX 0x03
#define NRF24_MODE_RX 0x04

#define NRF24_IS(byte, bits) (((byte) & (bits)) == (bits))
#define NRF24_IS_STATUS(nrf24, stat) NRF24_IS((nrf24)->status, (stat))
#define NRF24_SET_STATUS(nrf24, stat) { (nrf24)->status |= (stat); }
#define NRF24_UNSET_STATUS(nrf24, stat) { (nrf24)->status &= ~(stat); }

//#define NRF24_GPIO_SET(pin) (HAL_GPIO_WritePin((pin).GPIOx, (pin).GPIO_Pin, GPIO_PIN_SET))
//#define NRF24_GPIO_RESET(pin) (HAL_GPIO_WritePin((pin).GPIOx, (pin).GPIO_Pin, GPIO_PIN_RESET))

typedef uint8_t NRF24_Mode;
typedef uint8_t NRF24_Irq;

struct NRF24_GPIO_pin {
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
};

typedef struct NRF24_Buffer {
  uint8_t buf[NRF24_BUFFER_SIZE];
  uint8_t posRead;
  uint8_t posWrite;
  uint8_t length;
} NRF24_Buffer;

typedef struct NRF24_Pipe {
  uint8_t setup;          // #7(reg: EN_RXADDR) #6(reg: EN_AA) #5:0(reg: RX_PW_Px)
  uint8_t address[5];     // reg: RX_ADDR_Px
} NRF24_Pipe;

typedef struct NRF24_Handler {
  NRF24_Mode mode;
  uint8_t status;

  // pin
  SPI_HandleTypeDef *hspi;
  struct {
    struct NRF24_GPIO_pin CSN;
    struct NRF24_GPIO_pin CE;
  } pin;

  // configuration
  uint8_t channel;              		  // reg: RF_CH
  uint8_t datarate;                   // reg: RF_SETUP (#5 #3)
  uint8_t power;                      // reg: RF_SETUP (#2 #1)
  uint8_t retransmitDelay;        		// reg: SETUP_RETR (#7:4)
  uint8_t retransmitCount;          		// reg: SETUP_RETR (#7:4)
  uint8_t txAddress;            	    // reg: TX_ADDR
  uint8_t txPayloadWidth;
  uint8_t pipeNumber;
  NRF24_Pipe pipe[NRF24_PIPE_NUMBER];
  uint8_t feature;
  NRF24_Irq irqEnabled;
  uint32_t sendingTimeout;
  // struct {
  //   uint8_t isDynamicPayloadLen;
  //   uint8_t isEnableAckPayload;
  //   uint8_t isEnableNoAck;
  // } feature;

  // buffer
  NRF24_Buffer buffer;

} NRF24_HandlerTypedef;

// setup
void NRF24_SetupFeature(NRF24_HandlerTypedef *hnrf24, uint8_t feature);
void NRF24_Wait(uint32_t ms);
uint32_t NRF24_GetTick(void);

// running proccess

void NRF24_Init(NRF24_HandlerTypedef *hnrf24);
HAL_StatusTypeDef NRF24_Check(NRF24_HandlerTypedef *hnrf24);
void NRF24_SetPipe(NRF24_HandlerTypedef *hnrf24,
                     uint8_t pipeIndex, uint8_t setup, uint8_t width, uint8_t *addr);
void NRF24_SetTxAddress(NRF24_HandlerTypedef *hnrf24, uint8_t *addr);
void NRF24_SetMode(NRF24_HandlerTypedef *hnrf24, NRF24_Mode mode);
HAL_StatusTypeDef NRF24_SendData(NRF24_HandlerTypedef *hnrf24, uint8_t *data, uint8_t size, uint8_t withAcK);
void NRF24_SendDataNoAck(NRF24_HandlerTypedef *hnrf24, uint8_t *data, uint8_t size);
uint8_t NRF24_ReadData(NRF24_HandlerTypedef *hnrf24, uint8_t *data, uint8_t size);
void NRF24_FlushData(NRF24_HandlerTypedef *hnrf24);
void NRF24_EnableIrq(NRF24_HandlerTypedef *hnrf24, NRF24_Irq);
void NRF24_DisableIrq(NRF24_HandlerTypedef *hnrf24, NRF24_Irq);
void NRF24_ResetIrq(NRF24_HandlerTypedef *hnrf24, NRF24_Irq);
NRF24_Irq NRF24_IrqHandler(NRF24_HandlerTypedef *hnrf24);

// spi command

void NRF24_ReadRegister(NRF24_HandlerTypedef *hnrf24, uint8_t addr, uint8_t *result);
void NRF24_WriteRegister(NRF24_HandlerTypedef *hnrf24, uint8_t addr, uint8_t *value);
uint8_t NRF24_ReadPayload(NRF24_HandlerTypedef *hnrf24, uint8_t pipeIndex, uint8_t *result);
uint8_t NRF24_RecivedPayloadWidth(NRF24_HandlerTypedef *hnrf24);
void NRF24_WritePayload(NRF24_HandlerTypedef *hnrf24, uint8_t *value, uint8_t size);
void NRF24_WritePayloadNoAck(NRF24_HandlerTypedef *hnrf24, uint8_t *value, uint8_t size);
void NRF24_FlushRx(NRF24_HandlerTypedef *hnrf24);
void NRF24_FlushTx(NRF24_HandlerTypedef *hnrf24);
void NRF24_WriteAck(NRF24_HandlerTypedef *hnrf24, uint8_t *value, uint8_t size);

#endif /* DRIVER_INC_NRF24_H_ */
