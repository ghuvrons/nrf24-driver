# nRF24 Driver for STM32f4xx

## Intalisation

Clone this ptoject in "Drivers" directory in your stm32 project.

```bash
git clone https://github.com/ghuvrons/stm32f4xx-nrf24.git --depth 1 Drivers/nRF24
```

Than add this path directory in Project > Properties > C/C++ Build > Settings > Tool Settings > Include paths

## Configuration

Above is example transceiver configuration.

```C
#include "nRF24.h"

NRF24_Handler hnrf24_1;

static void MX_NRF24_Init(void)
{
  hnrf24_1.hspi = &hspi4;
  hnrf24_1.pin.CSN.GPIOx = GPIOE;
  hnrf24_1.pin.CSN.GPIO_Pin = GPIO_PIN_4;
  hnrf24_1.pin.CE.GPIOx = GPIOE;
  hnrf24_1.pin.CE.GPIO_Pin = GPIO_PIN_3;
  hnrf24_1.txPayloadWidth = 16;
  NRF24_SetFeature(&hnrf24_1, NRF24_FEATURE_EN_DYN_ACK);

  NRF24_Init(&hnrf24_1);
}
```

Above is example reciver configuration.

```C
#include "nRF24.h"

NRF24_Handler hnrf24_1;

static void MX_NRF24_Init(void)
{
  hnrf24_1.hspi = &hspi3;
  hnrf24_1.pin.CSN.GPIOx = GPIOA;
  hnrf24_1.pin.CSN.GPIO_Pin = GPIO_PIN_15;
  hnrf24_1.pin.CE.GPIOx = GPIOC;
  hnrf24_1.pin.CE.GPIO_Pin = GPIO_PIN_12;
  hnrf24_1.txPayloadWidth = 16;
  NRF24_SetupFeature(&hnrf24_1, NRF24_FEATURE_EN_ACK_PAY);
  NRF24_SetupPipe(&hnrf24_1, 0, NRF24_PIPE_ENABLE, 16);

  NRF24_Init(&hnrf24_1);
}
