#ifndef INC_TCA6424_H_
#define INC_TCA6424_H_

#include "stm32f0xx_hal.h"

#define TCA6424_I2C_ADDR (0x22 << 1)

#define TCA6424_REG_IN0 0x00
#define TCA6424_REG_IN1 0x01
#define TCA6424_REG_IN2 0x02
#define TCA6424_REG_OUT0 0x04
#define TCA6424_REG_OUT1 0x05
#define TCA6424_REG_OUT2 0x06
#define TCA6424_REG_INV0 0x08
#define TCA6424_REG_INV1 0x09
#define TCA6424_REG_INV2 0x0A
#define TCA6424_REG_CONFIG0 0x0C
#define TCA6424_REG_CONFIG1 0x0D
#define TCA6424_REG_CONFIG2 0x0E

#define TCA6424_AUTO_INCREMENT 0x80

typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	GPIO_TypeDef *gpioHandle;
	uint16_t resetPin;

} TCA6424;

HAL_StatusTypeDef TCA6424_Init(TCA6424 *dev, I2C_HandleTypeDef *i2c, GPIO_TypeDef *gpio, uint16_t rstPin);

HAL_StatusTypeDef TCA6424_WriteRegister(TCA6424 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef TCA6424_WriteRegisters(TCA6424 *dev, uint8_t reg, uint8_t *data, uint8_t size);

HAL_StatusTypeDef TCA6424_SetAsOutputs(TCA6424 *dev);

#endif
