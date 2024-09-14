#include "TCA6424.h"

HAL_StatusTypeDef TCA6424_Init(TCA6424 *dev, I2C_HandleTypeDef *i2c, GPIO_TypeDef *gpio, uint16_t rstPin){
	dev->i2cHandle = i2c;
	dev->gpioHandle = gpio;
	dev->resetPin = rstPin;

	/* Pin needs to be pulled low to reset device to a known state */
	HAL_GPIO_WritePin(dev->gpioHandle, dev->resetPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(dev->gpioHandle, dev->resetPin, GPIO_PIN_SET);
	HAL_Delay(1);

	/* Sets port 0 pins as input to make sure the device is responding */
	uint8_t dummyReg = 0xFF;
	return HAL_I2C_Mem_Write(dev->i2cHandle, TCA6424_I2C_ADDR, TCA6424_REG_CONFIG0, I2C_MEMADD_SIZE_8BIT, &dummyReg, 1, 100);
}

HAL_StatusTypeDef TCA6424_WriteRegister(TCA6424 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, TCA6424_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

HAL_StatusTypeDef TCA6424_WriteRegisters(TCA6424 *dev, uint8_t reg, uint8_t *data, uint8_t size){
	return HAL_I2C_Mem_Write(dev->i2cHandle, TCA6424_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size, 100);
}

HAL_StatusTypeDef TCA6424_SetAsOutputs(TCA6424 *dev){
	uint8_t config[3] = {0x00, 0x00, 0x00};
	//sets auto increment bit to true to be able to write to all three registers in one i2c message
	return TCA6424_WriteRegisters(dev, TCA6424_REG_CONFIG0 | TCA6424_AUTO_INCREMENT, config, 3);
}
