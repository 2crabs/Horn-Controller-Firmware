#include "Display.h"

HAL_StatusTypeDef Display_SetDigits(TCA6424 *IOExpander, uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t mode){
	if((digit1>9) || (digit2>9) || (digit3>9)){
		return HAL_ERROR;
	}

	static uint8_t digit1Values[10] = DIGIT1_VALUES;
	static uint8_t digit23Values[10] = DIGIT23_VALUES;

	uint8_t ports[3];
	for(uint32_t i = 0; i<3; i++){
		ports[i] = 0;
	}

	if(mode & DISPLAY_MODE_DIGIT1){
		ports[0] = digit1Values[digit1];
	}

	if(mode & DISPLAY_MODE_DIGIT2){
		ports[0] = (digit23Values[digit2] << 7) | ports[0];
		ports[1] = digit23Values[digit2] >> 1;
	}

	if(mode & DISPLAY_MODE_DIGIT3){
		ports[1] = ports[1] | (digit23Values[digit3] << 6);
		ports[2] = digit23Values[digit3] >> 2;
	}

	return TCA6424_WriteRegisters(IOExpander, TCA6424_REG_OUT0 | TCA6424_AUTO_INCREMENT, ports, 3);
}

HAL_StatusTypeDef Display_SetValue(TCA6424 *IOExpander, uint16_t value, uint8_t mode){
	if (value > 999){
		return HAL_ERROR;
	}
	uint8_t hundreds = value / 100;
	uint8_t tens = (value / 10) % 10;
	uint8_t ones = value % 10;

	Display_SetDigits(IOExpander, hundreds, tens, ones, mode);
}

HAL_StatusTypeDef Display_SetMinSec(TCA6424 *IOExpander, uint8_t minutes,  uint8_t seconds){
	Display_SetDigits(IOExpander, minutes, seconds/10, seconds%10, DISPLAY_MODE_ALL);
}
