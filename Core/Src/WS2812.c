#include "WS2812.h"

void WS2812_Init(WS2812 *dev, TIM_HandleTypeDef *timer, uint32_t channel){
	dev->tim = timer;
	dev->tim_channel = channel;
}

void WS2812_Reset_Buf(uint8_t *buf){
	for(uint32_t i = 0; i<WS2812_BUF_LEN; i++){
		buf[i] = 0;
	}
}

void WS2812_Write_Buf(uint8_t *buf, uint8_t r, uint8_t g, uint8_t b, uint8_t led){
	uint8_t bitIndex = 0;
	//G 0-7
	for(uint32_t i = (led*24); i<((led*24)+8); i++){
		if ((g>>(7-bitIndex)) & 0x01){
			buf[i] = WS2812_HI_VAL;
		} else {
			buf[i] = WS2812_LO_VAL;
		}
		bitIndex++;
	}
	bitIndex = 0;

	//R 8-15
	for(uint32_t i = ((led*24)+8); i<((led*24)+16); i++){
		if ((r>>(7-bitIndex)) & 0x01){
			buf[i] = WS2812_HI_VAL;
		} else {
			buf[i] = WS2812_LO_VAL;
		}
		bitIndex++;
	}
	bitIndex = 0;

	//B 16-23
	for(uint32_t i = ((led*24)+16); i<((led*24)+24); i++){
		if ((b>>(7-bitIndex)) & 0x01){
			buf[i] = WS2812_HI_VAL;
		} else {
			buf[i] = WS2812_LO_VAL;
		}
		bitIndex++;
	}
}

HAL_StatusTypeDef WS2812_Send(WS2812 *dev, uint8_t *buf){
	return HAL_TIM_PWM_Start_DMA(dev->tim, dev->tim_channel, buf, WS2812_BUF_LEN);
}

HAL_StatusTypeDef WS2812_StopDMA(WS2812 *dev){
	return HAL_TIM_PWM_Stop_DMA(dev->tim, dev->tim_channel);
}
