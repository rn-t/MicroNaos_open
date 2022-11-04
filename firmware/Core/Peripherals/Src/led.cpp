/*
 * led.c
 *
 *  Created on: Feb 25, 2021
 *      Author: raku
 */
#include "led.hpp"
#include "main.h"
#include "tools.hpp"


namespace led{
	/**
	 * @brief LEDの初期化関数
	 */
	void init(){
		//とりあえず点灯しないようにしている。
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, static_cast<GPIO_PinState>(0));
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, static_cast<GPIO_PinState>(0));
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, static_cast<GPIO_PinState>(0));
	}

	/**
	 * @brief フルカラーLEDの色を変更する関数
	 * @param r 赤点灯(0 or 1)
	 * @param g 緑点灯(0 or 1)
	 * @param b 青点灯(0 or 1)
	 * 
	 */
	void set(uint8_t r, uint8_t g, uint8_t b) {
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, static_cast<GPIO_PinState>(r));
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, static_cast<GPIO_PinState>(g));
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, static_cast<GPIO_PinState>(b));
	}
}
