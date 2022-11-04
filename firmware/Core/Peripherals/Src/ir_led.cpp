#include "ir_led.hpp"
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim13;

namespace ir_led{

    /**
     * @brief IR-LEDの動作を開始する。デフォルトでは発光しない。
     * 
     */
    void init(void){
        TIM2->CCR1 = 0;
		TIM13->CCR1 = 0;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    }

    /**
     * @brief IRLEDの発光状態を変化させる。
     * 
     * @param ir1_state 1->on 0->off 
     * @param ir2_state 1->on 0->off
     */
    void set_state(uint8_t ir1_state, uint8_t ir2_state){

        if (ir1_state == 1){
            TIM2->CCR1 = 40;
        }else{
            TIM2->CCR1 = 0;
        }
        
        if (ir2_state == 1){
            TIM13->CCR1 = 40;
        }else{
            TIM13->CCR1 = 0;
        }
    }
}