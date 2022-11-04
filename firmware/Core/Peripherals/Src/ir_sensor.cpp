#include "main.h"
#include "ir_sensor.hpp"
#include "arm_math.h"
#include "SEGGER_RTT.h"

extern ADC_HandleTypeDef hadc3;

volatile uint16_t Ir_sensor::data[4] = {0};

/**
 * @brief センサを初期化し、ADCの値をDMAでdata[4]に取れるようにする
 * 
 */
void Ir_sensor::init(void){
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)data, 4);
}

/**
 * @brief data[data_index_]の値をbuffer_に格納し、buffer_index_を更新する。
 * 
 */
void Ir_sensor::update_data(void){
    if(buffer_index_ < buffer_size_){
        buffer_[buffer_index_] = data[data_index_];
        buffer_index_++;
    }else if(buffer_index_ == buffer_size_){
        uint16_t max_data = 0;
        for (uint8_t i = 0; i < buffer_size_; i++){
            if(buffer_[i] > max_data) max_data = buffer_[i];
        }
        intensity = max_data - 2000;
        buffer_index_ = 0;
    }
}

uint8_t Ir_sensor::wall_detect(void){
    if (intensity > border_){
        return 1;
    }else{
        return 0;
    }
}
