#ifndef INC_IR_SENSOR_HPP_
#define INC_IR_SENSOR_HPP_

#include "main.h"

class Ir_sensor{
    private:
        //data[4]での対応するindex 
        const uint8_t data_index_;
        const uint16_t border_;
        
        static const uint8_t buffer_size_ = 64;
        volatile uint16_t buffer_[buffer_size_];
        volatile uint8_t buffer_index_;
        
        

    public:
        
        //DMAで得られるデータは配列になるので、センサから得られるデータに対応する配列のインデックスで初期化
        explicit Ir_sensor(uint8_t data_index, uint16_t border): data_index_(data_index), border_(border){}
        static volatile uint16_t data[4];
        volatile uint16_t intensity;

	    static void init(void);
        void update_data(void);
        uint8_t wall_detect(void);
};
#endif