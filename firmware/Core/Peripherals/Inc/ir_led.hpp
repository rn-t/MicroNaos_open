#ifndef INC_IR_LED_HPP_
#define INC_IR_LED_HPP_

#include "main.h"

namespace ir_led{
    void init(void);
    void set_state(uint8_t ir1_state, uint8_t ir2_state);
}

#endif