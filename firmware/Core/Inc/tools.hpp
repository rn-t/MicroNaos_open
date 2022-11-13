#ifndef INC_TOOLS_HPP_
#define INC_TOOLS_HPP_

#include "main.h"
#include "mazelibrary.hpp"

namespace tools{
    uint8_t get_rotated_wall(uint8_t direction, uint8_t wall);
    int16_t direction_to_deg(uint8_t direction);
    int16_t deg_sub(int16_t a, int16_t b);
}

#endif