/*
 * led.h
 *
 *  Created on: Feb 25, 2021
 *      Author: raku
 */

#ifndef INC_LED_HPP_
#define INC_LED_HPP_

#include "main.h"
namespace led{
    void init();
    void set(uint8_t r, uint8_t g, uint8_t b);
}

#endif /* INC_LED_HPP_ */

