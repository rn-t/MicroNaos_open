#ifndef INC_FLASH_HPP_
#define INC_FLASH_HPP_

#include "main.h"

extern char _flash_data_start;

class Flash{
    private:
    const char* _start_address = &_flash_data_start;
    public:
    uint8_t is_elased = 1; 
    uint8_t wall[16][16] __attribute__ ((aligned(4)));

    void erase();
    void save(uint8_t (&wall_data)[16][16]);
    void load();
};

#endif