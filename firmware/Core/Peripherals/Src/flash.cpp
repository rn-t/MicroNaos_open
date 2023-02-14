#include "flash.hpp"
#include "main.h"
#include <string.h>

void Flash::erase(){
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseproperty;
    
    eraseproperty.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseproperty.Sector = FLASH_SECTOR_1;
    eraseproperty.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseproperty.NbSectors = 1;

    uint32_t error_sector;
    HAL_FLASHEx_Erase(&eraseproperty, &error_sector);
    
    HAL_FLASH_Lock();
}

void Flash::save(uint8_t (&wall_data)[16][16]){
    erase();

    HAL_FLASH_Unlock();

    uint32_t *wall_data_address = (uint32_t*) wall_data;

    const size_t write_count = sizeof(uint8_t) * 256 / sizeof(uint32_t);

    for (size_t i = 0; i < write_count; i++){
        HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD,
            (uint32_t)(_start_address) + sizeof(uint32_t) * i,
            wall_data_address[i]
        );
    }

    HAL_FLASH_Lock();
}

void Flash::load(){
    
    memcpy(wall, _start_address, sizeof(wall));
    
    // 消去後と仮定する(消去後であればすべて1)
    is_elased = 1;

    for (int i = 0; i < 16; i++){
        for (int j = 0; j < 16; j++){
            if(wall[i][j] == 0){
                // 0があれば消去前(消去後であればすべて1)
                is_elased = 0;
            }
        }
    }
}