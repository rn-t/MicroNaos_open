/*
 * spi.h
 *
 *  Created on: 2021/02/26
 *      Author: raku
 */

#ifndef INC_SPI_HPP_
#define INC_SPI_HPP_

#include "main.h"
#include "arm_math.h"

namespace spi{
    class Gyro{
        private:
            static const uint16_t buffer_size_ = 256;
            volatile float32_t buffer_[buffer_size_];
            volatile uint16_t buffer_index_;

            const float32_t gyro_scale_ = 2000.0f;
            const float32_t t_scale_ = 0.001f;

        public:
            void init(void);
            void who_am_i(void);
            void write(uint8_t address, uint8_t value);
            uint8_t read(uint8_t address);
            uint16_t read_2byte(uint8_t address1, uint8_t address2);

            int32_t read_accel_x(void);
            int32_t read_accel_y(void);
            int32_t read_accel_z(void);

            int32_t read_gyro_x(void);
            int32_t read_gyro_y(void);
            int32_t read_gyro_z(void);
            int32_t cast_from_2_complex(uint16_t input);
            void calibration_gyro_z(void);
            void update_z_angle(void);

            volatile uint8_t calibration_status = 0;
            volatile float32_t gyro_z_drift;
            volatile float32_t z_angle;
            volatile float32_t z_angle_previous;
    };
}
#endif /* INC_SPI_HPP_ */
