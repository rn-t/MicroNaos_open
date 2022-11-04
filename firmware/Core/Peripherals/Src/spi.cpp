/*
 * spi.c
 *
 *  Created on: 2021/02/26
 *      Author: raku
 */

#include "main.h"
#include "spi.hpp"

#include "arm_math.h"
#include "SEGGER_RTT.h"

#define SPI_GYRO_WHO_AM_I 0x75
#define SPI_GYRO_CONFIG 0x1A
#define SPI_GYRO_GYRO_CONFIG 0x1B
#define SPI_GYRO_ACCEL_CONFIG 0x1C
#define SPI_GYRO_POWER_MANAGEMENT_1 0x6B

#define SPI_GYRO_ACCEL_XOUT_H 0x3B
#define SPI_GYRO_ACCEL_XOUT_L 0x3C
#define SPI_GYRO_ACCEL_YOUT_H 0x3D
#define SPI_GYRO_ACCEL_YOUT_L 0x3E
#define SPI_GYRO_ACCEL_ZOUT_H 0x3F
#define SPI_GYRO_ACCEL_ZOUT_L 0x40

#define SPI_GYRO_GYRO_XOUT_H 0x43
#define SPI_GYRO_GYRO_XOUT_L 0x44
#define SPI_GYRO_GYRO_YOUT_H 0x45
#define SPI_GYRO_GYRO_YOUT_L 0x46
#define SPI_GYRO_GYRO_ZOUT_H 0x47
#define SPI_GYRO_GYRO_ZOUT_L 0x48


extern SPI_HandleTypeDef hspi3;
namespace spi{

	void Gyro::init(void){
		HAL_Delay(1);
		Gyro::write(SPI_GYRO_POWER_MANAGEMENT_1, 0x80);
		HAL_Delay(1);
		Gyro::write(SPI_GYRO_POWER_MANAGEMENT_1, 0x00);
		HAL_Delay(1);
		Gyro::write(SPI_GYRO_CONFIG, 0x00);
		HAL_Delay(1);
		Gyro::write(SPI_GYRO_GYRO_CONFIG, 0b00011000); // Gyro 250dps -> 2000dps
		HAL_Delay(1);
		Gyro::write(SPI_GYRO_ACCEL_CONFIG, 0b00011000); // Accel 2g -> 16g
		HAL_Delay(1);
	}

	void Gyro::who_am_i(void){
		uint8_t report = Gyro::read(SPI_GYRO_WHO_AM_I);
		SEGGER_RTT_printf(0, "WHO_AM_I = %d (18 is OK)\n", report);
	}

	void Gyro::write(uint8_t address, uint8_t value){
		uint8_t transmit[2] = {address, value};
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //CSピン立ち下げ
		HAL_SPI_Transmit(&hspi3, &transmit[0], 2, 100);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //CSピン立ち上げ
	}

	uint8_t Gyro::read(uint8_t address){
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //CSピン立ち下げ
		uint8_t transmit[2];
		transmit[0] = address | 0x80;
		uint8_t receive[2];
		HAL_SPI_TransmitReceive(&hspi3, &transmit[0], &receive[0], 2, 100);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //CSピン立ち上げ
		return receive[1];
	}

	uint16_t Gyro::read_2byte(uint8_t address1, uint8_t address2){
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //CSピン立ち下げ
		uint8_t transmit[3];
		transmit[0] = address1 | 0x80;
		transmit[1] = address2 | 0x80;
		uint8_t receive[3];
		HAL_SPI_TransmitReceive(&hspi3, &transmit[0], &receive[0], 3, 100);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //CSピン立ち上げ
		uint16_t out = receive[1];
		out = out << 8;
		out += receive[2];
		return out;
	}

	int32_t Gyro::read_accel_x(void){
		return cast_from_2_complex(Gyro::read_2byte(SPI_GYRO_ACCEL_XOUT_H, SPI_GYRO_ACCEL_XOUT_L));
	}

	int32_t Gyro::read_accel_y(void){
		return cast_from_2_complex(Gyro::read_2byte(SPI_GYRO_ACCEL_YOUT_H, SPI_GYRO_ACCEL_YOUT_L));
	}

	int32_t Gyro::read_accel_z(void){
		return cast_from_2_complex(Gyro::read_2byte(SPI_GYRO_ACCEL_ZOUT_H, SPI_GYRO_ACCEL_ZOUT_L));
	}

	int32_t Gyro::read_gyro_x(void){
		return cast_from_2_complex(Gyro::read_2byte(SPI_GYRO_GYRO_XOUT_H, SPI_GYRO_GYRO_XOUT_L));
	}

	int32_t Gyro::read_gyro_y(void){
		return cast_from_2_complex(Gyro::read_2byte(SPI_GYRO_GYRO_YOUT_H, SPI_GYRO_GYRO_YOUT_L));
	}

	int32_t Gyro::read_gyro_z(void){
		return cast_from_2_complex(Gyro::read_2byte(SPI_GYRO_GYRO_ZOUT_H, SPI_GYRO_GYRO_ZOUT_L));
	}
	int32_t Gyro::cast_from_2_complex(uint16_t input){
		int32_t out = static_cast<int32_t>(input); 
		//16bitの2の補数から強引に変換する
		if ((out & 0x00008000) == 0x00008000) out = out - 65536;
		return out;
	}
	
	/**
	 * @brief ジャイロのキャリブレーションを行う。buffer_size_ + 1回呼び出すと終わる。
	 * 
	 */
	void Gyro::calibration_gyro_z(void){
		if (buffer_index_ < buffer_size_){
			buffer_[buffer_index_] = static_cast<float32_t>(read_gyro_z());
    		buffer_index_++;
		}else if(buffer_index_ == buffer_size_){
            float32_t buffer_copy[buffer_size_];
			for (uint16_t i = 0; i < buffer_size_; i++){
				buffer_copy[i] = buffer_[i];
			}
			float32_t mu_y;
			arm_mean_f32(buffer_copy, buffer_size_, &mu_y);

			gyro_z_drift = mu_y;

			buffer_index_ = 0;
			SEGGER_RTT_printf(0, "gyro calibration end\n");
			calibration_status = 1;
		}else{
			SEGGER_RTT_printf(0, "ERROR!!! gyro_buffer_index_ > gyro_buffer_size_\n");
		}
	}
	void Gyro::update_z_angle(void){
		float32_t dz = static_cast<float32_t>(read_gyro_z()) - gyro_z_drift;
		z_angle_previous = z_angle;
		z_angle += static_cast<float32_t>(dz) * gyro_scale_ * t_scale_ / 32768.0f;	
	}
}