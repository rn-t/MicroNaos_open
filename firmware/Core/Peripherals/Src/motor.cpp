/*
 * motor.c
 *
 *  Created on: 2021/03/02
 *      Author: raku
 */

#include "main.h"
#include "motor.hpp"
#include "tools.hpp"

#include "arm_math.h"

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;



/**
 * @brief モーターの初期化関数
 * 
 */
void Motor::init(void){
	HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET); //enableをhighにして出力をon
	HAL_Delay(1);
	HAL_GPIO_WritePin(MOTOR_RESET_GPIO_Port, MOTOR_RESET_Pin, GPIO_PIN_RESET); //resetをlowにして電気角リセットから通常動作に
	HAL_Delay(1);
	HAL_GPIO_WritePin(MOTOR_DMODE0_GPIO_Port, MOTOR_DMODE0_Pin, GPIO_PIN_SET); //dmodeを8W1-2相励磁に
	HAL_GPIO_WritePin(MOTOR_DMODE1_GPIO_Port, MOTOR_DMODE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_DMODE2_GPIO_Port, MOTOR_DMODE2_Pin, GPIO_PIN_SET);

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 50);

	set_cw(1, 1);
	//速度を0にする	
	TIM3->CCR3 = 0;
	TIM8->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

void Motor::enable(uint8_t state){
	HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, static_cast<GPIO_PinState>(state)); //enableを設定
}

void Motor::set_vref(uint8_t v){
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, v);
}

/**
 * @brief モーターの回転方向をセットする
 * 
 * @param cw_l 1-正転, 0-逆転 
 * @param cw_r 1-正転, 0-逆転  
 */
void Motor::set_cw(uint8_t cw_l, uint8_t cw_r){
	cw_r = !cw_r;
	HAL_GPIO_WritePin(MOTOR1_CW_GPIO_Port, MOTOR1_CW_Pin, static_cast<GPIO_PinState>(cw_l));
	HAL_GPIO_WritePin(MOTOR2_CW_GPIO_Port, MOTOR2_CW_Pin, static_cast<GPIO_PinState>(cw_r));
}

/**
 * @brief モーターの速度を指定する関数
 * @param speed_l 左の移動速度 mm/sでfを付ける
 * @param speed_r 右の移動速度 mm/sでfを付ける
 */
void Motor::set_speed(float32_t speed_l, float32_t speed_r){
	uint8_t cw[2] = {1, 1};
	if(speed_l < 0){
		cw[0] = 0;
		speed_l = -1 * speed_l;
	}
	if(speed_r < 0){
		cw[1] = 0;
		speed_r = -1 * speed_r;	
	} 
	set_cw(cw[0], cw[1]);

	//周期をusで計算する
	const float32_t period_l = 1000000.0f * radius * PI / 3200.0f / speed_l;
	const float32_t period_r = 1000000.0f * radius * PI / 3200.0f / speed_r;

	const int32_t period_us_l = static_cast<int32_t>(period_l);
	const int32_t period_us_r = static_cast<int32_t>(period_r);
	
	//左モーターのTIMのARRとCCR3を設定する
	TIM3->ARR = period_us_l - 1;
	TIM3->CCR3 = period_us_l / 2;

	//左モーターのTIMのARRとCCR3を設定する
	TIM8->ARR = period_us_r - 1;
	TIM8->CCR1 = period_us_r / 2;		
}

void Motor::update_speed(void){
	float32_t speed_l_toset = speedstate_l.current;
	float32_t speed_r_toset = speedstate_r.current;
	float32_t accel_dt = accel * t_scale;

	//目標値と現在値がずれている場合、一定の加速度だけ値を変化させる
	if(abs(speedstate_l.current - speedstate_l.target) < accel_dt){
		speed_l_toset = speedstate_l.target;
	}else if (speedstate_l.current < speedstate_l.target){
		speed_l_toset += accel_dt;
	}else if (speedstate_l.current > speedstate_l.target){
		speed_l_toset -= accel_dt;
	}
	
	if(abs(speedstate_r.current - speedstate_r.target) < accel_dt){
		speed_r_toset = speedstate_r.target;
	}else if (speedstate_r.current < speedstate_r.target){
		speed_r_toset += accel_dt;
	}else if (speedstate_r.current > speedstate_r.target){
		speed_r_toset -= accel_dt;
	}
	
	speedstate_l.previous = speedstate_l.current;
	speedstate_r.previous = speedstate_r.current;
	speedstate_l.current = speed_l_toset;
	speedstate_r.current = speed_r_toset;

	set_speed(speed_l_toset, speed_r_toset);
}

void Motor::set_target(float32_t speed_l, float32_t speed_r){
	speedstate_l.target =  speed_l;
	speedstate_r.target =  speed_r;		
}

void Motor::set_target_deg(float32_t deg_speed){
	float32_t speed = deg_speed_to_speed(deg_speed);
	speedstate_l.target =  speed;
	speedstate_r.target =  -1 * speed;	
}

void Motor::set_turn_speed(float32_t deg_speed){
	float32_t speed = deg_speed_to_speed(deg_speed);
	set_speed(speed, -1 * speed);
}

void Motor::stop(void){
	TIM3->CCR3 = 0;
	TIM8->CCR1 = 0;	
}

float32_t Motor::deg_speed_to_speed(float32_t deg_speed){
	return (tire_width / 2.0f) * deg_speed * PI / 180.0f;
}

float32_t Motor::speed_to_degspeed(float32_t speed){
	return (speed * 2.0f * 180.0f) /( tire_width * PI);
}

void Motor::forward(float32_t length, uint8_t slow_down){
	if(length < 80.0f){
		forward_speed = 180.0f;
	}else{
		forward_speed = 270.0f;
	}
	state.delta_max = length * forward_gain;
	state.mode = state.forward;
	state.slow_down = slow_down;
	HAL_Delay(2);
	while (1){
		if(state.mode_lock == 0) break;
	}
}

void Motor::turn(float32_t deg){
	state.delta_max_deg = deg;
	state.mode = state.turn;
	HAL_Delay(2);
	while (1){
		if(state.mode_lock == 0) break;
	}
	HAL_Delay(2);
}

void Motor::kabeate0(void){
	forward(-70.0f);
	forward(40.5f);
}

void Motor::kabeate1(void){
	turn(-90.0f);
	kabeate0();
}

void Motor::kabeate2(void){
	turn(90.0f);
	kabeate0();
}

void Motor::kabeate_turn_l(void){
	turn(-90.0f);
	kabeate0();
	turn(-90.0f);
	kabeate0();
}
void Motor::kabeate_turn_r(void){
	turn(90.0f);
	kabeate0();
	turn(90.0f);
	kabeate0();
}