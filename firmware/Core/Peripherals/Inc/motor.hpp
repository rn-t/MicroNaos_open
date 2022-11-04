/*
 * motor.h
 *
 *  Created on: 2021/03/02
 *      Author: raku
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "arm_math.h"
/**
 * @brief 速度に関する情報を保存するクラス
 * 
 */
class SpeedState{
    public:
        volatile float32_t target = 0.0f;
        volatile float32_t current = 0.0f;
        volatile float32_t previous = 0.0f;
};

class Coord{
    public:
        float32_t x;
        float32_t y;
        float32_t z_angle;
};

class State{
    public:
        const uint8_t stop = 0;
        const uint8_t forward = 1;
        const uint8_t turn = 2;

        uint8_t mode = 0;
        uint8_t mode_previous = 0;
        uint8_t mode_lock = 0;

        volatile float32_t delta = 0.0f;
        volatile float32_t delta_max = 90.0f;
        
        volatile float32_t delta_deg = 0.0f;
        volatile float32_t delta_max_deg = 90.0f;
};

class Motor{
    public:
    //タイヤの大きさを設定
    	const float32_t tire_width = 82.0f; //mm
    	//いわゆる速度パラメーターを設定
	    float32_t forward_speed = 180.0f; //(mm/s)
	    float32_t turn_speed = 180.0f; //(deg/s)

        //加速度を設定
        const float32_t accel = 900.0f; //(mm/s^2)
        
        //時間スケールを設定
        const float32_t t_scale = 0.001f; //(s)
        SpeedState speedstate_l;
        SpeedState speedstate_r;
        Coord coord;
        State state;

        void init(void);
        void enable(uint8_t state);
        void set_vref(uint8_t);
        void set_cw(uint8_t cw_l, uint8_t cw_r);
        void set_speed(float32_t speed_l, float32_t speed_r);
        void update_speed(void);
        void set_target(float32_t speed_l, float32_t speed_r);
        void set_target_deg(float32_t deg_speed);
        void set_turn_speed(float32_t deg_speed);
        void stop(void);
        float32_t deg_speed_to_speed(float32_t deg_speed);
        float32_t speed_to_degspeed(float32_t speed);
        
        void forward(float32_t length);
        void turn(float32_t deg);
        void kabeate1(void);
        void kabeate2(void);
        void kabeate_inv(void);
};
#endif /* INC_MOTOR_HPP_ */
