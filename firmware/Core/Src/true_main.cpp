
#include "main.h"
#include "usb_device.h"
#include "usbd_hid.h"

#include "true_main.hpp"
#include "led.hpp"
#include "spi.hpp"
#include "motor.hpp"
#include "ir_led.hpp"
#include "ir_sensor.hpp"
#include "maze.hpp"
#include "SEGGER_RTT.h"
#include "arm_math.h"
#include "mazelibrary.hpp"

#include <stdio.h>


extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim11;


//各壁センサのインスタンスを作成
namespace ir{
	static Ir_sensor side_right(0, 350);
	static Ir_sensor front_right(1, 1000);
	static Ir_sensor front_left(2, 1000);
	static Ir_sensor side_left(3, 350);
}

//ジャイロセンサのインスタンスを作成
static spi::Gyro gyro;

static Motor motor;
namespace sw{
	static volatile uint8_t l = 0;
	static volatile uint8_t r = 0;
}

static volatile float32_t gyro_z_e_i = 0.0f;

static Maze maze;
static Mouse mouse;

static uint8_t usbmouse_enable = 0;

struct USBMouse {
	uint8_t button;
	int8_t x;
	int8_t y;
	int8_t w;
};

static struct USBMouse usbmouse;

extern USBD_HandleTypeDef hUsbDeviceFS;

void failsafe(void){
	motor.stop();
	motor.enable(0);
	__disable_irq();
	while(1){
		led::set(1, 0, 0);
		HAL_Delay(500);
		led::set(0, 0, 0);
		HAL_Delay(500);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	//TIM6オーバーフロー割り込みは160kHz周期で呼び出される
	//呼び出されたときにデータを格納する。
	if(htim->Instance == htim6.Instance){
		ir::front_right.update_data();
		ir::side_right.update_data();
		ir::front_left.update_data();
		ir::side_left.update_data();
	}

	//TIM7オーバーフロー割り込みは1kHz周期で呼び出される
	if(htim->Instance == htim7.Instance){
		if (gyro.calibration_status == 0){
			gyro.calibration_gyro_z();
		}else{
			gyro.update_z_angle();
		}
		
		/* //宴会芸用のプログラム
		if(gyro.calibration_status == 1){
			float32_t temp_deg_speed;
			const float32_t Kp = 10.0f;
			const float32_t Ki = 0.01f;
			const float32_t Kd = 1.0f;
			
			float32_t gyro_z_e = gyro.z_angle;
			
			//数値積分
			gyro_z_e_i += gyro.z_angle;
			if(gyro_z_e_i > 10.0f) gyro_z_e_i = 10.0f;
			if(gyro_z_e_i < -10.0f) gyro_z_e_i = -10.0f;

			//数値微分
			float32_t gyro_z_e_d = gyro.z_angle - gyro.z_angle_previous;

			temp_deg_speed = -1 * (Kp * gyro_z_e + Ki * gyro_z_e_i + Kd * gyro_z_e_d);
			motor.set_target_deg(temp_deg_speed);
		}
		*/

		/*
		 * motor.state.forwardではmotor.state.deltamax分だけ直進する。
		 * 
		 */
		if(motor.state.mode == motor.state.forward){
			// モード変更時はdeltaをリセットしておく。
			if(motor.state.mode_previous != motor.state.forward){
				motor.state.delta = 0.0f;
				motor.state.mode_lock = 1;

				float32_t target_speed;

				//速度ターゲットを直進スピードに設定する。(符号も合わせる)
				if(motor.state.delta_max > 0){
					target_speed = motor.forward_speed;
				}else{	
					target_speed = -1.0f * motor.forward_speed;	
				}
				//拘束条件として右と左のスピードは同じになる。
				motor.set_target(target_speed, target_speed);
				motor.state.mode_previous = motor.state.forward;
			}else{
				motor.state.delta += motor.speedstate_l.current * motor.t_scale;

				if(abs(motor.state.delta_max - motor.state.delta) 
					< motor.forward_speed * (motor.forward_speed / motor.accel) / 2.0f){
				
					//速度ターゲットを0に設定する
					//拘束条件として右と左のスピードは同じになる。
					motor.set_target(0.0f, 0.0f);
				}
				if(abs(motor.speedstate_l.current) < (motor.accel * motor.t_scale)){
					motor.state.mode = motor.state.stop;
					motor.state.mode_previous = motor.state.stop;
					motor.state.mode_lock = 0;
				}

			}
		}else if(motor.state.mode == motor.state.turn){
			// モード変更時はdeltaをリセットしておく。
			if(motor.state.mode_previous != motor.state.turn){
				motor.state.delta_deg = 0.0f;
				motor.state.mode_lock = 1;

				float32_t target_speed_deg;

				//速度ターゲットを直進スピードに設定する。(符号も合わせる)
				if(motor.state.delta_max_deg > 0){
					target_speed_deg = motor.turn_speed;
				}else{	
					target_speed_deg = -1.0f * motor.turn_speed;	
				}
				//回転方向の速度を設定
				motor.set_target_deg(target_speed_deg);
				motor.state.mode_previous = motor.state.turn;
			}else{
				motor.state.delta_deg += motor.speed_to_degspeed(motor.speedstate_l.current) * motor.t_scale;

				if(abs(motor.state.delta_max_deg - motor.state.delta_deg) 
					< motor.turn_speed * (motor.turn_speed / motor.speed_to_degspeed(motor.accel)) / 2.0f){
				
					//回転方向の速度を0に設定
					motor.set_target_deg(0.0f);
				}
				if(abs(motor.speedstate_l.current) < (motor.accel * motor.t_scale)){
					motor.state.mode = motor.state.stop;
					motor.state.mode_previous = motor.state.stop;
					motor.state.mode_lock = 0;
				}

			}
		}
		motor.update_speed();
		
	}
	//TIM7オーバーフロー割り込みは100Hz周期で呼び出される
	if(htim->Instance == htim11.Instance){
		if(motor.speedstate_l.target > 500 || motor.speedstate_r.target > 500){
			failsafe();
		}
		//スイッチの状態を確認
		if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET){
			sw::r = 1;
		}else{
			sw::r = 0;
		}if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET){
			sw::l = 1;
		}else{
			sw::l = 0;
		}
		if(usbmouse_enable == 1){

			usbmouse.button = 0;
			usbmouse.x = 0;
			usbmouse.y = 0;

			if (sw::r == 1) usbmouse.button = usbmouse.button | 0b00000010;
			if (sw::l == 1) usbmouse.button = usbmouse.button | 0b00000001;

			mouse.x = -1 * (gyro.read_accel_x() / 16) / 5;
			mouse.y = (gyro.read_accel_y() / 16) / 5;
			
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &mouse, sizeof(struct Mouse));
			SEGGER_RTT_printf(0, "%d, %d, %d, %d\n", ir::side_left.intensity, ir::front_left.intensity, ir::front_right.intensity, ir::side_right.intensity);	
		}

		//バッテリーの状態を確認
		HAL_ADC_Start(&hadc2);
		if(HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK){
			//約10.5V(3260)を下回った場合は警告する<-11.5Vで引っかかったので3100に変更
			if(HAL_ADC_GetValue(&hadc2) < 3100){
				failsafe();
			}
		}
		HAL_ADC_Stop(&hadc2);
		/*
		SEGGER_RTT_printf(0, "\n");
		SEGGER_RTT_printf(0, "+%d+\n", maze.node[0][1].wall.up);
		SEGGER_RTT_printf(0, "%d %d\n", maze.node[0][1].wall.left, maze.node[0][1].wall.right);
		SEGGER_RTT_printf(0, "+%d+\n", maze.node[0][0].wall.up);
		SEGGER_RTT_printf(0, "%d %d\n", maze.node[0][0].wall.left, maze.node[0][0].wall.right);
		SEGGER_RTT_printf(0, "+%d+\n", maze.node[0][0].wall.down);
		*/
	}
}

extern DAC_HandleTypeDef hdac;

/**
 * @brief 真のmain関数
 * 
 */
void true_main(void){
	HAL_Delay(100);
	//各部分の初期化
    SEGGER_RTT_Init();
	led::init();
	ir_led::init();
	motor.init();
	gyro.init();
	ir_led::set_state(1,1);
	Ir_sensor::init();

	//Who_am_iを読みジャイロをチェックする。(RTTに出力)
	gyro.who_am_i();

	//割り込み用のTIMを起動
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim11);

	uint16_t start_count = 0;
	if(sw::r == 1){
		led::set(0, 0, 1);
		
		usbmouse.button = 0;
		
		usbmouse.x = 0;
		usbmouse.y = 0;
		usbmouse.w = 0;
		usbmouse_enable = 1;

		motor.enable(0);

		while(1){

		}
	}
	while(1){
		
		//前センサを適度に塞ぐと起動する
		while (1){
			if(ir::front_right.wall_detect()){
				start_count++;
			}else{
				if(start_count > 0) start_count--;
			}
			if(start_count > 500) break;
			HAL_Delay(1);
		}
		
		//動作開始	
		led::set(0, 1, 0);
		HAL_Delay(2000);
		led::set(0, 0, 0);
		
		mouse.x = 0;
		mouse.y = 0;
		mouse.direction = Direction::up;
		
		motor.kabeate1();
		if(ir::front_left.wall_detect() || ir::front_right.wall_detect()){
			maze.wall_update(mouse.x, mouse.y, 0);
		}
		motor.kabeate2();
		if(ir::front_left.wall_detect() || ir::front_right.wall_detect()){
			maze.wall_update(mouse.x, mouse.y, 0);
		}

	}
}