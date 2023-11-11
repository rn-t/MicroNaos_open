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
#include "tools.hpp"
#include "flash.hpp"

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
	static Ir_sensor front_right(1, 300);
	static Ir_sensor front_left(2, 300);
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

static std::vector<std::vector<uint8_t>> start_coord = {{0, 0}};
// static std::vector<std::vector<uint8_t>> goal_coord = {{1, 0}, {1, 1}, {2, 0}, {2, 1}};
static std::vector<std::vector<uint8_t>> goal_coord = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
// static std::vector<std::vector<uint8_t>> goal_coord = {{8, 3}};

static Maze maze(start_coord, goal_coord);
static Mouse mouse;
static AdachiMethod method(&maze, &mouse);

static Flash flash;

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

				float32_t target_speed_l, target_speed_r;

				//速度ターゲットを直進スピードに設定する。(符号も合わせる)
				if(motor.state.delta_max > 0){
					target_speed_l = motor.forward_speed;
				}else{	
					target_speed_l = -1.0f * motor.forward_speed;	
				}

				target_speed_r = target_speed_l;

				
				//拘束条件として右と左のスピードは同じになる。
				motor.set_target(target_speed_l, target_speed_r);
				motor.state.mode_previous = motor.state.forward;
			}else{
				motor.state.delta += (motor.speedstate_l.current + motor.speedstate_r.current) / 2.0f * motor.t_scale;
				
				

				if(motor.state.slow_down == 1){
					//減速を行う場合(デフォルト)
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
				}else{
					float32_t target_speed_l, target_speed_r;

					//速度ターゲットを直進スピードに設定する。(符号も合わせる)
					if(motor.state.delta_max > 0){
						target_speed_l = motor.forward_speed;
					}else{	
						target_speed_l = -1.0f * motor.forward_speed;	
					}

					target_speed_r = target_speed_l;
					if(ir::side_left.intensity > 800){
						float32_t diff = static_cast<float32_t>(ir::side_left.intensity - 800);
						target_speed_r += diff * 0.1f;
					}else if(ir::side_right.intensity > 800){
						float32_t diff = static_cast<float32_t>(ir::side_right.intensity - 800);
						target_speed_l += diff * 0.1f;

					}
					motor.set_target(target_speed_l, target_speed_r);
					if(abs(motor.state.delta_max - motor.state.delta) < (motor.forward_speed * motor.t_scale)){
						//形式的にstopにしておく(あまり直観的ではない)
						motor.state.mode = motor.state.stop;
						motor.state.mode_previous = motor.state.stop;
						motor.state.mode_lock = 0;
					}
				}
			}
		}else if(motor.state.mode == motor.state.turn){
			// モード変更時はdeltaをリセットしておく。
			if(motor.state.mode_previous != motor.state.turn){
				motor.state.deg_std = gyro.z_angle;
				gyro_z_e_i = 0.0f;
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
				//差分を取得
				motor.state.delta_deg = motor.state.delta_max_deg - (gyro.z_angle - motor.state.deg_std);

				float32_t temp_deg_speed;
				/*
				const float32_t Kp = 10.0f;
				const float32_t Ki = 0.01f;
				const float32_t Kd = 1.0f;
				*/
				
				const float32_t Kp = 8.0f;
				const float32_t Ki = 0.05f;
				const float32_t Kd = 2.0f;

				float32_t gyro_z_e = motor.state.delta_deg;
				
				//数値積分
				gyro_z_e_i += gyro_z_e;
				if(gyro_z_e_i > 10.0f) gyro_z_e_i = 10.0f;
				if(gyro_z_e_i < -10.0f) gyro_z_e_i = -10.0f;

				//数値微分
				float32_t gyro_z_e_d = gyro.z_angle - gyro.z_angle_previous;

				temp_deg_speed = Kp * gyro_z_e + Ki * gyro_z_e_i + Kd * gyro_z_e_d;
				
				if(temp_deg_speed > motor.turn_speed) temp_deg_speed = motor.turn_speed;
				if(temp_deg_speed < -1 * motor.turn_speed) temp_deg_speed = -1 * motor.turn_speed;

				motor.set_target_deg(temp_deg_speed);

				if(abs(motor.state.delta_deg) < 0.1f){
					motor.set_target_deg(0.0f);
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

			usbmouse.x = -1 * (gyro.read_accel_x() / 16) / 5;
			usbmouse.y = -1 * (gyro.read_accel_y() / 16) / 5;
			
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &usbmouse, sizeof(struct USBMouse));
			SEGGER_RTT_printf(0, "%d, %d, %d, %d\n", ir::side_left.intensity, ir::front_left.intensity, ir::front_right.intensity, ir::side_right.intensity);	
		}

		//バッテリーの状態を確認
		HAL_ADC_Start(&hadc2);
		if(HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK){
			//約10.5V(3260)を下回った場合は警告する<-11.5Vで引っかかったので3100に変更
			if(HAL_ADC_GetValue(&hadc2) < 3000){
				failsafe();
			}
		}
		HAL_ADC_Stop(&hadc2);
			
		// 角度を送信
		//char s[8];
		//snprintf(s, sizeof(s), "%f", gyro.z_angle);
		//SEGGER_RTT_printf(0, "gyro_z = %s\n", s);	
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
	
	//Who_am_iを読みジャイロをチェックする。(RTTに出力)
	gyro.who_am_i();
	
	ir_led::set_state(1,1);
	Ir_sensor::init();


	//割り込み用のTIMを起動
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim11);

	uint8_t mode = 0;
	uint16_t start_count = 0;
	if(sw::r == 1){
		led::set(0, 0, 1);

		flash.erase();
		
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
			if(sw::l == 1){
				if(mode == 0){
					mode = 1;
					led::set(0, 1, 1);
				}else if(mode == 1){
					mode = 2;
					led::set(1, 0, 1);
				}else{
					mode = 0;
					led::set(0, 0, 0);
				}
				HAL_Delay(200);
			}
			if(ir::front_right.intensity > 1000){
				start_count++;
			}else{
				if(start_count > 0) start_count--;
			}
			if(start_count > 500) break;
			HAL_Delay(1);
		}
		
		//動作開始	
		led::set(0, 1, 0);
		
		gyro.calibration_status = 0;
		while(gyro.calibration_status == 0);
		gyro.z_angle = 0.0f;
		
		HAL_Delay(2000);
		led::set(0, 0, 0);
		if (mode == 0){
			for (int i = 0; i < 16; i++){
				for (int j = 0; j < 16; j++){
					maze.wall[i][j] = 0;
				}
			}
		}else{
			flash.load();
			if(flash.is_elased == 0){
				for (int i = 0; i < 16; i++){
					for (int j = 0; j < 16; j++){
						maze.wall[i][j] = flash.wall[i][j];
					}
				}
			}
		}
		
		mouse.x = 0;
		mouse.y = 0;
		mouse.direction = Direction::up;

		/*デバッグ向け直進処理*/
		/*		
		motor.forward(810, 0);
		motor.forward(90, 1);
		while(1){}
		*/
		
		/*デバッグ向け回転処理*/
		/*
		for (uint8_t i = 0; i < 4; i++){
			motor.turn(90.0f);
			HAL_Delay(10);
		}
		while(1){}
		*/
		/*---スタート時の処理---*/
		
		//1回目の壁当ては左壁でやるので右を向く
		motor.kabeate1();
		mouse.direction = Direction::right;
		
		if(ir::front_left.intensity > 1000 && ir::front_right.intensity > 1000){
			uint8_t temp_wall = Direction::up;
			temp_wall = tools::get_rotated_wall(mouse.direction, temp_wall);
			maze.wall_update(mouse.x, mouse.y, temp_wall);
		}
		
		//2回目の壁当ては下壁でやるので上を向く
		motor.kabeate2();
		mouse.direction = Direction::up;
		if(ir::front_left.intensity > 1000 && ir::front_right.intensity > 1000){
			uint8_t temp_wall = Direction::up;
			temp_wall = tools::get_rotated_wall(mouse.direction, temp_wall);
			maze.wall_update(mouse.x, mouse.y, temp_wall);
		}

		
		gyro.z_angle = 0.0f;

		motor.forward(90.0f, 0);
		mouse.y = 1;
		
		/*---スタート時の処理終了。(0,1からスタート)---*/
		//実際は(0,0.5)あたりにいる。

		for(uint8_t step = 0; step < 4; step++){
			if(mode != 0){
				step = 3;
			}
        	if(step != 2){
        	    method.set_goals(maze.goal);
        	}else if (step == 2){
            	method.set_goals(maze.start);
        	}
			switch(step){
				case 1:
					led::set(1, 1, 0);
					break;
				case 2:
					led::set(1, 0, 1);
					break;
				case 3:
					led::set(0, 1, 1);
					break;
			}
			
			if(step == 3){
				//最短走行の時に速度を決定する。
				switch(mode){
					case 0:
						motor.forward_scale = 1.25f;
						break;
					case 1:
						motor.forward_scale = 1.0f;
						break;
					case 2:
						motor.forward_scale = 1.25f;
						break;
				}
			}else{
				motor.forward_scale = 1.0f;
			}

			while(1){	
			//迷路からの壁情報の読み込み
				uint8_t temp_wall = 0;
				if(ir::front_left.wall_detect() && ir::front_right.wall_detect()){
					temp_wall += Direction::up;
				}
				if(ir::side_left.wall_detect()){
					temp_wall += Direction::left;
				}
				if(ir::side_right.wall_detect()){
					temp_wall += Direction::right;
				}

				/*ジャイロでの補正*/
				/*
				float32_t gyro_turn_deg = -1 * (gyro.z_angle 
					- static_cast<float32_t>(tools::direction_to_deg(mouse.direction)));
				motor.turn(tools::deg_normalize(gyro_turn_deg));
				*/

				uint8_t rotated_wall = tools::get_rotated_wall(mouse.direction, temp_wall);
				
				maze.wall_update(mouse.x, mouse.y, rotated_wall);
				std::vector<uint8_t> now{mouse.x, mouse.y};
				method.set_start(now);


				/*
				else if(step == 2){
					led::set(1, 0, 1);
					method.set_goals(method.get_unknown_in_shortest());
					if(method.goals.empty()) break;

				}
				*/

				if(step == 3){
					method.set_wall_at_unknown();
				}
				
				//コストの再計算
				method.cost_refresh();
				//いらない経路の削除
				method.delete_bad_route();

				if(step == 1){
					motor.forward(90.0f);
					if((rotated_wall & Direction::up) != Direction::up){
						//上に壁がない場合回転しない						

					}else if((rotated_wall & Direction::left) != Direction::left){
						//そのうえで左に壁がない場合左に進む
						mouse.turn_inv90();
						motor.turn(-90.0f);
						
					}else if((rotated_wall & Direction::right) != Direction::right){
						//そのうえで右に壁がない場合右に進む
						mouse.turn_90();
						motor.turn(90.0f);
					}else{
						mouse.turn_180();
						motor.kabeate_turn_r();
						gyro.z_angle = static_cast<float32_t>(tools::direction_to_deg(mouse.direction));
					}
					motor.forward(90.0f);
					mouse.move_forward();
					flash.save(maze.wall);
					led::set(0, 0, 1);
					HAL_Delay(500);
					break;
					
				}
				
				if(step == 0 && method.goal_check()){
					break;
				}
				if(step == 3 && method.goal_check()){
					motor.forward(90.0f);
					while(1){
						led::set(1, 1, 1);
						HAL_Delay(500);
						led::set(0, 0, 0);
						HAL_Delay(500);
					}
				}
				
				int16_t mouse_deg = tools::direction_to_deg(mouse.direction);
				int16_t maze_deg = tools::direction_to_deg(maze.route[mouse.x][mouse.y]);

				int16_t turn_deg = tools::deg_sub(mouse_deg, maze_deg);
				
				if(step == 2 && method.goal_check()){
					motor.forward(90.0f);
					turn_deg = tools::deg_sub(mouse_deg, tools::direction_to_deg(Direction::up));
					switch(turn_deg){
						case 0:
							break;
						case 90:
							mouse.turn_inv90();
							motor.turn(-90.0f);
							break;
						case -90:
							mouse.turn_90();
							motor.turn(90.0f);
							break;
						case 180:
							mouse.turn_180();
							motor.turn(180.0f);
							break;
					}

					motor.kabeate1();
					motor.kabeate2();
					
					flash.save(maze.wall);
					led::set(0, 0, 1);
					HAL_Delay(500);

					led::set(1, 0, 1);
					HAL_Delay(2000);
					
					gyro.z_angle = static_cast<float32_t>(tools::direction_to_deg(mouse.direction));

					motor.forward(90.0f, 0);
					mouse.y = 1;
					break;
				}

				if(step < 3){
					
					const uint16_t side_wall_near = 1200;
					float32_t current_ideal_angle = static_cast<float32_t>(tools::direction_to_deg(mouse.direction));
					switch(turn_deg){
						case 0:

							if(ir::side_left.intensity > side_wall_near){
								motor.forward(90.0f);
								motor.turn(90.0f);
								motor.kabeate0();
								motor.turn(-90.0f);
								motor.forward(90.0f, 0);

							}else if(ir::side_right.intensity > side_wall_near){
								motor.forward(90.0f);
								motor.turn(-90.0f);
								motor.kabeate0();
								motor.turn(90.0f);
								motor.forward(90.0f, 0);

							}else{
								const float32_t forward_gain = 0.98;
								motor.forward(180.0f * forward_gain, 0);
							}
							break;

						case 90:
							mouse.turn_inv90();
							motor.forward(90.0f);

							if(abs(tools::deg_sub(gyro.z_angle, current_ideal_angle)) < 5.0f ){
								//5°以上ずれている場合は壁当てする。
								motor.turn(-90.0f);
							}else{
								if(temp_wall == (Direction::left + Direction::up)){
									motor.turn(180.0f);
									motor.kabeate0();
									motor.turn(90.0f);
									motor.kabeate0();
									gyro.z_angle = current_ideal_angle;
								}else if(temp_wall == (Direction::up)){
									motor.turn(180.0f);
									motor.kabeate0();
									motor.turn(90.0f);
									gyro.z_angle = current_ideal_angle;
								}else if(temp_wall == (Direction::left)){
									motor.turn(-90.0f);
									motor.kabeate0();
									gyro.z_angle = current_ideal_angle;
								}

							}
							
							motor.forward(90.0f, 0);
							break;

						case -90:
							mouse.turn_90();
							motor.forward(90.0f);

							if(abs(tools::deg_sub(gyro.z_angle, current_ideal_angle)) < 5.0f ){
								//5°以上ずれている場合は壁当てする。
								motor.turn(90.0f);
							}else{
								if(temp_wall == (Direction::right + Direction::up)){
									motor.turn(180.0f);
									motor.kabeate0();
									motor.turn(-90.0f);
									motor.kabeate0();
									gyro.z_angle = current_ideal_angle;
								}else if(temp_wall == (Direction::up)){
									motor.turn(180.0f);
									motor.kabeate0();
									motor.turn(-90.0f);
									gyro.z_angle = current_ideal_angle;
								}else if(temp_wall == (Direction::right)){
									motor.turn(90.0f);
									motor.kabeate0();
									gyro.z_angle = current_ideal_angle;
								}
							}

							motor.forward(90.0f, 0);
							break;

						case 180:
							mouse.turn_180();
							motor.forward(90.0f);
							
							if(temp_wall == (Direction::left + Direction::right + Direction::up)
							|| temp_wall == (Direction::right + Direction::up)){
								motor.kabeate_turn_r();
								gyro.z_angle = static_cast<float32_t>(tools::direction_to_deg(mouse.direction));
							}else if(temp_wall == (Direction::left + Direction::up)){
								motor.kabeate_turn_l();
								gyro.z_angle = static_cast<float32_t>(tools::direction_to_deg(mouse.direction));
							}else if(temp_wall == Direction::left){
								motor.turn(-90.0f);
								motor.kabeate0();
								motor.turn(-90.0f);
							}else if(temp_wall == Direction::right){
								motor.turn(90.0f);
								motor.kabeate0();
								motor.turn(90.0f);
							}else{
								motor.turn(180.0f);
							}

							motor.forward(90.0f, 0);
							break;
					}
				}else{
					switch(turn_deg){
						case 0:
							motor.forward(180.0f, 0);
							break;
						case 90:
							mouse.turn_inv90();
							motor.forward(90.0f);
							motor.turn(-90.0f);
							motor.forward(90.0f, 0);
							break;
						case -90:
							mouse.turn_90();
							motor.forward(90.0f);
							motor.turn(90.0f);
							motor.forward(90.0f, 0);
							break;
						case 180:
							mouse.turn_180();
							motor.turn(180.0f);
							motor.forward(90.0f, 0);
							break;
					}
				}

				mouse.move_forward();
			}
		} 
	}

}