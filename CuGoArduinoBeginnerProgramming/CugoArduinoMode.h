// CugoArduinoBeginnerProguramingのリファクタリングプログラム

#ifndef CUGOARDUINOMODE_H
#define CUGOARDUINOMODE_H

#include "MotorController.h"
#include <Servo.h>

// モータとエンコーダのピン配置設定
#define PIN_MOTOR_L A0  // モータ出力ピン(L)
#define PIN_MOTOR_R A1  // モータ出力ピン(R)
#define PIN_ENCODER_L_A 2  // エンコーダ割り込み入力ピン(L)
#define PIN_ENCODER_L_B 8  // エンコーダ回転方向入力ピン(L)
#define PIN_ENCODER_R_A 3  // エンコーダ割り込み入力ピン(R)
#define PIN_ENCODER_R_B 9  // エンコーダ回転方向入力ピン(R)

// プロポ信号の読み取りピン（L/R/MODE_CAHNGE）
#define PWM_IN_PIN0   5   // プロポスティック入力ピン(L)
#define PWM_IN_PIN1   6   // プロポスティック入力ピン(MODE)
#define PWM_IN_PIN2   7   // プロポスティック入力ピン(R)

//cugo仕様関連
#define wheel_radius_l  0.03858f
#define wheel_radius_r  0.03858f
#define tread  0.380f
#define encoder_resolution  2048


// PID ゲイン調整
// L側
#define L_KP   1.0f   //CuGoV3
#define L_KI   0.02f   //CuGoV3
#define L_KD   0.1f   //CuGoV3

//const float L_KP = 1.0;
//const float L_KI = 0.06;
//const float L_KD = 0.1;

// R側
#define R_KP   1.0f   //CuGoV3
#define R_KI   0.02f   //CuGoV3
#define R_KD   0.1f   //CuGoV3
//const float R_KP = 1.0;
//const float R_KI = 0.06;
//const float R_KD = 0.1;

// ローパスフィルタ
#define L_LPF   0.2f 
#define R_LPF   0.2f
//const float L_LPF = 0.2;
//const float R_LPF = 0.2;


// Arduinoキットのスタートボタン
#define CMD_BUTTON_PIN A2 


// 動作モード定義
#define RC_MODE 0
#define ARDUINO_MODE 1

//各種閾値
#define ARDUINO_MODE_IN   1700  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
#define ARDUINO_MODE_OUT  1300  // ARDUINOモードから抜けるときの閾値(us) (1100~1900/中央1500)
#define CMD_SIZE 20 //　コマンド数上限
#define EXCEPTION_NO -32768 //int下限？

//モーター設定
#define MOTOR_NUM 2 // モータ接続数（最大4の予定）
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

//PIN関連
#define PIN_UP(no)    upTime[no] = micros();
#define PIN_DOWN(no)  time[no] = micros() - upTime[no]
#define PWM_IN_MAX    3



  void init_SPI();
  void init_KOPROPO(int runMode,int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE);
  void init_ARDUINO_CMD(int arduino_cmd_matrix[CMD_SIZE][6]);
  void set_arduino_cmd_matrix(int cmd_no, int cmd_0, int cmd_1, int cmd_2, int cmd_3, int cmd_4, int cmd_5,int arduino_cmd_matrix[CMD_SIZE][6]);
  void send_spi(int mode);
  void view_arduino_cmd_matrix(int arduino_cmd_matrix[CMD_SIZE][6]);
  void display_failsafe(bool FAIL_SAFE_DISPLAY,int runMode);
  void display_nothing(bool UDP_CONNECTION_DISPLAY,bool ENCODER_DISPLAY,bool PID_CONTROLL_DISPLAY);
  void spi_cmd(int spi_cmd_value,int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init);
  void calc_necessary_rotate(float degree,long int *target_count_L,long int *target_count_R); 
  void calc_necessary_count(float distance,long int *target_count_L,long int *target_count_R); // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
  void atamaopen(int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init);
  void atamaclose(int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init);
  void wait_button(int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init);
  void display_speed(MotorController motor_controllers[2],bool ENCODER_DISPLAY);
  void display_target_rpm(MotorController motor_controllers[2],bool ENCODER_DISPLAY);
  void display_PID(MotorController motor_controllers[2],bool PID_CONTROLL_DISPLAY);
  int split(String data, char delimiter, String *dst);
  void motor_direct_instructions(int left, int right,MotorController motor_controllers[2]);
  void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX],MotorController motor_controllers[2]);
  void stop_motor_immediately(MotorController motor_controllers[2]);
  void wait_time(int milisec,bool cmd_init,int *init_current_cmd,int arduino_cmd_matrix[CMD_SIZE][6] );

#endif