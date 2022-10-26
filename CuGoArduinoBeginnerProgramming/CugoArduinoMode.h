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
#define MAX_MOTOR_RPM 180 //モータの速度上限値

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

// PID位置制御のゲイン調整
#define L_COUNT_KP  0.008f
#define L_COUNT_KI  0.0f //速度上限を設定している場合はiは必ず0に
#define L_COUNT_KD  0.005f
#define R_COUNT_KP  0.008f
#define R_COUNT_KI  0.0f //速度上限を設定している場合はiは必ず0に
#define R_COUNT_KD  0.005f

#define CONTROLL_STOP_count  1000

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

//PID位置制御なしの場合の決め打ちの値
#define migimawari_count90  70
#define hidarimawari_count90  70
#define migimawari_count45  33
#define hidarimawari_count45  33
#define migimawari_count180  160
#define hidarimawari_count180  160


// グローバル変数宣言
extern long int arduino_cmd_matrix[CMD_SIZE][6];
extern int init_current_cmd;



//各種関数
  void init_SPI();
  void init_KOPROPO(int runMode,int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE);
  void init_ARDUINO_CMD();
  void set_arduino_cmd_matrix(long int cmd_0,long  int cmd_1, int cmd_2, int cmd_3,int cmd_4,int cmd_5);
  void send_spi(int mode);
  void view_arduino_cmd_matrix();
  void display_failsafe(bool FAIL_SAFE_DISPLAY,int runMode);
  void display_nothing(bool UDP_CONNECTION_DISPLAY,bool ENCODER_DISPLAY,bool PID_CONTROLL_DISPLAY);
  void spi_cmd(int spi_cmd_value,bool cmd_init);
  void calc_necessary_rotate(float degree,long int *target_count_L,long int *target_count_R); 
  void calc_necessary_count(float distance,long int *target_count_L,long int *target_count_R); // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
  void atamaopen(bool cmd_init);
  void atamaclose(bool cmd_init);
  void wait_button(bool cmd_init);
  void display_speed(MotorController motor_controllers[2],bool ENCODER_DISPLAY);
  void display_target_rpm(MotorController motor_controllers[2],bool ENCODER_DISPLAY);
  void display_PID(MotorController motor_controllers[2],bool PID_CONTROLL_DISPLAY);
  int split(String data, char delimiter, String *dst);
  void motor_direct_instructions(int left, int right,MotorController motor_controllers[2]);
  void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX],MotorController motor_controllers[2]);
  void stop_motor_immediately(MotorController motor_controllers[2]);
  void wait_time(int milisec,bool cmd_init);
  void reset_pid_gain(MotorController motor_controllers[2]);

#endif
