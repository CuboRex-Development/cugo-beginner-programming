// CugoArduinoBeginnerProguramingのリファクタリングプログラム
/* ★　元ファイルからの変更箇所概要
 * 各関数や定数変数のファイル移行
 * グローバル変数利用時の引数は一部参照渡しに変更　
 * set_arduino_cmd_matrixのinit_current_cmdは参照渡しへ変更
 * motor_controllersを引数に追加
*/

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
#define PWM_IN_PIN0   5   // プロポスティック入力ピン(L)//digitalRead ピン変化割り込みの設定
#define PWM_IN_PIN1   6   // プロポスティック入力ピン(MODE)//digitalRead ピン変化割り込みの設定
#define PWM_IN_PIN2   7   // プロポスティック入力ピン(R)//digitalRead ピン変化割り込みの設定

//cugo仕様関連
#define wheel_radius_l  0.03858f
#define wheel_radius_r  0.03858f
#define tread  0.380f
#define encoder_resolution  2048
#define MAX_MOTOR_RPM 180 //モータの速度上限値

// PID ゲイン調整
// L側
// 元ファイルからの★変更箇所　const floatからdefineへ
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
#define L_COUNT_KP  0.02f
#define L_COUNT_KI  0.001f 
#define L_COUNT_KD  0.01f
#define R_COUNT_KP  0.02f
#define R_COUNT_KI  0.001f 
#define R_COUNT_KD  0.01f

#define L_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
#define R_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に


#define CONTROLL_STOP_count  1000

// Arduinoキットのスタートボタン
#define CMD_BUTTON_PIN A2 


// 動作モード定義
#define RC_MODE 0
#define ARDUINO_MODE 1

//各種閾値
#define ARDUINO_MODE_IN   1700  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
#define ARDUINO_MODE_OUT  1300  // ARDUINOモードから抜けるときの閾値(us) (1100~1900/中央1500)
#define CMD_SIZE 60 //　コマンド数上限
#define EXCEPTION_NO -32768 //int下限？

//モーター設定
#define MOTOR_NUM 2 // モータ接続数（最大4の予定）
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

//PIN関連
#define PIN_UP(no)    upTime[no] = micros();
#define PIN_DOWN(no)  time[no] = micros() - upTime[no]
#define PWM_IN_MAX    3

//PID位置制御なしの場合の決め打ちの値 //現在は使ってない★
#define migimawari_count90  70
#define hidarimawari_count90  70
#define migimawari_count45  33
#define hidarimawari_count45  33
#define migimawari_count180  160
#define hidarimawari_count180  160


// グローバル変数宣言
extern long int arduino_count_cmd_matrix[CMD_SIZE][2];
extern int arduino_flag_cmd_matrix[CMD_SIZE][4];
extern int init_current_cmd;

extern long int target_count_L;
extern long int target_count_R;
extern long int target_wait_time;
extern int button_push_count;
extern bool button_enable;
extern bool cmd_init;
extern int current_cmd;
extern bool cmd_L_back;
extern bool cmd_R_back;
extern bool cmd_exec;
extern bool count_done;
extern bool wait_done;
extern bool button_done;
extern bool spi_done;
extern bool end_arduino_mode;
extern unsigned long long current_time;
extern unsigned long long prev_time_10ms; 
extern unsigned long long prev_time_100ms; 
extern unsigned long long prev_time_1000ms; 
//RUN_MODE runMode = RC_MODE;  // 初回起動時はRC_MODE（無意識な暴走を防ぐため）
extern int runMode;


//各種関数
  void init_SPI();
  void init_KOPROPO(int runMode,int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE);
  void init_ARDUINO_CMD();
  void set_arduino_cmd_matrix(long int cmd_0,long  int cmd_1, int cmd_2, int cmd_3,int cmd_4,int cmd_5);//★init_current_cmdは参照渡しへ
  void send_spi(int mode);
  void view_arduino_cmd_matrix();
  void display_failsafe(bool FAIL_SAFE_DISPLAY,int runMode);
  void display_nothing(bool UDP_CONNECTION_DISPLAY,bool ENCODER_DISPLAY,bool PID_CONTROLL_DISPLAY);
  void spi_cmd(int spi_cmd_value);
  void calc_necessary_rotate(float degree); 
  void calc_necessary_count(float distance); // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
  void atamaopen();
  void atamaclose();
  void wait_button();
  void botan();
  void button();
  void display_speed(MotorController motor_controllers[2],bool ENCODER_DISPLAY); 
  void display_target_rpm(MotorController motor_controllers[2],bool ENCODER_DISPLAY);
  void display_PID(MotorController motor_controllers[2],bool PID_CONTROLL_DISPLAY);
  int split(String data, char delimiter, String *dst);
  void motor_direct_instructions(int left, int right,MotorController motor_controllers[2]);
  void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX],MotorController motor_controllers[2]);
  void stop_motor_immediately(MotorController motor_controllers[2]);
  void set_wait_time_cmd();
  void wait_time(int milisec);
  void matsu(int milisec);
  void matu(int milisec);
  void reset_pid_gain(MotorController motor_controllers[2]);
  void set_button_cmd();
  void go_backward(float distance,float max_velocity);
  void sagaru(float distance);
  void sagaru(float distance,float max_velocity);
  void turn_clockwise(float degree,float max_velocity);
  void migimawari(float degree);
  void migimawari(float degree,float max_velocity);
  void migimawari90();
  void migimawari90(float max_velocity);
  void migimawari45();
  void migimawari45(float max_velocity);
  void migimawari180();
  void migimawari180(float max_velocity);
  void go_forward(float distance,float max_velocity);
  void susumu(float distance);
  void susumu(float distance,float max_velocity);
  void turn_counter_clockwise(float degree,float max_velocity);
  void hidarimawari(float degree);
  void hidarimawari(float degree,float max_velocity);
  void hidarimawari90();
  void hidarimawari90(float max_velocity);
  void hidarimawari45();
  void hidarimawari45(float max_velocity);
  void hidarimawari180();
  void hidarimawari180(float max_velocity);
  void reset_arduino_mode_flags();
  void set_go_forward_cmd(MotorController motor_controllers[2]);
  void view_flags();
  void check_achievement_spi_cmd();
  void cmd_end(MotorController motor_controllers[2]);
  void check_achievement_wait_time_cmd(MotorController motor_controllers[2]);
  void cmd_manager_flags_init(MotorController motor_controllers[2]);  

#endif
