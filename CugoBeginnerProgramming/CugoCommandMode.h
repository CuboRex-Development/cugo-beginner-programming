#ifndef CUGOCOMMANDMODE_H
#define CUGOCOMMANDMODE_H

//グローバル変数が圧迫されている場合はCMD_SIZEを変更してください

//インクルード関連
  #include <SPI.h>
  //#include <math.h>
  #include "Arduino.h"
  #include "RPi_Pico_TimerInterrupt.h"
  #include "CugoCommandMode.h"

//cugo仕様関連
  #define wheel_radius_l  0.03858d
  #define wheel_radius_r  0.03858d
  #define tread  0.380d
  #define encoder_resolution 360.0d 
  #define MAX_MOTOR_RPM 180 //モータの速度上限値

//PID位置制御のゲイン調整
  #define L_COUNT_KP  0.04f
  #define L_COUNT_KI  0.003f 
  #define L_COUNT_KD  0.01f
  #define R_COUNT_KP  0.04f
  #define R_COUNT_KI  0.003f 
  #define R_COUNT_KD  0.01f
  #define L_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
  #define R_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
  #define COUNT_MAX 65536
  #define TIME_MAX 4200000 //時間計測は７０分まで


//KITのスタートボタン
  //#define CMD_BUTTON_PIN 5 

//動作モード定義
  #define RC_MODE 0
  #define COMMAND_MODE 1

//各種閾値
  #define CMD_SIZE 4096 //コマンド数上限：コマンド数上限は変更可能です。
  #define EXCEPTION_NO -32768 //int下限

//グローバル変数宣言
//cmd_matrix関連
  extern long int count_cmd_matrix[CMD_SIZE][2];
  extern int flag_cmd_matrix[CMD_SIZE][4];
  extern int init_current_cmd;
  extern int current_cmd;

//カウント関連
  extern long int target_count_L;
  extern long int target_count_R;
  extern long int start_count_L;
  extern long int start_count_R;
  extern long int current_count_L;
  extern long int current_count_R;
  extern volatile long current_encoder_R;
  extern volatile long current_encoder_L;
  extern long int prev_encoder_L;
  extern long int prev_encoder_R;

//各種フラグ関連
  extern bool end_command_mode;
  extern bool cmd_init;
  extern bool cmd_exec;
  extern bool cmd_L_back;
  extern bool cmd_R_back;
  extern bool count_done;
  extern bool wait_done;
  extern bool button_done;
  extern bool spi_done;
  extern int run_mode;
  extern int old_run_mode;
  extern int button_push_count;

//時間関連
  extern unsigned long long current_time;
  extern unsigned long long prev_time_10ms; 
  extern unsigned long long prev_time_100ms; 
  extern unsigned long long prev_time_1000ms; 
  extern long int target_wait_time;

//PID位置制御関連
  extern float l_count_prev_i_;
  extern float l_count_prev_p_;
  extern float r_count_prev_i_;
  extern float r_count_prev_p_;
  extern float l_count_gain;
  extern float r_count_gain;

//ld2関連
  extern volatile long ld2_id ;
  extern volatile long ld2_feedback_hz;
  extern volatile long ld2_feedback_dutation;

//各種関数
//初期化関数
  void init_display();
  void init_CMD();
  void reset_command_mode_flags();
  void cmd_manager_flags_init( );  

//cmd_matrix関連
  void set_cmd_matrix(long int cmd_0,long  int cmd_1, int cmd_2, int cmd_3,int cmd_4,int cmd_5);
  void set_button_cmd();
  void set_wait_time_cmd();
  void set_go_forward_cmd( );
  void check_achievement_button_cmd( );
  void check_achievement_wait_time_cmd( );
  void check_achievement_go_forward_cmd( );
  void cmd_manager( );
  void cmd_end( );

//計算関連
  void calc_necessary_rotate(float degree); 
  void calc_necessary_count(float distance); 

//ボタン関連
  void wait_button();
  void botan();
  void button();

//待機関連
  void wait_time(long int milisec);
  void matsu(long int milisec);
  void matu(long int milisec);

//モーター制御関連
  void motor_direct_instructions(int left, int right);
  void stop_motor_immediately( );

//前進後進関連
  void go_forward(float distance,float max_velocity);
  void susumu(float distance);
  void susumu(float distance,float max_velocity);
  void go_backward(float distance,float max_velocity);
  void sagaru(float distance);
  void sagaru(float distance,float max_velocity);

//左右回転関連
  void turn_clockwise(float degree,float max_velocity);
  void migimawari(float degree);
  void migimawari(float degree,float max_velocity);
  void migimawari90();
  void migimawari90(float max_velocity);
  void migimawari45();
  void migimawari45(float max_velocity);
  void migimawari180();
  void migimawari180(float max_velocity);
  void turn_counter_clockwise(float degree,float max_velocity);
  void hidarimawari(float degree);
  void hidarimawari(float degree,float max_velocity);
  void hidarimawari90();
  void hidarimawari90(float max_velocity);
  void hidarimawari45();
  void hidarimawari45(float max_velocity);
  void hidarimawari180();
  void hidarimawari180(float max_velocity);

//ld2関数
    bool ld2_timer_handler(struct repeating_timer *t);
    void ld2_float_to_frame(float data, long int start, unsigned char* index);      //配列indexの4番目からfloat dataを書き込む場合-> FloatTolong int(data, 4, index);
    void ld2_frame_to_float(unsigned char* index, long int start, float* data);  //配列indexの3番目からfloat dataに書き込む場合-> ld2_frame_to_float(index, 3, data);
    void ld2_frame_to_short(unsigned char* index, long int start, short* data);  //配列indexの3番目からulong int16_t dataに書き込む場合-> ld2_frame_to_float(index, 3, data);
    void ld2_write_cmd(unsigned char cmd[10]);
    void ld2_get_cmd();
    void ld2_set_encoder(unsigned char frame[12]);
    void ld2_encoder_reset();
    void ld2_set_feedback(unsigned char freq_index, unsigned char kindof_data);  //freq{0:10[hz] 1:50[hz] 2:100[hz]} kindof_data{0b1:Mode 0b10:CMD_RPM 0b100:CurrentRPM 0b1000:AveCurrentRPM 0b10000000:encoderData}
    void ld2_set_control_mode(unsigned char mode);                       //mode{0:RC_mode 1:CMD_Mode}


//未使用変数および関数
  #define MOTOR_LEFT 0
  #define MOTOR_RIGHT 1

  extern bool ENCODER_DISPLAY;
  extern bool PID_CONTROLL_DISPLAY;
  extern bool FAIL_SAFE_DISPLAY;
  extern bool spi_done;

  void send_spi(int mode);
  void spi_cmd(int spi_cmd_value);
  void check_achievement_spi_cmd();
  int split(String data, char delimiter, String *dst);


  void view_cmd_matrix();
  void view_flags();
  void display_failsafe(bool FAIL_SAFE_DISPLAY);
  void display_nothing();
  void display_speed( bool ENCODER_DISPLAY); 
  void display_target_rpm(bool ENCODER_DISPLAY);
  //void display_PID( bool PID_CONTROLL_DISPLAY);
  void display_detail();

  void job_100ms();
  void job_1000ms();
  //void cugo_test(int test_number );//テスト用関数

#endif
