#ifndef CUGOARDUINOMODE_H
#define CUGOARDUINOMODE_H

//CugoArduinoSDKライブラリ
#include "RPi_Pico_TimerInterrupt.h"

#include "Arduino.h"
#include <SPI.h>
//#include <Servo.h>
//#include "MotorController.h"
#include "CugoArduinoMode.h"

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
#define wheel_radius_l  0.03858d
#define wheel_radius_r  0.03858d
#define tread  0.380d
//#define encoder_resolution  2048.0d　//360
#define encoder_resolution 360.0d
#define MAX_MOTOR_RPM 180 //モータの速度上限値
//↑の仕様が変更される場合は下の変換係数も変更してください。
#define conversion_distance_to_count 8448.660535308492d // 変換係数： encoder_resolution / (2 * wheel_radius_l * PI)の計算結果
#define conversion_count_to_distance 0.000118361958d    // 変換係数： 2 * wheel_radius_l * PI  / encoder_resolution の計算結果

//encoder_resolution / (2 * wheel_radius_l * PI);

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
#define L_COUNT_KP  0.04f
#define L_COUNT_KI  0.003f 
#define L_COUNT_KD  0.01f
#define R_COUNT_KP  0.04f
#define R_COUNT_KI  0.003f 
#define R_COUNT_KD  0.01f

#define L_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
#define R_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に


#define CONTROLL_STOP_count  1000

// Arduinoキットのスタートボタン
#define CMD_BUTTON_PIN 5 


// 動作モード定義
#define RC_MODE 0
#define ARDUINO_MODE 1

//各種閾値
#define ARDUINO_MODE_IN 1700  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
#define ARDUINO_MODE_OUT 1300  // ARDUINOモードから抜けるときの閾値(us) (1100~1900/中央1500)
#define CUGO_PROPO_MAX_A 2200
#define CUGO_PROPO_MIN_A 800
#define CUGO_PROPO_MAX_B 1900
#define CUGO_PROPO_MIN_B 1100
#define CUGO_PROPO_MAX_C 2200
#define CUGO_PROPO_MIN_C 800
#define CMD_SIZE 60 //　コマンド数上限
#define EXCEPTION_NO -32768 //int下限

//モーター設定
#define MOTOR_NUM 2 // モータ接続数（最大4の予定）
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

//PIN関連
#define PIN_UP(no)    upTime[no] = micros();
#define PIN_DOWN(no)  time[no] = micros() - upTime[no]
#define PWM_IN_MAX    3



/***** ↓必要に応じて各ユーザーごとに設定可能↓ *****/
// 回転方向ソフトウェア切り替え
//const bool CUGO_L_reverse = false;
//const bool CUGO_R_reverse = true;
// joshibi: L:True, R:false
// cugo-chan: L:false, R:True
  //config
//#define FEEDBACK_HZ 100
//#define FEEDBACK_DUTATION 10 //(1000UL /  FEEDBACK_HZ)

  extern volatile long FEEDBACK_HZ;
  extern volatile long FEEDBACK_DUTATION;


  //cugo仕様関連
    #define CUGO_WHEEL_RADIUS_L  0.03858d
    #define CUGO_WHEEL_RADIUS_R  0.03858d
    #define CUGO_TREAD  0.380d
    #define CUGO_ENCODER_RESOLUTION  2048.0d 
    #define CUGO_MAX_MOTOR_RPM 180 //モータの速度上限値
    //上記の仕様が変更される場合は下の変換係数も変更してください。
      #define CUGO_CONVERSION_DISTANCE_TO_COUNT 8448.660535308492d // 変換係数： CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI)の計算結果
      #define CUGO_CONVERSION_COUNT_TO_DISTANCE 0.000118361958d    // 変換係数： 2 * CUGO_WHEEL_RADIUS_L * PI  / CUGO_ENCODER_RESOLUTION の計算結果

  // PID位置制御のゲイン調整
    #define CUGO_L_COUNT_KP  50.0f
    #define CUGO_L_COUNT_KI  0.5f 
    #define CUGO_L_COUNT_KD  10.0f
    #define CUGO_R_COUNT_KP  50.0f
    #define CUGO_R_COUNT_KI  0.5f 
    #define CUGO_R_COUNT_KD  10.0f
    #define CUGO_L_MAX_COUNT_I  120 
    #define CUGO_R_MAX_COUNT_I  120 

  // CugoArduinoキットのスタートボタン
    #define CUGO_CMD_BUTTON_PIN 5

  // 各種動作モード定義
    //#define CUGO_RC_MODE 0
    //#define CUGO_SELF_DRIVE_MODE 1
    #define TIMER0_INTERVAL_MS 10

  //プロポ設定
    #define CUGO_PROPO_A 0
    #define CUGO_PROPO_B 1
    #define CUGO_PROPO_C 2

  //オドメトリ設定
    #define CUGO_ODO_X 0
    #define CUGO_ODO_Y 1
    #define CUGO_ODO_THETA 2
    #define CUGO_ODO_DEGREE 3

  //モーター設定
    #define CUGO_MOTOR_LEFT 0
    #define CUGO_MOTOR_RIGHT 1

  //各種閾値
    #define CUGO_NORMAL_MOTOR_RPM 50
    #define CUGO_BUTTON_CHECK_BORDER 50000


/***** ↑必要に応じて各ユーザーごとに設定可能↑ *****/


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
extern int runMode;
extern bool UDP_CONNECTION_DISPLAY;
extern bool ENCODER_DISPLAY;
extern bool PID_CONTROLL_DISPLAY;
extern bool FAIL_SAFE_DISPLAY;
extern const bool L_reverse;
extern const bool R_reverse;
extern float l_count_prev_i_;
extern float l_count_prev_p_;
extern float r_count_prev_i_;
extern float r_count_prev_p_;
extern float l_count_gain;
extern float r_count_gain;
extern int OLD_PWM_IN_PIN0_VALUE; 
extern int OLD_PWM_IN_PIN1_VALUE; 
extern int OLD_PWM_IN_PIN2_VALUE; 
extern volatile unsigned long upTime[PWM_IN_MAX];
extern volatile unsigned long rcTime[PWM_IN_MAX];
//extern volatile unsigned long time[PWM_IN_MAX];
extern long int count_prev_L;
extern long int count_prev_R;




// グローバル変数宣言※関数はCugoArduinoSDK.cppに記載
  extern int cugo_old_runmode;
  extern int cugo_button_count;
  extern long int cugo_count_prev_L;
  extern long int cugo_count_prev_R;
  extern unsigned long long int calc_odometer_time;
  extern float cugo_odometer_theta;
  extern float cugo_odometer_x;
  extern float cugo_odometer_y;
  extern float cugo_odometer_degree;
  extern long int cugo_target_count_L;
  extern long int cugo_target_count_R;
  extern long int cugo_odometer_count_theta;
  extern int cugoRunMode;
  //extern const bool CUGO_L_reverse;
  //extern const bool CUGO_R_reverse;
  extern bool cugo_direction_L; 
  extern bool cugo_direction_R; 
  extern bool cugo_button_check;
  extern bool cugo_button_check;
  extern int CUGO_OLD_CMD_BUTTON_VALUE; 
  extern int CUGO_OLD_PWM_IN_PIN0_VALUE; 
  extern int CUGO_OLD_PWM_IN_PIN1_VALUE; 
  extern int CUGO_OLD_PWM_IN_PIN2_VALUE; 
  //extern volatile unsigned long cugoUpTime[CUGO_PWM_IN_MAX];
  //extern volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX];
  //extern volatile unsigned long cugo_time[CUGO_PWM_IN_MAX];
  extern volatile unsigned long long cugoButtonStartTime;

//BLDC
  extern volatile long int  FEEDBACK_HZ ;
  extern volatile long int FEEDBACK_DUTATION;
  extern volatile long int id ;
  extern volatile float accelerationR ;
  extern volatile float accelerationL ;
  extern float rpm_current_R ;
  extern float rpm_current_L ;
  extern float target_rpmR ;
  extern float target_rpmL ;
  extern volatile short _encorderR ;
  extern volatile short _encorderL ;
  extern volatile long  __encorderR;
  extern volatile long __encorderL;




//RPI_PICO_Timer ITimer0(0);



//各種関数
  void init_display();
  void init_SPI();
  void init_KOPROPO();
  void init_ARDUINO_CMD();
  void set_arduino_cmd_matrix(long int cmd_0,long  int cmd_1, int cmd_2, int cmd_3,int cmd_4,int cmd_5);
  void send_spi(int mode);
  void view_arduino_cmd_matrix();
  void display_failsafe(bool FAIL_SAFE_DISPLAY);
  void display_nothing();
  void spi_cmd(int spi_cmd_value);
  void calc_necessary_rotate(float degree); 
  void calc_necessary_count(float distance); 
  void atamaopen();
  void atamaclose();
  void wait_button();
  void botan();
  void button();
  void display_speed( bool ENCODER_DISPLAY); 
  void display_target_rpm(bool ENCODER_DISPLAY);
  void display_PID( bool PID_CONTROLL_DISPLAY);
  int split(String data, char delimiter, String *dst);
  void motor_direct_instructions(int left, int right);
  void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX] );
  void stop_motor_immediately( );
  void set_wait_time_cmd();
  void wait_time(int milisec);
  void matsu(int milisec);
  void matu(int milisec);
  void reset_pid_gain( );
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
  void set_go_forward_cmd( );
  void view_flags();
  void check_achievement_spi_cmd();
  void cmd_end( );
  void check_achievement_wait_time_cmd( );
  void cmd_manager_flags_init( );  
  void check_achievement_go_forward_cmd( );
  void cmd_manager( );
  void check_achievement_button_cmd( );
  void job_100ms( );
  void job_1000ms();
  void display_detail( );

//各種関数
  //初期設定関数関連
    void cugo_init();
    bool TimerHandler0(struct repeating_timer *t);
    void cugo_init_display();
    //void cugo_init_KOPROPO(int CUGO_OLD_PWM_IN_PIN0_VALUE,int CUGO_OLD_PWM_IN_PIN1_VALUE,int CUGO_OLD_PWM_IN_PIN2_VALUE);
    void cugo_reset_pid_gain(  );
    void cugo_check_mode_change(  );
    void cugo_button_interrupt();

  //モータ直接制御関連
    void cugo_motor_direct_instructions(int left, int right );
    //void cugo_rcmode(volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX] );
    void cugo_stop( );

  //前進後進、回転、円軌道関数
    void cugo_move_forward(float target_distance );
    void cugo_move_forward(float target_distance,float target_rpm );//単位はm,rpm
    void cugo_move_forward_raw(float target_distance,float target_rpm );//単位はm,rpm
    void cugo_turn_clockwise(float target_degree );
    void cugo_turn_clockwise(float target_degree,float target_rpm );//単位はm,rpm
    void cugo_turn_clockwise_raw(float target_degree,float target_rpm );//単位はm,rpm
    void cugo_turn_counterclockwise(float target_degree );
    void cugo_turn_counterclockwise(float target_degree,float target_rpm );//単位はm,rpm
    void cugo_turn_counterclockwise_raw(float target_degree,float target_rpm );//単位はm,rpm  

  //極座標での移動命令関数
    void cugo_curve_theta_raw(float target_radius,float target_theta,float target_rpm );
    void cugo_curve_distance_raw(float target_radius,float target_distance,float target_rpm );

  //wait関数
    void cugo_wait(unsigned long long int  wait_ms);
    void cugo_long_wait(unsigned long long int wait_seconds);

  //プロポ入力確認関数
    int cugo_check_propo_channel_value(int channel_number); 
  //ボタン関連の関数
    bool cugo_check_button(); //現状の押されているか
    int cugo_check_button_times(); //現状の押された回数
    void cugo_reset_button_times(); //現状の押された回数の初期化
    long int cugo_button_press_time(); //ボタンの押されている時間

  //オドメトリ関連関数
    float cugo_check_odometer(int check_number); 
    void cugo_start_odometer();
    void cugo_calc_odometer(  );
    void cugo_reset_odometer();
  //その他関数
    void cugo_calc_necessary_rotate(float degree ); 
    void cugo_calc_necessary_count(float distance ); 
    bool cugo_check_count_achievement(int motor_num_ );
    void cugo_move_pid(float target_rpm,bool use_pid );//単位はm,rpm
  //テスト関数
    void cugo_test(int test_number );//テスト用関数
//BLDC
  //便利関数
    void FloatToUC(float data, long int start, unsigned char* index);      //配列indexの4番目からfloat dataを書き込む場合-> FloatTolong int(data, 4, index);
    void IndexToFloat(unsigned char* index, long int start, float* data);  //配列indexの3番目からfloat dataに書き込む場合-> IndexToFloat(index, 3, data);
    void IndexToShort(unsigned char* index, long int start, short* data);  //配列indexの3番目からulong int16_t dataに書き込む場合-> IndexToFloat(index, 3, data);
  //通信関係
    void write_bldc(unsigned char cmd[10]);
    void get_bldc();
  //通信処理関係
    void set_encorder(unsigned char frame[12]);
  //タイマー関係
    void BackGround();  //timerでgetDuration()[ms]間隔で呼んでほしい関数
  //取得系
    void Encorder_reset();
  //設定
    void set_feedback(unsigned char freq_index, unsigned char kindof_data);  //freq{0:10[hz] 1:50[hz] 2:100[hz]} kindof_data{0b1:Mode 0b10:CMD_RPM 0b100:CurrentRPM 0b1000:AveCurrentRPM 0b10000000:EncorderData}
    void setControlMode(unsigned char mode);                       //mode{0:RC_mode 1:CMD_Mode}
    long int convert_distanceTopulse(float distance);
    float convert_pulseTodistance(long int pulse);  //ピッチ円直径77.16mm 360パルスで 242.4/360 = 0.67333333333

//削除予定
/*
  //インスタンス生成時
  //c_BLDC_Driver();



 // モータとエンコーダのピン配置設定
  //#define CUGO_PIN_MOTOR_L A0  // モータ出力ピン(L)
  //#define CUGO_PIN_MOTOR_R A1  // モータ出力ピン(R)
  //#define CUGO_PIN_ENCODER_L_A 2  // エンコーダ割り込み入力ピン(L)
  //#define CUGO_PIN_ENCODER_L_B 8  // エンコーダ回転方向入力ピン(L)
  //#define CUGO_PIN_ENCODER_R_A 3  // エンコーダ割り込み入力ピン(R)
  //#define CUGO_PIN_ENCODER_R_B 9  // エンコーダ回転方向入力ピン(R)

 // プロポ信号の読み取りピン（L/R/MODE_CAHNGE）
  //#define CUGO_PWM_IN_PIN0   5   // プロポスティック入力ピン(L)//digitalRead ピン変化割り込みの設定
  //#define CUGO_PWM_IN_PIN1   6   // プロポスティック入力ピン(MODE)//digitalRead ピン変化割り込みの設定
  //#define CUGO_PWM_IN_PIN2   7   // プロポスティック入力ピン(R)//digitalRead ピン変化割り込みの設定
 // PID速度制御ゲイン調整 MotorControllerで使用
  //#define CUGO_L_KP   1.0f   //CuGoV3
  //#define CUGO_L_KI   0.02f   //CuGoV3
  //#define CUGO_L_KD   0.1f   //CuGoV3
  //const float CUGO_L_KP = 1.0;
  //const float CUGO_L_KI = 0.06;
  //const float CUGO_L_KD = 0.1;
  //#define CUGO_R_KP   1.0f   //CuGoV3
  //#define CUGO_R_KI   0.02f   //CuGoV3
  //#define CUGO_R_KD   0.1f   //CuGoV3
  //const float CUGO_R_KP = 1.0;
  //const float CUGO_R_KI = 0.06;
  //const float CUGO_R_KD = 0.1;

  // ローパスフィルタ
    //#define CUGO_L_LPF   0.2f 
    //#define CUGO_R_LPF   0.2f
    //const float CUGO_L_LPF = 0.2;
    //const float CUGO_R_LPF = 0.2;
    //#define CUGO_L_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
    //#define CUGO_R_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に

  //各種閾値
  #define CUGO_PROPO_MAX_A   2200  // SELF_DRIVEモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_PROPO_MIN_A   800  // SELF_DRIVEモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_PROPO_MAX_B   1900  // SELF_DRIVEモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_PROPO_MIN_B   1100  // SELF_DRIVEモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_PROPO_MAX_C   2200  // SELF_DRIVEモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_PROPO_MIN_C   800  // SELF_DRIVEモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_SELF_DRIVE_MODE_IN   1700  // SELF_DRIVEモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_SELF_DRIVE_MODE_OUT  1300  // SELF_DRIVEモードから抜けるときの閾値(us) (1100~1900/中央1500)
  #define CUGO_EXCEPTION_NO -32768 //int下限

  //割り込みPIN関連 
    #define CUGO_PIN_UP(no)    cugoUpTime[no] = micros();
    #define CUGO_PIN_DOWN(no)  cugo_time[no] = micros() - cugoUpTime[no]
    #define CUGO_PWM_IN_MAX  4

  //BLDC動作
  void forward(long int milli_metter);
  void rotation(long int degree, long int radius_milli);
  void setSpeed(float left_rpm, float right_rpm);   //非同期処理 IOcontrolと並行
  void accelR(float accel_rpmR, float target_rpm);  //一秒あたりの加速 非同期処理 IOcontrolと並行
  void accelL(float accel_rpmL, float target_rpm);  //一秒あたりの加速 非同期処理 IOcontrolと並行
  void move_stop(long int pulsesR, long int pulsesL, float accel_rpmR, float accel_rpmL, float max_rpmR, float max_rpmL, float brake_rpmR, float brake_rpmL);
  void move_stop(long int pulses, float accel_rpm, float max_rpm, float brake_rpm);  //同期処理 加速度accel_rpmで加速しmax_rpmで走り加速度-brake_rpmで減速する。道のりはpulses[パルス]になるようにする。
  void move_stop(long int milli_meter);                                              //同期処理 加速度accel_rpmで加速しmax_rpmで走り加速度-brake_rpmで減速する。道のりはpulses[パルス]になるようにする。
  void rotation_stop(long int degree, float accel_rpm, float max_rpm, float brake_rpm);
  void rotation_stop(long int degree);
  long getEncorder_L();
  long getEncorder_R();
  long int getDuration();
*/



#endif
