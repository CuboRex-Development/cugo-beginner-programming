#include "CugoArduinoMode.h"
//#include "CugoArduinoSDKPico.h"
#include "Arduino.h"
#include <math.h>


/***** ↓必要に応じて各ユーザーごとに設定可能↓ *****/
// シリアル通信での情報の表示有無
bool UDP_CONNECTION_DISPLAY = false;
bool ENCODER_DISPLAY = false;
bool PID_CONTROLL_DISPLAY = false;
bool FAIL_SAFE_DISPLAY = false;

// 回転方向ソフトウェア切り替え
const bool L_reverse = false;
const bool R_reverse = true;

// joshibi: L:True, R:false
// cugo-chan: L:false, R:True

/***** ↑必要に応じて各ユーザーごとに設定可能↑ *****/


long int arduino_count_cmd_matrix[CMD_SIZE][2];
int arduino_flag_cmd_matrix[CMD_SIZE][4];
int init_current_cmd = 0;

long int target_count_L = 0;
long int target_count_R = 0;
long int count_prev_L = 0;
long int count_prev_R = 0;

long int target_wait_time = 0;
int button_push_count = 0;
bool button_enable = false;
bool cmd_init = false;
int current_cmd = 0;
bool cmd_L_back = false;
bool cmd_R_back = false;
bool cmd_exec = false;
bool count_done  = false;
bool wait_done   = false;
bool button_done = false;
bool spi_done    = false;
bool end_arduino_mode = false;
unsigned long long current_time = 0; // オーバーフローしても問題ないが64bit確保
unsigned long long prev_time_10ms = 0; // オーバーフローしても問題ないが64bit確保
unsigned long long prev_time_100ms = 0; // オーバーフローしても問題ないが64bit確保
unsigned long long prev_time_1000ms = 0; // オーバーフローしても問題ないが64bit確保
int runMode = ARDUINO_MODE;
// PID位置制御のデータ格納
float l_count_prev_i_ = 0;
float l_count_prev_p_ = 0;
float r_count_prev_i_ = 0;
float r_count_prev_p_ = 0;
float l_count_gain = 0;
float r_count_gain = 0;

//int OLD_PWM_IN_PIN0_VALUE;   // プロポスティック入力値(L)
//int OLD_PWM_IN_PIN1_VALUE;   // プロポスティック入力値(MODE)
//int OLD_PWM_IN_PIN2_VALUE;   // プロポスティック入力値(R)
//volatile unsigned long upTime[PWM_IN_MAX];
//volatile unsigned long rcTime[PWM_IN_MAX];
//volatile unsigned long time[PWM_IN_MAX];


  volatile long FEEDBACK_HZ =100UL;
  volatile long FEEDBACK_DUTATION=10UL;



void init_SPI()
{
  //Serial.println(F("#   init_SPI"));//確認用

  SPI.begin();
  digitalWrite(SS, HIGH);
}

void send_spi(int mode) {
  //Serial.println(F("#   send_spi"));//確認用
  digitalWrite(SS, LOW);
  SPI.transfer(mode);
  digitalWrite(SS, HIGH);
}

void init_KOPROPO()
{
  //Serial.println(F("#   init_KOPROPO"));//確認用
  // ピン変化割り込みの初期状態保存
  runMode = RC_MODE;
  /*
  OLD_PWM_IN_PIN0_VALUE = digitalRead(PWM_IN_PIN0);
  OLD_PWM_IN_PIN1_VALUE = digitalRead(PWM_IN_PIN1);
  OLD_PWM_IN_PIN2_VALUE = digitalRead(PWM_IN_PIN2);

  // ピン変化割り込みの設定（D5,D6,D7をレジスタ直接読み取りで割り込み処理）
  pinMode(PWM_IN_PIN0, INPUT);
  pinMode(PWM_IN_PIN1, INPUT);
  pinMode(PWM_IN_PIN2, INPUT);
  //PCMSK2 |= B11100000;  // D5,6,7を有効
  //PCICR  |= B00000100;  // PCIE2を有効

  pinMode(LED_BUILTIN, OUTPUT); // Arduino/RC MODEの表示
  delay(100);
  */
}

void set_arduino_cmd_matrix(long int cmd_0, long int cmd_1, int cmd_2, int cmd_3, int cmd_4, int cmd_5)
{
  //Serial.println(F("#   set_arduino_cmd_matrix"));//確認用
  arduino_count_cmd_matrix[init_current_cmd][0] = cmd_0;//L側目標カウント数
  arduino_count_cmd_matrix[init_current_cmd][1] = cmd_1;//R側目標カウント数
  arduino_flag_cmd_matrix[init_current_cmd][0] = cmd_2;//milisecがEXCEPTION_NO以外なら待ち
  arduino_flag_cmd_matrix[init_current_cmd][1] = cmd_3;//255ならボタンまち
  arduino_flag_cmd_matrix[init_current_cmd][2] = cmd_4;//L側上限速度
  arduino_flag_cmd_matrix[init_current_cmd][3] = cmd_5;//R側上限速度

}

void init_ARDUINO_CMD()
{
  //Serial.println(F("#   init_ARDUINO_CMD"));//確認用
  pinMode(CMD_BUTTON_PIN, INPUT_PULLUP);
  for (int i = 0; i < CMD_SIZE; i++)
  {
    init_current_cmd = i;
    set_arduino_cmd_matrix(EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO);
  }
  init_current_cmd = 0;//初期化
}

void view_arduino_cmd_matrix()
{
  //Serial.println(F("#   view_arduino_cmd_matrix"));//確認用
  for (int i = 0; i < CMD_SIZE; i++)
  {
    Serial.println(arduino_count_cmd_matrix[i][0]);
    Serial.println(arduino_count_cmd_matrix[i][1]);
    Serial.println(arduino_flag_cmd_matrix[i][0]);
    Serial.println(arduino_flag_cmd_matrix[i][1]);
    Serial.println(arduino_flag_cmd_matrix[i][2]);
    Serial.println(arduino_flag_cmd_matrix[i][3]);
    Serial.println(i);
  }

}

void display_failsafe(bool FAIL_SAFE_DISPLAY)
{
  //Serial.println(F("#   display_failsafe"));//確認用
  if (FAIL_SAFE_DISPLAY == true)
  {
    Serial.println(F("DISPLAY FAIL SAFE PARAM"));
    Serial.print(F("Mode(ARDUINO/RC): "));
    Serial.println(runMode);
    Serial.print(F("UDP recieve fail count: "));
    Serial.println(F(""));
  }
}

void display_nothing()//1000msごとに表示したいものがあれば記載
{
  //Serial.println(F("#   display_nothing"));//確認用
  if (UDP_CONNECTION_DISPLAY == false && ENCODER_DISPLAY == false && PID_CONTROLL_DISPLAY == false)
  {
    //Serial.println(F("Display item not set"));
    //Serial.println(F("Arduino is working..."));
    //Serial.println(F(""));
  }
}

void spi_cmd(int spi_cmd_value)
{
  //Serial.println(F("#   spi_cmd"));//確認用
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.print(F("init_current_cmd: "));
      Serial.println(String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);

    }
    // 初回起動時の処理
    set_arduino_cmd_matrix( EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, spi_cmd_value, 0, 0);
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void set_wait_time_cmd()
{
  //Serial.println(F("#   set_wait_time_cmd"));//確認用
  //target_wait_time = micros() + arduino_flag_cmd_matrix[current_cmd][0] * 1000;
  target_wait_time = arduino_flag_cmd_matrix[current_cmd][0];
  target_wait_time = target_wait_time * 1000; // こちらで1000倍処理
  target_wait_time = target_wait_time + micros();

}

void wait_time(int milisec)
{

  if (cmd_init == false)
  {
    //Serial.println(F("#   wait_time"));//確認用
    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.print(F("init_current_cmd: "));
      Serial.println(String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);
    }
    // 初回起動時の処理
    set_arduino_cmd_matrix(EXCEPTION_NO, EXCEPTION_NO, milisec, EXCEPTION_NO, 0, 0);
    if (ARDUINO_MODE == runMode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      Serial.print(String(milisec));
      Serial.println(F("ms待つ"));
    }
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
void check_achievement_wait_time_cmd( )
{
  //Serial.println(F("#   check_achievement_wait_time_cmd"));//確認用
  if (target_wait_time < micros())
  {
    stop_motor_immediately( );
    wait_done = true;
    Serial.print(F("###"));
    if (current_cmd < 9)
      Serial.print(F("0"));

    Serial.print(String(current_cmd + 1));
    Serial.print(F("番目のコマンド：終了  "));
    Serial.print(String(arduino_flag_cmd_matrix[current_cmd][0] + (micros() - target_wait_time) / 1000 ));
    Serial.println(F("ミリ秒　待った  ###"));
  }
}

void matsu(int milisec)
{
  wait_time(milisec);
}

void matu(int milisec)
{
  wait_time(milisec);
}

void calc_necessary_rotate(float degree)
{
  //Serial.println(F("#   calc_necessary_rotate"));//確認用
  target_count_L =  ((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_l * PI);
  target_count_R = -((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_r * PI);
  Serial.println("target_count_L:" + String(target_count_L));
  //Serial.println("degree: " + String(degree));
  //Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * tread * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
}


void calc_necessary_count(float distance)
{
  //Serial.println(F("#   calc_necessary_count"));//確認用
    target_count_L = distance * encoder_resolution / (2 * wheel_radius_l * PI);
    target_count_R = distance * encoder_resolution / (2 * wheel_radius_r * PI);

  //target_count_L = distance / (2 * wheel_radius_l * PI);
  //target_count_R = distance / (2 * wheel_radius_r * PI);
  //target_count_L = target_count_L * encoder_resolution;
  //target_count_R = target_count_R * encoder_resolution;
  //target_count_L = distance * conversion_distance_to_count;
  //target_count_R = distance * conversion_distance_to_count;
  //target_count_L = convert_distanceTopulse(distance *1000.0);
  //target_count_R = convert_distanceTopulse(distance *1000.0);

  //Serial.println("distance: " + String(distance));
  //Serial.println("distance: " + String(encoder_resolution));
  //Serial.println("2 * wheel_radius_l * PI: " + String(2 * wheel_radius_l * PI));
  //Serial.println("calc: " + String(distance * encoder_resolution / (2 * wheel_radius_l * PI)));

  //Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("distance: " + String(distance));
  //Serial.println("wheel_radius_l: " + String(wheel_radius_l));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));

}


void atamaopen()
{
  spi_cmd(6);
}

void atamaclose()
{
  spi_cmd(5);
}

void wait_button()
{
  //Serial.println(F("#   wait_button"));//確認用
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);

    }
    // 初回起動時の処理
    set_arduino_cmd_matrix(EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, 255, 0, 0);
    if (ARDUINO_MODE == runMode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      Serial.println(F("ボタン　押し待ち"));
    }
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
void botan()
{
  wait_button();
}
void button()
{
  wait_button();
}

void display_speed( bool ENCODER_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{

  if (ENCODER_DISPLAY == true)
  {
    //Serial.println(F("#   display_speed"));//確認用
    //Serial.println("DISPLAY MOTOR COUNTER & SPEED");
    //Serial.print("Mode:");
    //Serial.println(runMode);

    Serial.print(F("Encoder count (L/R):"));
    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm());   // 制御量を見るため。開発用
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。開発用
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    //Serial.print(F(","));
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());  //制御量を見るため。
    //Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));


    //Serial.print("PID CONTROL RPM(L/R):");
    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。
    //Serial.print(",");
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());    //制御量を見るため。

    //Serial.println(""); // 改行
  }
}
void display_target_rpm( bool ENCODER_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{

  if (ENCODER_DISPLAY == true)
  {
    //Serial.println(F("#   display_target_rpm"));//確認用
    Serial.print(F("target_rpm[L]:"));
    //Serial.println(String(motor_controllers[MOTOR_LEFT].getTargetRpm()));
    Serial.print(F("target_rpm[R]:"));
    //Serial.println(String(motor_controllers[MOTOR_RIGHT].getTargetRpm()));
  }
}
void display_PID( bool PID_CONTROLL_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{


  if (PID_CONTROLL_DISPLAY == true)
  {
    //Serial.println("#   display_PID");// 確認用
    Serial.print(F("Encoder count (L/R): "));
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    Serial.print(F(","));
    //Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));

    Serial.print(F("Target RPM (L/R): "));
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getTargetRpm()));
    Serial.print(F(","));
    //Serial.println(String(motor_controllers[MOTOR_RIGHT].getTargetRpm()));

    Serial.print(F("PID CONTROL RPM(L/R):"));
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getRpm()));
    Serial.print(F(","));
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用

    Serial.println(F("PID controll gain = P x kp + I x ki + D x kd"));
    Serial.print(F("[L]: "));
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getSpeed()));
    Serial.print(F(" = "));
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_P()));
    Serial.print(F(" x "));
    Serial.print(String(L_KP));
    Serial.print(F(" + "));
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_I()));
    Serial.print(F(" x "));
    Serial.print(String(L_KI));
    Serial.print(F(" + "));
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_D()));
    Serial.print(F(" x "));
    Serial.println(String(L_KD));
    Serial.print(F("[R]: "));
    //Serial.print(String(motor_controllers[MOTOR_RIGHT].getSpeed()));
    Serial.print(F(" = "));
    //Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_P()));
    Serial.print(F(" x "));
    Serial.print(String(R_KP));
    Serial.print(F(" + "));
    //Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_I()));
    Serial.print(F(" x "));
    Serial.print(String(R_KI));
    Serial.print(F(" + "));
    //Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_D()));
    Serial.print(F(" x "));
    Serial.println(String(R_KD));
  }
}

int split(String data, char delimiter, String *dst)//dstは参照引き渡し
{
  //Serial.println("#   split");// デバッグ用確認
  int index = 0;
  int arraySize = (sizeof(data) / sizeof((data)[0]));
  int datalength = data.length();
  for (int i = 0; i < datalength; i++)
  {
    char tmp = data.charAt(i);
    if (tmp == delimiter)
    {
      index++;
      if (index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}

void motor_direct_instructions(int left, int right  ) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println(F("#   motor_direct_instructions"));//確認用
  //motor_controllers[0].servo_.writeMicroseconds(left);
  //motor_controllers[1].servo_.writeMicroseconds(right);

    unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
    FloatToUC(left , 2, frame);
    FloatToUC(right, 6, frame);
    write_bldc(frame);        
}
void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX]  )
{
  //Serial.println(F("#   rc_mode"));//確認用
  digitalWrite(LED_BUILTIN, LOW); // RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  if((rcTime[0] < CUGO_PROPO_MAX_A && rcTime[0] > CUGO_PROPO_MIN_A) && (rcTime[2] < CUGO_PROPO_MAX_C && rcTime[2] > CUGO_PROPO_MIN_C) ) {
    motor_direct_instructions(rcTime[0], rcTime[2]);
    //Serial.println("input cmd:" + String(rcTime[0]) + ", " + String(rcTime[2]));
  }
}

void stop_motor_immediately( )
{
  //Serial.println(F("#   stop_motor_immediately"));//確認用
  //motor_controllers[0].setTargetRpm(0.0);
  //motor_controllers[1].setTargetRpm(0.0);
  //motor_direct_instructions(1500, 1500);
  unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
  FloatToUC(0 , 2, frame);
  FloatToUC(0, 6, frame);
  //write_bldc(frame);        

}

void reset_pid_gain( )
{
  //Serial.println(F("#   reset_pid_gain"));//確認用
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    //motor_controllers[i].reset_PID_param();
  }
}

void set_button_cmd()
{
  //Serial.println(F("#   set_button_cmd"));//確認用
  button_push_count = 0;
  button_enable = 0;
}

void go_backward(float distance, float max_velocity)
{

  if (cmd_init == false)
  {
    //Serial.println(F("#   go_backward"));//確認用
    if (init_current_cmd >= CMD_SIZE - 1)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);
    }

    // 初回起動時の処理
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_count(distance);
    //Serial.print(F("target_count_L/R: "));
    //Serial.print(String(-target_count_L));
    //Serial.print(F(", "));
    //Serial.println(String(-target_count_R));
    float velocity = 0.0;
    if (max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
    } else {
      velocity = max_velocity;
    }
    set_arduino_cmd_matrix(-target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, -velocity);
    if (ARDUINO_MODE == runMode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      Serial.print(String(distance));
      Serial.print(F("m　後ろに進む"));
      Serial.print(F("(上限速度："));
      Serial.print(String(velocity));
      Serial.println(F("rpm )"));
    }
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理
  }
}
void sagaru(float distance)
{
  go_backward(distance, EXCEPTION_NO);
}
void sagaru(float distance, float max_velocity)
{
  go_backward(distance, max_velocity);
}

void turn_clockwise(float degree, float max_velocity)
{
  if (cmd_init == false)
  {
    //Serial.println(F("#   turn_clockwise"));//確認用
    if (init_current_cmd >= CMD_SIZE - 1)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);
    }

    // 初回起動時の処理
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_rotate(degree);
    //Serial.println("target_count_L/R: " + String(target_count_L) + ", " + String(target_count_R));
    float velocity = 0.0;
    if (max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
    } else {
      velocity = max_velocity;
    }
    set_arduino_cmd_matrix(target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, -velocity);
    if (ARDUINO_MODE == runMode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      Serial.print(String(degree));
      Serial.print(F("度　右回り"));
      Serial.print(F("(上限速度："));
      Serial.print(String(velocity));
      Serial.println(F("rpm )"));
    }
    init_current_cmd++;
  }
  else
  {
    // 通常ループ時の処理
  }
}

void migimawari(float degree)
{
  turn_clockwise(degree, EXCEPTION_NO);
}
void migimawari(float degree, float max_velocity)
{
  turn_clockwise(degree, max_velocity);
}
void migimawari90()
{
  migimawari(90, EXCEPTION_NO);
}
void migimawari90(float max_velocity)
{
  migimawari(90, max_velocity);
}
void migimawari45()
{
  migimawari(45, EXCEPTION_NO);
}
void migimawari45(float max_velocity)
{
  migimawari(45, max_velocity);
}
void migimawari180()
{
  migimawari(180, EXCEPTION_NO);
}
void migimawari180(float max_velocity)
{
  migimawari(180, max_velocity);
}

void go_forward(float distance, float max_velocity)
{
  if (cmd_init == false)
  {
    //Serial.println(F("#   go_forward"));//確認用
    // 初回起動時の処理
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_count(distance);
    //Serial.println("target_count_L/R: " + String(target_count_L) + ", " + String(target_count_R));
    float velocity = 0.0;
    if (max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
    } else {
      velocity = max_velocity;
    }
    set_arduino_cmd_matrix(target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, velocity);
    //Serial.println("matrix_target_count_L/R: " + String(arduino_count_cmd_matrix[init_current_cmd][0]) + ", " + String(arduino_count_cmd_matrix[init_current_cmd][0]));
    if (ARDUINO_MODE == runMode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      Serial.print(String(distance));
      Serial.print(F("m 前に進む "));
      //Serial.print(String(target_count_L));
      Serial.print(F("(上限速度："));
      Serial.print(String(velocity));
      Serial.println(F("rpm)"));

    }
    init_current_cmd++;
  }
  else
  {
    // 通常ループ時の処理
  }
}

void susumu(float distance)
{
  go_forward(distance, EXCEPTION_NO);
}
void susumu(float distance, float max_velocity) {
  go_forward(distance, max_velocity);

}
void turn_counter_clockwise(float degree, float max_velocity)
{
  if (cmd_init == false)
  {
    //Serial.println(F("#   turn_counter_clockwise"));//確認用
    if (init_current_cmd >= CMD_SIZE - 1)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);
    }

    // 初回起動時の処理
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_rotate(degree);
    //Serial.print(F("target_count_L/R: "));
    //Serial.print(String(-target_count_L));
    //Serial.print(F(", "));
    //Serial.println(String(-target_count_R));

    float velocity = 0.0;
    if (max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
    } else {
      velocity = max_velocity;
    }
    set_arduino_cmd_matrix(-target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, velocity);
    if (ARDUINO_MODE == runMode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      Serial.print(String(degree));
      Serial.print(F("度　左回り "));
      Serial.print(F("(上限速度："));
      Serial.print(String(velocity));
      Serial.println(F("rpm)"));
    }
    init_current_cmd++;
  }
  else
  {
    // 通常ループ時の処理
  }
}

void hidarimawari(float degree)
{
  turn_counter_clockwise(degree, EXCEPTION_NO);
}
void hidarimawari(float degree, float max_velocity)
{
  turn_counter_clockwise(degree, max_velocity);
}
void hidarimawari90()
{
  hidarimawari(90, EXCEPTION_NO);
}
void hidarimawari90(float max_velocity)
{
  hidarimawari(90, max_velocity);
}
void hidarimawari45()
{
  hidarimawari(45, EXCEPTION_NO);
}
void hidarimawari45(float max_velocity)
{
  hidarimawari(45, max_velocity);
}
void hidarimawari180()
{
  hidarimawari(180, EXCEPTION_NO);
}
void hidarimawari180(float max_velocity)
{
  hidarimawari(180, max_velocity);
}

void reset_arduino_mode_flags()
{
  //Serial.println(F("#   reset_arduino_mode_flags"));//確認用
  cmd_init = false;
  current_cmd = 0;
  target_count_L = 0;
  target_count_R = 0;
  cmd_exec = false;
  count_done = false;
  wait_done  = false;
  button_done = false;
  spi_done = false;
  target_wait_time = 0;
  button_push_count = 0;
  button_enable = 0;
  init_current_cmd = 0;
  init_ARDUINO_CMD();
}

void set_go_forward_cmd( )
{
  //Serial.println(F("#   set_go_forward_cmd"));//確認用
  count_prev_L = __encorderL;
  target_count_L = __encorderL + arduino_count_cmd_matrix[current_cmd][0];//★

  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String(motor_controllers[MOTOR_LEFT].getCount()) + " + " + String(arduino_count_cmd_matrix[current_cmd][0]));
  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String( __encorderL) + " + " + String(arduino_count_cmd_matrix[current_cmd][0]));
  //while(1);
  if (arduino_count_cmd_matrix[current_cmd][0] >= 0) {
    cmd_L_back = false;
  } else {
    cmd_L_back = true;
  }
  count_prev_R = __encorderR;
  target_count_R = __encorderR + arduino_count_cmd_matrix[current_cmd][1];//★
  if (arduino_count_cmd_matrix[current_cmd][1] >= 0) {
    cmd_R_back = false;
  } else {
    cmd_R_back = true;
  }
}

void view_flags()
{
  //Serial.println(F("#   view_flags"));//確認用
  Serial.println(F(""));
  Serial.println(F("FLAGS"));
  Serial.print(F("cmd_init: "));
  Serial.println(String(cmd_init));
  Serial.print(F("current_cmd: "));
  Serial.println(String(current_cmd));
  Serial.print(F("target_count_L: "));
  Serial.println(String(target_count_L));
  Serial.print(F("target_count_R: "));
  Serial.println(String(target_count_R));
  Serial.print(F("cmd_exec: "));
  Serial.println(String(cmd_exec));
  Serial.print(F("count_done: "));
  Serial.println(String(count_done));
  Serial.print(F("wait_done: "));
  Serial.println(String(wait_done));
  Serial.print(F("button_done: "));
  Serial.println(String(button_done));
  Serial.print(F("spi_done: "));
  Serial.println(String(spi_done));
  Serial.print(F("target_wait_time: "));
  Serial.println(String(target_wait_time));
  Serial.print(F("button_push_count: "));
  Serial.println(String(button_push_count));
  Serial.print(F("button_enable: "));
  Serial.println(String(button_enable));
  Serial.println(F(""));
}

void check_achievement_spi_cmd()
{
  //Serial.println(F("#   check_achievement_spi_cmd"));//確認用
  send_spi(arduino_flag_cmd_matrix[current_cmd][1]);
  spi_done = true;
  Serial.print(F("###"));
  if (current_cmd < 9)
    Serial.print(F("0"));

  Serial.print(String(current_cmd + 1));
  Serial.println(F("番目のコマンド：終了  ###"));
}

void cmd_end( )
{

  if (cmd_init == false)
  {
    //Serial.println(F("#   cmd_end"));//確認用
    // 初回起動時の処理
    //Serial.println("CMD_SIZE: " + String(CMD_SIZE));
    if (init_current_cmd >= CMD_SIZE)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);
    }

    // 初回起動時の処理をここで無効化
    //reset_pid_gain( );
    //Serial.println(runMode);

    if (ARDUINO_MODE == runMode) {
      Serial.println(F("###   コマンド準備完了    ###"));
      Serial.println(F("##########################"));
      Serial.println(F("###   コマンド実行開始    ###"));
    }
    cmd_init = true;   // 最初の一回だけ。全部のコマンドが終了したとき、最初のコマンドに戻すときにリセット。
  }
  else
  {
    // 通常ループ時の処理
    // すべてのコマンドが終了しているか判定
  }
}



void cmd_manager_flags_init( )
{
  //Serial.println(F("#   cmd_manager_flags_init"));
  // これからコマンドを実行するときの処理

  reset_pid_gain( );
  cmd_exec = true;
  count_done = false;
  wait_done = false;
  button_done = false;
  spi_done = false;
  cmd_L_back = false;
  cmd_R_back = false;

  //while(1);
  if (init_current_cmd >= CMD_SIZE - 1)
  {
    //Serial.print(F("#   init_current_cmd: "));
    Serial.println(String(init_current_cmd));
    Serial.println(F("コマンドの上限数以上にコマンドを設定しています。強制終了。"));
    while (1);
  }
  if (arduino_count_cmd_matrix[current_cmd][0] == EXCEPTION_NO && arduino_count_cmd_matrix[current_cmd][1] == EXCEPTION_NO)
    count_done = true;

  if (arduino_flag_cmd_matrix[current_cmd][0] == EXCEPTION_NO)
    wait_done = true;

  if (arduino_flag_cmd_matrix[current_cmd][1] != 255)
    button_done = true;

  if (arduino_flag_cmd_matrix[current_cmd][1] < 1 || 7 < arduino_flag_cmd_matrix[current_cmd][1])
    spi_done = true;

  //Serial.print(F("Current cmd: "));
  //Serial.println(String(current_cmd));
  //view_flags();
  //Serial.println(F(""));

  // ここに入ったら終了。
  if (count_done == true && wait_done == true && button_done == true && spi_done == true)
  {
    //Serial.println(F("すべてのコマンド実行完了。または、初期化のままでコマンド入力できていない。"));
    //view_flags();
    end_arduino_mode = true;
  } else {

    Serial.print(F("###"));
    if (current_cmd < 9)
      Serial.print(F("0"));

    Serial.print(String(current_cmd + 1));
    Serial.print(F("番目のコマンド：開始  "));
    
      float degree = 0 ;
      float distance = 0 ;
      if(arduino_flag_cmd_matrix[current_cmd][0] != EXCEPTION_NO)//実際の待ち時間確認
      {
      Serial.print(String(arduino_flag_cmd_matrix[current_cmd][0]));
      Serial.print(F("ミリ秒　待つ"));
      }else if(arduino_flag_cmd_matrix[current_cmd][1] == 255)//ボタン押し待ち
      {
      Serial.print(F("ボタン　押し待ち"));
      }else if(arduino_flag_cmd_matrix[current_cmd][2] > 0 && arduino_flag_cmd_matrix[current_cmd][3] > 0)//前進
      {
      distance =  (arduino_count_cmd_matrix[current_cmd][0]) * (( 2 * wheel_radius_l * PI) / encoder_resolution );
      Serial.print(String(abs(distance)));
      Serial.print(F(" m　前進"));
      }else if(arduino_flag_cmd_matrix[current_cmd][2] < 0 && arduino_flag_cmd_matrix[current_cmd][3] > 0)//左回り
      {
      degree = (2 * wheel_radius_l * PI * arduino_count_cmd_matrix[current_cmd][0] * 360) / (encoder_resolution * tread * PI);
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　左回り"));
      }else if(arduino_flag_cmd_matrix[current_cmd][2] > 0 && arduino_flag_cmd_matrix[current_cmd][3] < 0)//右回り
      {
      degree = (2 * wheel_radius_l * PI * arduino_count_cmd_matrix[current_cmd][0] * 360) / (encoder_resolution * tread * PI);
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　右回り"));
      }else if(arduino_flag_cmd_matrix[current_cmd][2] < 0 && arduino_flag_cmd_matrix[current_cmd][3] < 0)//後進
      {
      distance =  (arduino_count_cmd_matrix[current_cmd][0]) * (( 2 * wheel_radius_l * PI) / encoder_resolution );
      Serial.print(String(abs(distance)));
      Serial.print(F(" m　後進"));
      }else{
      Serial.print(F("不明なコマンド"));
      }

    Serial.println(F("###"));

  }
  // ここに入ったら誤作動
  if (count_done == false && wait_done == false || count_done == false && button_done == false || \
      count_done == false && wait_done == false || wait_done == false && button_done == false || \
      wait_done == false && spi_done == false || button_done == false && spi_done == false)
  {
    Serial.println(F("## BAD CASE!! ##"));
    view_flags();
    view_arduino_cmd_matrix();
    Serial.println(F("複数コマンド入力。入力関数に不備があるか、コマンドを上書きしている可能性あり。"));
    stop_motor_immediately( );

    while (1);
  }
}

void check_achievement_go_forward_cmd( )// motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  bool L_done = false;
  bool R_done = false;
  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String( __encorderL) + " + " + String(arduino_count_cmd_matrix[current_cmd][0]));
  //Serial.println();
  // L側目標達成チェック
  if (cmd_L_back == false) {
    if (target_count_L <= __encorderL){
      L_done = true;
      //Serial.println(F("#   L_done"));
    }
  }else{
    if (target_count_L >= __encorderL){
      L_done = true;
      //Serial.println(F("#   L_done"));
    }
  }

  // R側目標達成チェック
  if (cmd_R_back == false) {
    if (target_count_R <= __encorderR){
      R_done = true;
      //Serial.println(F("#   R_done"));
    }
  }else{
    if (target_count_R >= __encorderR){
      R_done = true;
      //Serial.println(F("#   R_done"));
    }
  }

  if (L_done == true)
  {
    //motor_controllers[0].setTargetRpm(0);
  }
  if (R_done == true)
  {
    //motor_controllers[1].setTargetRpm(0);
  }

  // L/R達成していたら終了
  if (L_done == true && R_done == true){
    stop_motor_immediately( );
    count_done = true;
    Serial.print(F("###"));
    if (current_cmd < 9)
      Serial.print(F("0"));

    Serial.print(String(current_cmd + 1));
    Serial.print(F("番目のコマンド：終了  "));
    double degree = 0 ;
    double distance = 0 ;
    if (arduino_flag_cmd_matrix[current_cmd][2] > 0 && arduino_flag_cmd_matrix[current_cmd][3] > 0) //前進
    {
      distance =  (__encorderL-count_prev_L) * (( 2 * wheel_radius_l * PI) / encoder_resolution);
      //distance =  * conversion_count_to_distance;//★
      Serial.print(String(fabsf(distance)));
      Serial.print(F(" m　進んだ"));
    } else if (arduino_flag_cmd_matrix[current_cmd][2] < 0 && arduino_flag_cmd_matrix[current_cmd][3] > 0) //左回り
    {
      degree = (2 * wheel_radius_l * PI * (__encorderL-count_prev_L) * 360) / (encoder_resolution * tread * PI);//★
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　左回りに回転した"));
    } else if (arduino_flag_cmd_matrix[current_cmd][2] > 0 && arduino_flag_cmd_matrix[current_cmd][3] < 0) //右回り
    {
      degree = (2 * wheel_radius_l * PI * (__encorderL-count_prev_L) * 360) / (encoder_resolution * tread * PI);//★
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　右回りに回転した"));
    } else if (arduino_flag_cmd_matrix[current_cmd][2] < 0 && arduino_flag_cmd_matrix[current_cmd][3] < 0) //後進
    {
      distance =  (__encorderL-count_prev_L) * (( 2 * wheel_radius_l * PI) / encoder_resolution);//★
      //distance = convert_pulseTodistance(__encorderL);
      Serial.print(String(abs(distance)));
      Serial.print(F(" m 後に進んだ"));
    } else {
      Serial.print(F("不明なコマンド"));
    }
    Serial.println(F("  ###"));

  }
}


void cmd_manager( )
{
  if (cmd_init == false)
  {
    Serial.println(F("##########################"));
    Serial.println(F("###   コマンド準備開始    ###"));
  }
  else
  {
    // 通常ループ時の処理

    // コマンド実行直前処理
    if (cmd_exec == false)
    {
      cmd_manager_flags_init( );
      // 前後進の指示をセット
      if (count_done == false)
      {
        set_go_forward_cmd( );
      }

      // 待機の指示をセット
      if (wait_done == false)
      {
        set_wait_time_cmd();
      }

      // ボタンの指示をセット
      if (button_done == false)
      {
        // ボタンに関しては、終了目標値がないため何もしない。
      }

      // ボタンの指示をセット
      if (spi_done == false)
      {
        // SPIに関しては、終了目標値がないため何もしない。
      }
      //Serial.println(F("### Command Start ###"));
    }

    // コマンド実行中処理
    if (cmd_exec == true) // elseにしないのは、スタートしてから実行したいため
    {
      //Serial.println(F("#   cmd_exec_main"));//確認用
      // コマンドで設定された速度に設定
      //ここで速度制御：setTargetRpm目標速度の設定

      //// PID位置制御の制御値
      float l_count_p;  // P制御値
      float l_count_i;  // I制御値
      float l_count_d;  // D制御値
      float r_count_p;  // P制御値
      float r_count_i;  // I制御値
      float r_count_d;  // D制御値
      if (arduino_flag_cmd_matrix[current_cmd][2] == 0 && arduino_flag_cmd_matrix[current_cmd][3] == 0)
      {
      unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
      FloatToUC(0 * 1.005, 2, frame);
      FloatToUC(0, 6, frame);
      write_bldc(frame);        
      } else {
        // 各制御値の計算
        //l_count_p = arduino_count_cmd_matrix[current_cmd][0] - __encorderL;
        l_count_p = target_count_L - __encorderL;
        l_count_i = l_count_prev_i_ + l_count_p;
        l_count_d = l_count_p - l_count_prev_p_;
        //r_count_p = arduino_count_cmd_matrix[current_cmd][1] - __encorderR;
        r_count_p = target_count_R - __encorderR;
        r_count_i = r_count_prev_i_ + r_count_p;
        r_count_d = r_count_p - r_count_prev_p_;

        l_count_i = min( max(l_count_i, -L_MAX_COUNT_I), L_MAX_COUNT_I);
        r_count_i = min( max(r_count_i, -R_MAX_COUNT_I), R_MAX_COUNT_I);
        // PID制御
        l_count_gain = l_count_p * L_COUNT_KP + l_count_i * L_COUNT_KI + l_count_d * L_COUNT_KD;
        r_count_gain = r_count_p * R_COUNT_KP + r_count_i * R_COUNT_KI + r_count_d * R_COUNT_KD;
        // prev_ 更新
        l_count_prev_p_ = l_count_p;
        l_count_prev_i_ = l_count_i;
        r_count_prev_p_ = r_count_p;
        r_count_prev_i_ = r_count_i;

        l_count_gain = min( max(l_count_gain, -MAX_MOTOR_RPM), MAX_MOTOR_RPM); //モーターの速度上限
        r_count_gain = min( max(r_count_gain, -MAX_MOTOR_RPM), MAX_MOTOR_RPM); //モーターの速度上限
        l_count_gain = min( max(l_count_gain, -fabsf(arduino_flag_cmd_matrix[current_cmd][2])), fabsf(arduino_flag_cmd_matrix[current_cmd][2])); //ユーザ設定の速度上限
        r_count_gain = min( max(r_count_gain, -fabsf(arduino_flag_cmd_matrix[current_cmd][3])), fabsf(arduino_flag_cmd_matrix[current_cmd][3])); //ユーザ設定の速度上限


        //位置制御
        //motor_controllers[MOTOR_LEFT].setTargetRpm(l_count_gain);
        //motor_controllers[MOTOR_RIGHT].setTargetRpm(r_count_gain);
        //Serial.print(F("gain:l/r "));
        //Serial.print(String(l_count_gain));
        //Serial.print(F(","));
        //Serial.println(String(r_count_gain));
      unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
      FloatToUC(l_count_gain * 1.005, 2, frame);
      FloatToUC(r_count_gain, 6, frame);
      write_bldc(frame);        

      }

      // 成功条件の確認
      // if conuntの成功条件
      if (count_done == false)
        check_achievement_go_forward_cmd( );

      if (wait_done == false)
        check_achievement_wait_time_cmd( );

      if (button_done == false)
        check_achievement_button_cmd( );

      if (spi_done == false)
        check_achievement_spi_cmd();


      // if waitの成功条件
      // if buttonの成功条件

      if (count_done == true && wait_done == true && button_done == true)
      {
        // モータを止める
        cmd_exec = false;
        //初期化
        l_count_prev_i_ = 0;
        l_count_prev_p_ = 0;
        r_count_prev_i_ = 0;
        r_count_prev_p_ = 0;
        l_count_gain = 0;
        r_count_gain = 0;

        current_cmd++;

        if (end_arduino_mode == true)
        {
          Serial.println(F("###   コマンド実行終了    ###"));
          Serial.println(F("##########################"));
          reset_arduino_mode_flags();
          end_arduino_mode = false;
          runMode = RC_MODE;
          reset_arduino_mode_flags();
          Serial.println(F("##########################"));
          Serial.println(F("###   モード:RC_MODE    ###"));
          Serial.println(F("##########################"));

        }
      }
    }
  }
}

void check_achievement_button_cmd( )
{
  if (digitalRead(CMD_BUTTON_PIN) == 0)
  {
    button_push_count++;
  } else {
    button_push_count = 0;
  }

  if (button_push_count >= 5) // 実測で50ms以上長いと小刻みに押したとき反応しないと感じてしまう。
  {
    stop_motor_immediately( );
    button_done = true;
    Serial.print(F("###"));
    if (current_cmd < 9)
      Serial.print(F("0"));

    Serial.print(String(current_cmd + 1));
    Serial.println(F("番目のコマンド：終了  ボタン　押された  ###"));
  }
}

void init_display()
{
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("#######################################"));
  Serial.println(F("#######################################"));
  Serial.println(F("#                                     #"));
  Serial.println(F("#   ####    ##  ##    ####     ####   #"));
  Serial.println(F("#  ##  ##   ##  ##   ##  ##   ##  ##  #"));
  Serial.println(F("#  ##       ##  ##   ##       ##  ##  #"));
  Serial.println(F("#  ##       ##  ##   ## ###   ##  ##  #"));
  Serial.println(F("#  ##       ##  ##   ##  ##   ##  ##  #"));
  Serial.println(F("#  ##  ##   ##  ##   ##  ##   ##  ##  #"));
  Serial.println(F("#   ####     ####     ####     ####   #"));
  Serial.println(F("#                                     #"));
  Serial.println(F("#######################################"));
  Serial.println(F("#######################################"));
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("##################################"));
  Serial.println(F("###    CugoAruduinoKit起動     ###"));
  Serial.println(F("##################################"));

}



void job_100ms( )//100msごとに必要な情報を表示
{
  display_speed(  ENCODER_DISPLAY);
  display_target_rpm(  ENCODER_DISPLAY);
  display_PID(  PID_CONTROLL_DISPLAY);
  display_failsafe(FAIL_SAFE_DISPLAY);
}

void job_1000ms()//1000msごとに必要な情報があれば表示
{
  display_nothing();
}

void display_detail( )
{
  if (current_time - prev_time_100ms > 100000)
  {
    job_100ms( );
    prev_time_100ms = current_time;
  }

  if (current_time - prev_time_1000ms > 1000000)
  {
    job_1000ms();
    prev_time_1000ms = current_time;
  }
}




//モード切替関連
  int cugoRunMode = RC_MODE;
  int cugoOldRunMode = ARDUINO_MODE;

//モータ制御関連
  bool cugo_direction_L; //true:forward false:backward
  bool cugo_direction_R; //true:forward false:backward
  long int cugo_target_count_L = 0;
  long int cugo_target_count_R = 0;
  
//ボタン・プロポ入力関連
  bool cugo_button_check = false;
  int cugo_button_count =0;  
  int CUGO_OLD_PWM_IN_PIN0_VALUE;   // プロポスティック入力値(L)
  int CUGO_OLD_PWM_IN_PIN1_VALUE;   // プロポスティック入力値(MODE)
  int CUGO_OLD_PWM_IN_PIN2_VALUE;   // プロポスティック入力値(R)
  int CUGO_OLD_CMD_BUTTON_VALUE = 0; 
  //volatile unsigned long cugoUpTime[CUGO_PWM_IN_MAX];
  //volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX];
  //volatile unsigned long cugo_time[CUGO_PWM_IN_MAX];
  volatile unsigned long long cugoButtonStartTime = 0;
//オドメトリ関連
  float cugo_odometer_theta = 0.0;
  float cugo_odometer_degree = 0.0;
  float cugo_odometer_x = 0.0;
  float cugo_odometer_y = 0.0;
  long int cugo_count_prev_L = 0;
  long int cugo_count_prev_R = 0;
  long int cugo_odometer_count_theta =0;
  unsigned long long int calc_odometer_time = 0;

//BLDC

  const float DIST_PERROT = 242.4;  //一回転何[mm]すすむか
  const long int PULSE_PERROT = 360;     //一回転何[パルス]か
  const long int WIDTH_BLDC = 350;       //一回転何[パルス]か
  const long int index_tofreq[3] = { 10, 50, 100 };




  //realtime
  volatile long int id = 0;

  volatile float accelerationR = 0;
  volatile float accelerationL = 0;
  float rpm_current_R = 0;
  float rpm_current_L = 0;
  float target_rpmR = 0;
  float target_rpmL = 0;
  volatile short _encorderR = 0, _encorderL = 0;
  volatile long __encorderR = 0, __encorderL = 0;



//各種関数
//初期化関数
void cugo_init(){
  //Serial.print(F("CPU Frequency = "));
  //Serial.print(F_CPU / 1000000);
  //Serial.println(F(" MHz"));

  //Serial.begin(115200, SERIAL_8N1);//PCとの通信
  //Serial1.begin(115200, SERIAL_8N1);//BLDCとの通信

  //pinMode(CUGO_CMD_BUTTON_PIN, INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(CUGO_CMD_BUTTON_PIN), cugo_button_interrupt, CHANGE); 
  //CBD = new c_BLDC_Driver();
  //set_feedback(2, 0b10000001);//freq{0:10[hz] 1:50[hz] 2:100[hz]} kindof_data{0b1:Mode 0b10:CMD_RPM 0b100:CurrentRPM 0b1000:AveCurrentRPM 0b10000000:EncorderData}
    /*
      Bit Description Default Value
      0 Mode 0
      1 CMD RPM 0
      2 Current RPM 1
      3 Ave Current RPM 0
      4 NU -
      5 NU -
      6 NU -
      7 Encoder Data 1
      */
  //マイクロ秒単位の間隔
  //delay(1000);  
  //if (ITimer0.attachInterruptInterval( FEEDBACK_DUTATION * 1000, TimerHandler0)) {
  //  Serial.print(F("Starting ITimer0 OK, millis() = "));
  //  Serial.println(millis());
  //} else {
  //  Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  //}
  //delay(1000);

  //setControlMode(1);
  //Serial.println("Set CMD Mode.");
  //delay(1000);

  //cugo_init_display();
  //cugo_init_KOPROPO(CUGO_OLD_PWM_IN_PIN0_VALUE,CUGO_OLD_PWM_IN_PIN1_VALUE,CUGO_OLD_PWM_IN_PIN2_VALUE);
  //cugo_reset_button_times();
  //cugo_reset_odometer();
  //pinMode(CUGO_PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力 入力割り込みpinを使用 内蔵プルアップ有効
  //pinMode(CUGO_PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力 内蔵プルアップ有効
  //pinMode(CUGO_PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力 入力割り込みpinを使用 内蔵プルアップ有効
  //pinMode(CUGO_PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力 内蔵プルアップ有効    
}

bool TimerHandler0(struct repeating_timer *t) {
   BackGround();
  return true;
}

void cugo_button_interrupt(){
  if(digitalRead(CUGO_CMD_BUTTON_PIN) == HIGH){
  cugo_button_count++;
  cugo_button_check = true;
  cugoButtonStartTime = micros();
  }
  if(digitalRead(CUGO_CMD_BUTTON_PIN) == LOW){
  cugoButtonStartTime = 0;
  cugo_button_check = false;
  }

}

//モード切り替わり確認
void cugo_check_mode_change( )
{
  //noInterrupts();      //割り込み停止
  //cugoRcTime[0] = cugo_time[0];
  //cugoRcTime[1] = cugo_time[1];
  //cugoRcTime[2] = cugo_time[2];
  //cugoButtonTime = cugo_time[3];
  //interrupts();     //割り込み開始
  /*
  if ((cugoRunMode == CUGO_SELF_DRIVE_MODE) && (cugo_old_runmode == CUGO_RC_MODE))
  {
    Serial.println(F("### MODE:CUGO_SELF_DRIVE_MODE ###"));
    digitalWrite(LED_BUILTIN, HIGH);  // CUGO_SELF_DRIVE_MODEでLED点灯           
    cugo_old_runmode = CUGO_SELF_DRIVE_MODE;
    cugo_reset_pid_gain(  );
    cugo_motor_direct_instructions(1500, 1500 ); //直接停止命令を出す
    cugo_wait(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  }
  if(cugoRunMode == CUGO_RC_MODE && cugo_old_runmode == CUGO_SELF_DRIVE_MODE)
  {
    Serial.println(F("###   MODE:CUGO_RC_MODE    ###"));
    digitalWrite(LED_BUILTIN, LOW); // CUGO_RC_MODEでLED消灯
    cugo_old_runmode = CUGO_RC_MODE;            
    cugo_reset_pid_gain(  );
    cugo_motor_direct_instructions(1500, 1500 ); //直接停止命令を出す
    cugo_wait(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  } 
  */
                      
}

void cugo_wait(unsigned long long int  wait_ms){
  //約70分まで計測可能
  //例１時間待機したい場合：cugo_wait(120UL*60UL*1000UL);
  unsigned long long int cugo_target_wait_time = wait_ms*1000ull;
  unsigned long long int MAX_MICROS = 4294967295ull; // micros() 関数で計測する最大値
  unsigned long long int startMicros = 0; // 計測開始時刻
  unsigned long long int elapsedMicros = 0; // 経過時間（マイクロ秒単位）
  unsigned long long int currentMicros = micros();

  if(cugo_target_wait_time < MAX_MICROS){
    while(elapsedMicros < cugo_target_wait_time){
    
      if (startMicros == 0) {
        startMicros = micros();  // 計測開始時刻が初期化されていない場合、初期化する
      }
      currentMicros = micros();//時刻を取得
  
      // 経過時間を計算する
      if (currentMicros >= startMicros) {
        elapsedMicros = currentMicros - startMicros;
      } else {
      // オーバーフローが発生した場合
      elapsedMicros = (MAX_MICROS - startMicros) + currentMicros + 1;
      }
    }
  }else{
    Serial.println(F("##WARNING::cugo_waitの計測可能時間を超えています。##"));
  }
  
}

void cugo_long_wait(unsigned long long int wait_seconds){
  //例 24時間計測したい場合：cugo_wait(24UL*60UL*60UL);

  unsigned long long int cugo_target_wait_time = wait_seconds*1000ull;
  unsigned long long int MAX_MILLIS = 4294967295ull; // millis() 関数で計測する最大値
  unsigned long long int startMillis = 0; // 計測開始時刻
  unsigned long long int elapsedMillis = 0; // 経過時間（マイクロ秒単位）
  unsigned long long int currentMillis = millis();

  if(cugo_target_wait_time < MAX_MILLIS){
  
    while(elapsedMillis < cugo_target_wait_time){    
      if (startMillis == 0) {
        startMillis = millis();  // 計測開始時刻が初期化されていない場合、初期化する
      }
      currentMillis = millis();//現在時刻を取得
  
      // 経過時間を計算する
      if (currentMillis >= startMillis) {
        elapsedMillis = currentMillis - startMillis;
      } else {
        // オーバーフローが発生した場合
        elapsedMillis = (MAX_MILLIS - startMillis) + currentMillis + 1;
      }
    }
  }else{
  Serial.println(F("##WARNING::cugo_long_waitの計測可能時間を超えています。##"));
  }

}

void cugo_move_pid(float target_rpm,bool use_pid ){
  cugo_reset_pid_gain(  );
  cugo_start_odometer();      
  Encorder_reset();    

  float l_tagat_rpm = 0;
  float r_tagat_rpm = 0;

  //PID制御値
  float l_count_p =0 ;  
  float l_count_i =0 ;      
  float l_count_d =0 ;  
  float r_count_p =0 ;  
  float r_count_i =0 ;  
  float r_count_d =0 ;  
  // PID位置制御のデータ格納
  float l_count_prev_i_ =0 ;
  float l_count_prev_p_ =0 ;
  //float l_count_prev_p_ = (cugo_target_count_L -      /*/*.getCount()*/*/)/10000.0;
  float r_count_prev_i_ =0 ;
  float r_count_prev_p_ =0 ;
  //float r_count_prev_p_ = (cugo_target_count_R -      /*/*.getCount()*/*/)/10000.0 ;
  float l_count_gain =0 ;
  float r_count_gain =0 ;

  if(target_rpm <= 0){
    Serial.println(F("##WARNING::目標速度が0以下のため進みません##"));          
  }else if(cugo_target_count_L == 0 && cugo_target_count_R == 0){
    Serial.println(F("##WARNING::目標距離が左右ともに0のため進みません##"));          
  }else{
    
    if(cugo_target_count_L >= 0){
      cugo_direction_L = true;
    }else{
      cugo_direction_L = false;
    }
    if(cugo_target_count_R >= 0){
      cugo_direction_R = true;
    }else{
      cugo_direction_R = false;
    }

    if(!use_pid){
      if(cugo_direction_L){

      }else{
      }
      if(cugo_direction_R){
      }else{
      }
    }
    //★
    //if(abs(     /*.getTargetRpm()*/)>180 || abs(     /*.getTargetRpm()*/) > 180){
    //  Serial.println(F("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##"));          
    //}

       
    //cugo_test時確認用
    
    //Serial.println("start_count l:r:"+" ,");  
    Serial.println("target_rpm l:r:" + String(rpm_current_L)+" ,"+ String(rpm_current_R));  
    Serial.println("target_count l:r:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));    
    //*/

    //    while(!cugo_check_count_achievement(CUGO_MOTOR_LEFT ) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT )){  
    while(!cugo_check_count_achievement(CUGO_MOTOR_LEFT) ){  
      if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
      {
        //停止しているだけの時
             //.setTargetRpm(0);
             //.setTargetRpm(0);
      } else{
        if(use_pid){
          // 各制御値の計算
          //★
          l_count_p = cugo_target_count_L - __encorderL;
          l_count_i = l_count_prev_i_ + l_count_p;
          l_count_d = l_count_p - l_count_prev_p_;
          //★
          r_count_p = cugo_target_count_R - __encorderR;
          r_count_i = r_count_prev_i_ + r_count_p;
          r_count_d = r_count_p - r_count_prev_p_;

          l_count_i = min( max(l_count_i,-CUGO_L_MAX_COUNT_I),CUGO_L_MAX_COUNT_I);        
          r_count_i = min( max(r_count_i,-CUGO_R_MAX_COUNT_I),CUGO_R_MAX_COUNT_I);
          // PID制御
          l_count_gain = (l_count_p * CUGO_L_COUNT_KP + l_count_i * CUGO_L_COUNT_KI + l_count_d * CUGO_L_COUNT_KD);  
          r_count_gain = (r_count_p * CUGO_R_COUNT_KP + r_count_i * CUGO_R_COUNT_KI + r_count_d * CUGO_R_COUNT_KD);  
          // prev_ 更新
          l_count_prev_p_ = l_count_p;
          l_count_prev_i_ = l_count_i;
          r_count_prev_p_ = r_count_p;
          r_count_prev_i_ = r_count_i;
          l_count_gain = min( max(l_count_gain,-CUGO_MAX_MOTOR_RPM),CUGO_MAX_MOTOR_RPM);//モーターの速度上限        
          r_count_gain = min( max(r_count_gain,-CUGO_MAX_MOTOR_RPM),CUGO_MAX_MOTOR_RPM);//モーターの速度上限             
          l_count_gain = min( max(l_count_gain,-fabsf(target_rpm)),fabsf(target_rpm));//ユーザ設定の速度上限        
          r_count_gain = min( max(r_count_gain,-fabsf(target_rpm)),fabsf(target_rpm));//ユーザ設定の速度上限  

          //rpm_current_L = l_count_gain;
          //rpm_current_R = r_count_gain;

          //位置制御
          if(!cugo_check_count_achievement(CUGO_MOTOR_LEFT )){
            rpm_current_L = l_count_gain;
          } 
          if(!cugo_check_count_achievement(CUGO_MOTOR_RIGHT )){
            rpm_current_R = r_count_gain;
          }
          Serial.println("target_rpm l:r:" + String(rpm_current_L)+" ,"+ String(rpm_current_R));  
        }
      }
      unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
      FloatToUC(rpm_current_L * 1.005, 2, frame);
      FloatToUC(rpm_current_R, 6, frame);
      write_bldc(frame);        

      cugo_wait(10);
      cugo_calc_odometer(  );
    }
        
    //cugo_test時確認用
    
    //Serial.println(F("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));      
    //Serial.println(F("result_count l:r:" + String(     .getCount())+" ,"+ String(     .getCount()));
    //Serial.println(F("==========="));
    
  }
  cugo_stop(  );   
  //出力
  unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
      FloatToUC(0, 2, frame);
      FloatToUC(0, 6, frame);
      write_bldc(frame);        
  Encorder_reset();

  cugo_reset_pid_gain(  );                 
}

  //前進制御＆回転制御
void cugo_move_forward(float target_distance ){
  cugo_calc_necessary_count(target_distance );
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM,true );
  }
void cugo_move_forward(float target_distance,float target_rpm ){
  cugo_calc_necessary_count(target_distance );
  cugo_move_pid(target_rpm,true );
  }
void cugo_move_forward_raw(float target_distance,float target_rpm ){
  cugo_calc_necessary_count(target_distance );
  cugo_move_pid(target_rpm,false );
  }
void cugo_turn_clockwise(float target_degree ){
  cugo_calc_necessary_rotate(target_degree );  
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM,true );  
  }
void cugo_turn_clockwise(float target_degree,float target_rpm ){
  cugo_calc_necessary_rotate(target_degree );
  cugo_move_pid(target_rpm,true );
  }
void cugo_turn_clockwise_raw(float target_degree,float target_rpm ){
  cugo_calc_necessary_rotate(target_degree );
  cugo_move_pid(target_rpm,false );
  }
void cugo_turn_counterclockwise(float target_degree ){
  cugo_calc_necessary_rotate(-target_degree );
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM,true );    
  }
void cugo_turn_counterclockwise(float target_degree,float target_rpm ){
  cugo_calc_necessary_rotate(-target_degree );
  cugo_move_pid(target_rpm,true );  
  }
void cugo_turn_counterclockwise_raw(float target_degree,float target_rpm ){
  cugo_calc_necessary_rotate(-target_degree );
  cugo_move_pid(target_rpm,false );  
  }
  //円軌道での移動命令
void cugo_curve_theta_raw(float target_radius,float target_theta,float target_rpm ){
  cugo_reset_pid_gain(  );
  cugo_start_odometer();          
  cugo_target_count_L = (target_radius-CUGO_TREAD/2)*(target_theta*PI/180)*CUGO_CONVERSION_DISTANCE_TO_COUNT;
  cugo_target_count_R = (target_radius+CUGO_TREAD/2)*(target_theta*PI/180)*CUGO_CONVERSION_DISTANCE_TO_COUNT;      



  if(target_rpm <= 0){
    Serial.println(F("##WARNING::目標速度が0以下のため進みません##"));          
  }else if(cugo_target_count_L == 0 && cugo_target_count_R == 0){
    Serial.println(F("##WARNING::目標距離が左右ともに0のため進みません##"));          
  }else{
    if(target_radius<0){
      cugo_target_count_L = -cugo_target_count_L;
      cugo_target_count_R = -cugo_target_count_R; 
    }
    if(target_theta>0){
      if(target_radius != 0){
             //.setTargetRpm(target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
             //.setTargetRpm(target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
      }else{
             //.setTargetRpm(-target_rpm);
             //.setTargetRpm(target_rpm);      
      }
    }else if(target_theta<0){
      if(target_radius != 0){
             //.setTargetRpm(-target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
             //.setTargetRpm(-target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
      }else{
             //.setTargetRpm(target_rpm);
             //.setTargetRpm(-target_rpm);      
      }
    }else if(target_theta=0){
           //.setTargetRpm(0);
           //.setTargetRpm(0);     
    }else{
    
    }
    //★
    //if(abs(     /*.getTargetRpm()*/)>CUGO_MAX_MOTOR_RPM || abs(     /*.getTargetRpm()*/) > CUGO_MAX_MOTOR_RPM){
    //  Serial.println(F("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##"));          
    //}

    //★
    //if(abs(     /*.getTargetRpm()*/) < 20 || abs(     /*.getTargetRpm()*/) < 20){
    //  Serial.println(F("##WARNING::目標速度が十分な速度ではないため正確な軌道を進まない可能性があります。##"));          
    //}

    if(cugo_target_count_L >= 0){
      cugo_direction_L = true;
    }else{
      cugo_direction_L = false;
    }

    if(cugo_target_count_R >= 0){
      cugo_direction_R = true;
    }else{
      cugo_direction_R = false;
    }
    
     
    //cugo_test時確認用
    /*
    //Serial.println(F("start_count l:r:" + String(     .getCount())+" ,"+ String(     .getCount()));  
    //Serial.println(F("target_rpm l:r:" + String(     .getTargetRpm())+" ,"+ String(     .getTargetRpm()));  
    //Serial.println(F("target_count l:r:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));    
    */
    while(!cugo_check_count_achievement(CUGO_MOTOR_LEFT ) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT )){  
      if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
      {
        //停止しているだけの時
             //.setTargetRpm(0);
             //.setTargetRpm(0);
      }

      unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
      FloatToUC(rpm_current_L * 1.005, 2, frame);
      FloatToUC(rpm_current_R, 6, frame);
      write_bldc(frame);        

      cugo_wait(10);
      cugo_calc_odometer(  );
    }
       
    //cugo_test時確認用
    
    //Serial.println(F("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));      
    //Serial.println(F("result_count l:r:" + String(     .getCount())+" ,"+ String(     .getCount()));
    //Serial.println(F("==========="));
    
  }
  cugo_stop(  ); 
  cugo_reset_pid_gain(  );          
  }
void cugo_curve_distance_raw(float target_radius,float target_distance,float target_rpm ){
  cugo_reset_pid_gain(  );
  cugo_start_odometer();
  if(target_radius == 0){          
    Serial.println(F("##WARNING::軌道半径が0のため進みません##"));
  }else{
    cugo_target_count_L = target_distance*((target_radius-CUGO_TREAD/2)/target_radius)*CUGO_CONVERSION_DISTANCE_TO_COUNT;
    cugo_target_count_R = target_distance*((target_radius+CUGO_TREAD/2)/target_radius)*CUGO_CONVERSION_DISTANCE_TO_COUNT;
        
    if(target_rpm <= 0){
      Serial.println(F("##WARNING::目標速度が0以下のため進みません##"));          
    }else if(cugo_target_count_L == 0 && cugo_target_count_R == 0){
    Serial.println(F("##WARNING::目標距離が左右ともに0のため進みません##"));          
    }else{
    
      if(target_distance>0){
        if(target_radius != 0){
               //.setTargetRpm(target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
               //.setTargetRpm(target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
        }else{
               //.setTargetRpm(-target_rpm);
               //.setTargetRpm(target_rpm);      
        }
      }else if(target_distance<0){
        if(target_radius != 0){
               //.setTargetRpm(-target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
               //.setTargetRpm(-target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
        }else{
               //.setTargetRpm(target_rpm);
               //.setTargetRpm(-target_rpm);      
        }
      }else if(target_distance=0){
             //.setTargetRpm(0);
             //.setTargetRpm(0);     
      }else{
    
      }

      //if(abs(     /*.getTargetRpm()*/)>CUGO_MAX_MOTOR_RPM || abs(     /*.getTargetRpm()*/) > CUGO_MAX_MOTOR_RPM){
      //  Serial.println(F("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##"));          
      //}

      //if(abs(     /*.getTargetRpm()*/) < 20 || abs(     /*.getTargetRpm()*/) < 20){
      //  Serial.println(F("##WARNING::目標速度が十分な速度ではないため正確な軌道を進まない可能性があります。##"));          
      //}

      if(cugo_target_count_L >= 0){
        cugo_direction_L = true;
      }else{
        cugo_direction_L = false;
      }

      if(cugo_target_count_R >= 0){
        cugo_direction_R = true;
      }else{
        cugo_direction_R = false;
      }
         
      //cugo_test時確認用
      /*
      //Serial.println(F("start_count l:r:" + String(     .getCount())+" ,"+ String(     .getCount()));  
      //Serial.println(F("target_rpm l:r:" + String(     .getTargetRpm())+" ,"+ String(     .getTargetRpm()));  
      //Serial.println(F("target_count l:r:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));
      //   
      while(!cugo_check_count_achievement(CUGO_MOTOR_LEFT ) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT )){  
        if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
        {
          //停止しているだけの時
               //.setTargetRpm(0);
               //.setTargetRpm(0);
        }
        for (int i = 0; i < CUGO_MOTOR_NUM; i++){ 
            //[i].driveMotor();
        }
        cugo_wait(10);     
        cugo_calc_odometer(  );    
      }
        
      //cugo_test時確認用    
      /*  
      //Serial.println(F("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));      
      //Serial.println(F("result_count l:r:" + String(     .getCount())+" ,"+ String(     .getCount()));
      //Serial.println(F("==========="));
      */
    }
  }
  cugo_stop(  ); 
  cugo_reset_pid_gain(  );
  }
  //チェック関連
bool cugo_check_count_achievement(int motor_num_ ){
  //
    long int target_count_ = 0;
    long int current_count_ = 0;
    bool cugo_direction_;//前進後進方向の変数変更 
    if(motor_num_ == CUGO_MOTOR_LEFT){
      target_count_ = cugo_target_count_L;
      cugo_direction_ = cugo_direction_L;
      current_count_ =  __encorderL;
    }else if(motor_num_ == CUGO_MOTOR_RIGHT){
      target_count_ = cugo_target_count_R;
      cugo_direction_ = cugo_direction_R;      
      current_count_ =  __encorderR;

    }else{
    return false;
    }
    Serial.println(String(current_count_));


    // 目標達成チェック
    if(cugo_direction_){
      if (target_count_<=  current_count_){
      //    [motor_num_]//.setTargetRpm(0);
        return true;
      } 
    }else{
      if (target_count_>=  current_count_){
      //    [motor_num_]//.setTargetRpm(0);
        return true;
      } 
    }

    return false;
  }
int cugo_check_propo_channel_value(int channel_number){
  //noInterrupts();      //割り込み停止
  //cugoRcTime[0] = cugo_time[0];
  //cugoRcTime[1] = cugo_time[1];
  //cugoRcTime[2] = cugo_time[2];  
  //interrupts();     //割り込み開始  

  if(channel_number == CUGO_PROPO_A){
  //return cugoRcTime[0];    
  }else if(channel_number == CUGO_PROPO_B){
  //return cugoRcTime[1];    
  }else if(channel_number == CUGO_PROPO_C){
  //return cugoRcTime[2];  
  }else{
  return 0;    
  }
  return 0;

  }
bool cugo_check_button(){
  return cugo_button_check;  
  }
int cugo_check_button_times(){
  
  return cugo_button_count;  
  }
void cugo_reset_button_times(){
  cugo_button_count = 0;
  }
long int cugo_button_press_time(){
  volatile unsigned long long cugoButtonTime;
  if(cugoButtonStartTime != 0){
  cugoButtonTime = micros();
  /*if(cugo_button_check){
    noInterrupts();
    CUGO_PIN_DOWN(3);
    cugoButtonTime = cugo_time[3];
  interrupts();     
  */
  return (cugoButtonTime-cugoButtonTime)/1000;  
  }else{
  return 0;  
  }
  return 0;  

}
float cugo_check_odometer(int check_number){
  //odometer_number 0:x,1:y,2:theta
  if(check_number == CUGO_ODO_X){
  return cugo_odometer_x;    
  }else if(check_number == CUGO_ODO_Y){
  return cugo_odometer_y;    
  }else if(check_number == CUGO_ODO_THETA){
  return cugo_odometer_theta;
  }else if(check_number == CUGO_ODO_DEGREE){
  return cugo_odometer_degree;  
  }else{
  return 0;    
  }
  return 0;

  }
void cugo_calc_odometer( ){
  if(calc_odometer_time < 10000 + micros()){
    long int cugo_dif_count_theta_ =(     /*.getCount()*/-cugo_count_prev_R)-(     /*.getCount()*/-cugo_count_prev_L);
    long int cugo_dif_count_v_ =((     /*.getCount()*/-cugo_count_prev_R)+(     /*.getCount()*/-cugo_count_prev_L))/2;
    cugo_odometer_theta += cugo_dif_count_theta_*(CUGO_CONVERSION_COUNT_TO_DISTANCE/CUGO_TREAD);

    cugo_odometer_x += (cugo_dif_count_v_ * cos(cugo_odometer_theta) )*CUGO_CONVERSION_COUNT_TO_DISTANCE;
    cugo_odometer_y += (cugo_dif_count_v_ * sin(cugo_odometer_theta) )*CUGO_CONVERSION_COUNT_TO_DISTANCE;    
    cugo_odometer_degree = cugo_odometer_theta*180/PI;
    //cugo_count_prev_L =      /*.getCount()*/;
    //cugo_count_prev_R =      /*.getCount()*/;  
    calc_odometer_time = micros();
  }
  }
void cugo_reset_odometer(){
  cugo_count_prev_L = 0;
  cugo_count_prev_R = 0;
  cugo_odometer_count_theta =0;
  cugo_odometer_x = 0 ;
  cugo_odometer_y = 0 ;
  cugo_odometer_theta = 0 ;
  cugo_odometer_degree = 0 ;
  calc_odometer_time = micros();
  }
void cugo_start_odometer(){
  cugo_count_prev_L = 0;
  cugo_count_prev_R = 0;
  calc_odometer_time = micros();
  }
void cugo_init_KOPROPO(int CUGO_OLD_PWM_IN_PIN0_VALUE,int CUGO_OLD_PWM_IN_PIN1_VALUE,int CUGO_OLD_PWM_IN_PIN2_VALUE){
  //Serial.println(F("#   cugo_init_KOPROPO"));//確認用
  // ピン変化割り込みの初期状態保存
  //cugoRunMode = CUGO_RC_MODE;
  //CUGO_OLD_PWM_IN_PIN0_VALUE = digitalRead(CUGO_PWM_IN_PIN0);
  //CUGO_OLD_PWM_IN_PIN1_VALUE = digitalRead(CUGO_PWM_IN_PIN1);
  //CUGO_OLD_PWM_IN_PIN2_VALUE = digitalRead(CUGO_PWM_IN_PIN2);
  //CUGO_OLD_PWM_IN_PIN2_VALUE = digitalRead(CUGO_CMD_BUTTON_PIN);
  
  // ピン変化割り込みの設定（D5,D6,D7をレジスタ直接読み取りで割り込み処理）
  //pinMode(CUGO_PWM_IN_PIN0, INPUT);
  //pinMode(CUGO_PWM_IN_PIN1, INPUT);
  //pinMode(CUGO_PWM_IN_PIN2, INPUT);
  //pinMode(CUGO_CMD_BUTTON_PIN, INPUT_PULLUP);
  
  //PCMSK1 |= B00000100;  // A2を有効 :PCINT10
  //PCMSK1 |= (1 << INT10);
  //PCICR  |= (1 << PCIE1);
  //PCMSK2 |= (1 << INT5);
  //PCMSK2 |= (1 << INT6);
  //PCMSK2 |= (1 << INT7);
  //PCICR  |= (1 << PCIE2);
  //PCMSK2 |= B11100000;  // D5,6,7を有効 :PCINT21,22,23
  //PCICR  |= B00000110;  // PCIE1,2を有効

  //pinMode(LED_BUILTIN, OUTPUT); // Arduino/RC MODEの表示
       cugo_wait(100);
  }
void cugo_calc_necessary_rotate(float degree ) {
  //Serial.println(F("#   cugo_calc_necessary_rotate"));//確認用
  cugo_target_count_L = ((degree / 360) * CUGO_TREAD * PI) * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI);
  cugo_target_count_R = -((degree / 360) * CUGO_TREAD * PI) * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_R * PI);
  //Serial.println("cugo_target_count_L:" + String(cugo_target_count_L));
  //Serial.println(F("degree: " + String(degree));
  //Serial.println(F("### cugo_target_count_L/R: " + String(*cugo_target_count_L) + " / " + String(*cugo_target_count_R) + "###"));
  //Serial.println(F("kakudo: " + String((degree / 360) * CUGO_TREAD * PI));
  //Serial.println(F("PI: " + String(PI));
  //Serial.println(F("issyuu: " + String(2 * CUGO_WHEEL_RADIUS_R * PI));
  }

void cugo_calc_necessary_count(float distance ) {
  //Serial.println(F("#   cugo_calc_necessary_count"));//確認用

  //cugo_target_count_L = distance / (2 * CUGO_WHEEL_RADIUS_L * PI);
  //cugo_target_count_R = distance / (2 * CUGO_WHEEL_RADIUS_R * PI);
  //cugo_target_count_L = cugo_target_count_L * CUGO_ENCODER_RESOLUTION;
  //cugo_target_count_R = cugo_target_count_R * CUGO_ENCODER_RESOLUTION;
  cugo_target_count_L = convert_distanceTopulse(distance *1000.0);
  cugo_target_count_R = convert_distanceTopulse(distance *1000.0);
  Serial.print(F("distance: ")); 
  Serial.println(String(cugo_target_count_L));


  //★ 
  //cugo_target_count_L = distance * CUGO_CONVERSION_DISTANCE_TO_COUNT;
  //★ 
  //cugo_target_count_R = distance * CUGO_CONVERSION_DISTANCE_TO_COUNT;
  
  //Serial.println(F("distance: " + String(distance));
  //Serial.println(F("distance: " + String(CUGO_ENCODER_RESOLUTION));
  //Serial.println(F("2 * CUGO_WHEEL_RADIUS_L * PI: " + String(2 * CUGO_WHEEL_RADIUS_L * PI));
  //Serial.println(F("calc: " + String(distance * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI)));
  //Serial.println(F("### cugo_target_count_L/R: " + String(*cugo_target_count_L) + " / " + String(*cugo_target_count_R) + "###"));
  //Serial.println(F("distance: " + String(distance));
  //Serial.println(F("CUGO_WHEEL_RADIUS_L: " + String(CUGO_WHEEL_RADIUS_L));
  //Serial.println(F("PI: " + String(PI));
  //Serial.println(F("issyuu: " + String(2 * CUGO_WHEEL_RADIUS_R * PI));

  }
void cugo_motor_direct_instructions(int left, int right ){
  //Serial.println(F("#   cugo_motor_direct_instructions"));//確認用
       //.servo_.writeMicroseconds(left);
       //.servo_.writeMicroseconds(right);
  }
/*
void cugo_rcmode(volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX] ){
  //Serial.println(F("#   cugo_CUGO_RC_MODE"));//確認用  
  digitalWrite(LED_BUILTIN, LOW); // CUGO_RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  //if((cugoRcTime[0] < CUGO_PROPO_MAX_A && cugoRcTime[0] > CUGO_PROPO_MIN_A) && (cugoRcTime[2] < CUGO_PROPO_MAX_C && cugoRcTime[2] > CUGO_PROPO_MIN_C) ){
  //  cugo_motor_direct_instructions(cugoRcTime[0], cugoRcTime[2] );
  //}
  //★超えた値のパターン変更 max超えたとき min超えたとき 
  //左 max超えたときは？ 
  //外れ値の場合は無視、プロポの入力かどうかの確認//1500 +-20くらい？と仮定する
  //①についてまずは仮定 ②は仮定

  }
  */
void cugo_stop( ){
  //Serial.println(F("#   cugo_stop"));//確認用
       //.setTargetRpm(0.0);
       //.setTargetRpm(0.0);
  cugo_motor_direct_instructions(1500, 1500 );
  cugo_wait(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  
  }
void cugo_reset_pid_gain( ){
  //Serial.println(F("#   cugo_reset_pid_gain"));//確認用  
  }
void cugo_init_display(){
  //delay(30);
  cugo_wait(30);

  /*  
  Serial.println(F(""));
  Serial.println(F(""));  
  Serial.println(F("#######################################"));
  Serial.println(F("#######################################"));
  Serial.println(F("#                                     #"));
  Serial.println(F("#   ####    ##  ##    ####     ####   #"));
  Serial.println(F("#  ##  ##   ##  ##   ##  ##   ##  ##  #"));
  Serial.println(F("#  ##       ##  ##   ##       ##  ##  #"));
  Serial.println(F("#  ##       ##  ##   ## ###   ##  ##  #"));
  Serial.println(F("#  ##       ##  ##   ##  ##   ##  ##  #"));
  Serial.println(F("#  ##  ##   ##  ##   ##  ##   ##  ##  #"));
  Serial.println(F("#   ####     ####     ####     ####   #"));
  Serial.println(F("#                                     #"));
  Serial.println(F("#######################################"));
  Serial.println(F("#######################################"));
  Serial.println(F(""));
  Serial.println(F(""));  
  */
  Serial.println(F("###############################"));  
  Serial.println(F("###   CugoAruduinoKitStart  ###"));
  Serial.println(F("###############################"));  

  }
void cugo_test(int test_number ){

  if(test_number == 0){//試験プログラムパターン⓪：サンプルプログラムテスト
    Serial.println(F("自動走行モード開始"));  
    Serial.println(F("1.0mの正方形移動の実施"));

    cugo_move_forward(1.0 );
    cugo_wait(1000);
    cugo_turn_clockwise(90,0 );
    cugo_wait(1000);
    cugo_move_forward(1.0 );
    cugo_wait(1000);
    cugo_turn_clockwise(90,0 );
    cugo_wait(1000);
    cugo_move_forward(1.0 );
    cugo_wait(1000);
    cugo_turn_clockwise(90,0 );
    cugo_wait(1000);
    cugo_move_forward(1.0 );
    cugo_wait(1000);
    cugo_turn_clockwise(90,0 );
    cugo_wait(1000);

    Serial.println(F("自動走行モード終了")); 
    cugoRunMode = RC_MODE;  
  }
  if(test_number == 1){//試験プログラムパターン①：走行関連テスト
    Serial.println(F("自動走行モード開始"));
    unsigned long int cugo_test_start = micros();  
    Serial.println(F("cugo_move_forward(0.5 )"));
      cugo_move_forward(0.5 );
    Serial.println(F("cugo_move_forward(-0.5 )"));
      cugo_move_forward(-0.5 );      
    Serial.println(F("cugo_move_forward(0 )"));
      cugo_move_forward(0 ); 
    
    Serial.println(F("cugo_move_forward_raw(0.5,70 )"));
      cugo_move_forward_raw(0.5,70 );      
    Serial.println(F("cugo_move_forward_raw(-0.5,40 )"));
      cugo_move_forward_raw(-0.5,40 );      

    Serial.println(F("cugo_turn_clockwise(90 )"));
      cugo_turn_clockwise(90 );
    Serial.println(F("cugo_turn_clockwise(-90 )"));
      cugo_turn_clockwise(-90 );
    Serial.println(F("cugo_turn_clockwise(0 )"));
      cugo_turn_clockwise(0 ); 
    Serial.println(F("cugo_turn_clockwise(90,90 )"));
      cugo_turn_clockwise(90,90 );
    Serial.println(F("cugo_turn_clockwise(90,0 )"));
      cugo_turn_clockwise(90,0 );
    Serial.println(F("cugo_turn_clockwise(90,-90 )"));
      cugo_turn_clockwise(90,-90 ); 

    Serial.println(F("cugo_turn_clockwise_raw(60,80 )"));
      cugo_turn_clockwise_raw(60,80 );
    Serial.println(F("cugo_turn_clockwise_raw(-60,40 )"));
      cugo_turn_clockwise_raw(-60,40 );

    Serial.println(F("cugo_turn_counterclockwise(90 )"));
      cugo_turn_counterclockwise(90 );
    Serial.println(F("cugo_turn_counterclockwise(-90 )"));
      cugo_turn_counterclockwise(-90 );
    Serial.println(F("cugo_turn_counterclockwise(0 )"));
      cugo_turn_counterclockwise(0 );
    Serial.println(F("cugo_turn_counterclockwise(90,90 )"));
      cugo_turn_counterclockwise(90,90 );
    Serial.println(F("cugo_turn_counterclockwise(90,0 )"));
      cugo_turn_counterclockwise(90,0 );
    Serial.println(F("cugo_turn_counterclockwise(90,-90 )"));
      cugo_turn_counterclockwise(90,-90 );

    Serial.println(F("cugo_turn_counterclockwise_raw(60,80 )"));
      cugo_turn_counterclockwise_raw(60,80 );
    Serial.println(F("cugo_turn_counterclockwise_raw(-60,40 )"));
      cugo_turn_counterclockwise_raw(-60,40 );

    Serial.println(F("cugo_curve_theta_raw(1.0,90,90 )"));
      cugo_curve_theta_raw(1.0,90,90 );
    Serial.println(F("cugo_curve_theta_raw(-1.0,90,90 )"));
      cugo_curve_theta_raw(-1.0,90,90 );
    Serial.println(F("cugo_curve_theta_raw(0,90,90 )"));
      cugo_curve_theta_raw(0,90,90 );
    Serial.println(F("cugo_curve_theta_raw(0.5,540,90 )"));
      cugo_curve_theta_raw(0.5,540,90 );
    Serial.println(F("cugo_curve_theta_raw(1.0,-90,90 )"));
      cugo_curve_theta_raw(1.0,-90,90 );
    Serial.println(F("cugo_curve_theta_raw(1.0,0,90 )"));
      cugo_curve_theta_raw(1.0,0,90 );
    Serial.println(F("cugo_curve_theta_raw(1.0,90,180 )"));
      cugo_curve_theta_raw(1.0,90,180 );
    Serial.println(F("cugo_curve_theta_raw(1.0,90,-90 )"));
      cugo_curve_theta_raw(1.0,90,-90 );
    Serial.println(F("cugo_curve_theta_raw(1.0,90,0 )"));
      cugo_curve_theta_raw(1.0,90,0 );
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,90 )"));
      cugo_curve_distance_raw(1.0,1.0,90 );
    Serial.println(F("cugo_curve_distance_raw(-1.0,1.0,90 )"));
      cugo_curve_distance_raw(-1.0,1.0,90 );    
    Serial.println(F("cugo_curve_distance_raw(0,1.0,90 )"));
      cugo_curve_distance_raw(0,1.0,90 );
    Serial.println(F("cugo_curve_distance_raw(0.5,12.0,90 )"));
      cugo_curve_distance_raw(0.5,12.0,90 );
    Serial.println(F("cugo_curve_distance_raw(1.0,-1.0,90 )"));
      cugo_curve_distance_raw(1.0,-1.0,90 );    
    Serial.println(F("cugo_curve_distance_raw(1.0,0,90 )"));
      cugo_curve_distance_raw(1.0,0,90 );
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,180 )"));
      cugo_curve_distance_raw(1.0,1.0,180 );
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,-90 )"));
      cugo_curve_distance_raw(1.0,1.0,-90 );    
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,0 )"));
      cugo_curve_distance_raw(1.0,1.0,0 );

    Serial.println(F("自動走行モード終了")); 
    Serial.println("処理時間(micros)::" + String(micros()-cugo_test_start)); 
    cugoRunMode = RC_MODE;
  }
   
  if(test_number == 2){//試験プログラムパターン②：プロポ入力、ボタン関連テスト
  
  unsigned long int cugo_test_start = micros();  
  //試験用関数記載
  Serial.print("Ach::" + String(cugo_check_propo_channel_value(0))+"  Bch::" + String(cugo_check_propo_channel_value(1))+ "  Cch::" + String(cugo_check_propo_channel_value(2))); 
  Serial.print(", cugo_button_press_time::"+String(cugo_button_press_time()) +", cugo_check_button_times::"+ String(cugo_check_button_times()) +", cugo_check_button::"+ String(cugo_check_button()));
  cugo_wait(100);
  if(cugo_check_button_times() >10){
    Serial.println(F(""));
    Serial.println(F("####################################"));
    Serial.println(F("cugo_reset_button_times::count_reset"));
    Serial.println(F("####################################"));
    Serial.println(F(""));
    cugo_reset_button_times();
  }  
  
  Serial.println(", 処理時間(micros)::" + String(micros()-cugo_test_start)); 
  cugo_wait(50);
  cugoRunMode = RC_MODE; //自動走行モードをループしたい場合はCUGO_SELF_DRIVE_MODEに変更
   
  }
  if(test_number == 3){//試験プログラムパターン③
  Serial.println(F("自動走行モード開始"));  
  unsigned long int cugo_test_start = micros();  
  //試験用関数記載
    Serial.println(F("半径1.0mのS字移動"));
    cugo_curve_distance_raw(1.0,180,90 );
    cugo_wait(1000);
    cugo_curve_distance_raw(-1.0,180,90 );
    cugo_wait(1000);

  Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
  Serial.println(F("自動走行モード終了")); 
  }
  
  if(test_number == 4){//試験プログラムパターン④ランダム試験
  Serial.println(F("自動走行モード開始"));  
  unsigned long int cugo_test_start = micros();  
  //試験用関数記載

  /*
    //試験用関数記載
    unsigned long int  target_time_test;//32767
    target_time_test = 60*60*1000;
    Serial.println(F("test_time"+String(target_time_test));
    cugo_long_wait(target_time_test);
    Serial.println(String(target_time_test));
    Serial.println(F("done!"));
    cugo_turn_clockwise(90,-20 );
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_clockwise(90,0 );
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_counterclockwise(90,-20 );
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_counterclockwise(90,-20 );
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_theta_raw(1.0,30,-90 );
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_theta_raw(1.0,30,0 );
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 
    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_distance_raw(0,18,90 );
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
        Serial.println(F("start"));  
        cugo_long_wait(7200);
        Serial.println(F("done"));  

  */
  Serial.println(F("自動走行モード終了")); 

  }

  }

//bldc  
//c_BLDC_Driver() {
//}

//------------------------------------便利関数
void  FloatToUC(float data, long int start, unsigned char* index) {  //配列indexの4番目からfloat dataを書き込む場合-> FloatToInt(data, 4, index);
  memcpy(&index[start], &data, 4);
}
void  IndexToFloat(unsigned char* index, long int start, float* data) {  //配列indexの3番目からfloat dataに書き込む場合-> IndexToFloat(index, 3, data);
  memcpy(data, &index[start], 4);
}
void  IndexToShort(unsigned char* index, long int start, short* data) {  //配列indexの3番目からuint16_t dataに書き込む場合-> IndexToFloat(index, 3, data);
  memcpy(data, &index[start], 2);
}

long int  convert_distanceTopulse(float distance_mm) {  //ミリメートル指定
  return 2.15592274678 * distance_mm * DIST_PERROT / PULSE_PERROT;
}

//------------------------------------通信関係
void  write_bldc(unsigned char cmd[10]) {  //引数はidとチェックサム以外の配列
  long int i;
  unsigned char checksum = id;
  for (i = 0; i < 10; i++) {
    Serial1.write(cmd[i]);
    //Serial.print(cmd[i],HEX);
    //Serial.print(",");
    checksum += (unsigned char)(cmd[i]);
  }
  Serial1.write(id);
  //Serial.print(id,HEX);
  //Serial.print(",");
  Serial1.write(checksum);
  //Serial.println(checksum,HEX);
  id++;
  if (id>0xFF) id = 0;
}

void  get_bldc() {  //引数はidとチェックサム以外の配列
  unsigned char frame[12];
  while (Serial1.available() >= 12) {
    while(Serial1.read() != 0xFF){
    }
    frame[0] = 0xFF;
    //Serial.print(String(frame[0]));
    //Serial.print(",");      
    for (long int i = 1; i < 12; i++) {
      frame[i] = Serial1.read();
    //  Serial.print(String(frame[i]));
    //  Serial.print(",");      
    }
    //  Serial.println("");
    /*
    if(frame[1] == 0x8E){  //5.5.6 Encoder Feedback
      set_encorder(frame);

    }*/
    
    if (frame[1] == 0x80){  //5.5.1 Control Mode Feedback
      if(frame[2] == 0x00){
        if(cugoOldRunMode == ARDUINO_MODE){
          cugoRunMode = RC_MODE;
          runMode= RC_MODE;
          cugoOldRunMode = RC_MODE;
          Serial.println(F("###   MODE:CUGO_RC_MODE        ###"));
          reset_arduino_mode_flags();
                    
        }else if(cugoOldRunMode == RC_MODE){

        }else{

        }
      }else if(frame[2] == 0x01){
        if(cugoOldRunMode == RC_MODE){
          cugoRunMode = ARDUINO_MODE;
          runMode= ARDUINO_MODE;
          cugoOldRunMode = ARDUINO_MODE;
          Serial.println(F("###   MODE:CUGO_ARDUINO_MODE###"));          
        }else if(cugoOldRunMode == RC_MODE){

        }else{

        }
      }
    }else if(frame[1] == 0x82){  //5.5.2 Command RPM Feedback

    }else if(frame[1] == 0x84){  //5.5.3 Current RPM Feedback
    //IndexToFloat(frame,2,&rpm_current_L);
    //IndexToFloat(frame,4,&rpm_current_R);

    }else if(frame[1] == 0x86){  //5.5.4 Average RPM Feedback

    }else if(frame[1] == 0x8D){  //5.5.5 SBUS Signal Feedback

    }else if(frame[1] == 0x8E){  //5.5.6 Encoder Feedback
      set_encorder(frame);

    }else if(frame[1] == 0x8F){  //Data Feedback Config

    }else{

    }

  }
  
}

void  BackGround() {  //Timerから呼ばれる
  //入力
  get_bldc();
  /*
  //出力(加速度と速度制御)
  rpm_current_L += 1.0 * accelerationL /  FEEDBACK_HZ;
  rpm_current_R += 1.0 * accelerationR /  FEEDBACK_HZ;

  if (target_rpmR - fabs(accelerationR) /  FEEDBACK_HZ < rpm_current_R && rpm_current_R < target_rpmR + fabs(accelerationR) /  FEEDBACK_HZ) {
    rpm_current_R = target_rpmR;
    accelerationR = 0;
  }

  if (target_rpmL - fabs(accelerationL) /  FEEDBACK_HZ < rpm_current_L && rpm_current_L < target_rpmL + fabs(accelerationL) /  FEEDBACK_HZ) {
    rpm_current_L = target_rpmL;
    accelerationL = 0;
  }

  //出力
  unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
  FloatToUC(rpm_current_L * 1.005, 2, frame);
  FloatToUC(rpm_current_R, 6, frame);
  write_bldc(frame);
  */
}

//------------------------------------通信処理
void  set_encorder(unsigned char frame[12]) {
  short encorderR = 0, encorderL = 0;
  IndexToShort(frame, 2, &encorderL);
  IndexToShort(frame, 4, &encorderR);
  __encorderL = __encorderL + (int)encorderL - (int)_encorderL;
  __encorderR = __encorderR + (int)encorderR - (int)_encorderR;
  _encorderL = encorderL;
  _encorderR = encorderR;
}

void  Encorder_reset() {
  unsigned char frame[10] = { 0xFF, 0x0E, 0x01, 0x01, 0, 0, 0, 0, 0, 0 };
  write_bldc(frame);
  __encorderL = _encorderL = 0;
  __encorderR = _encorderR = 0;
  delay(200);
}

//----------------------------------------------------------------------以下操作関係-------------------------------------------------------
//------------------------------------設定関係

void  set_feedback(unsigned char freq_index, unsigned char kindof_data) {  //freq 0:10[hz] 1:50[hz] 2:100[hz] kindof_data 1:Mode 2:CMD_RPM 4:CurrentRPM 8:AveCurrentRPM 128:EncorderData
  unsigned char frame[10] = { 0xFF, 0x0F, freq_index, kindof_data, 0, 0, 0, 0, 0, 0 };
   FEEDBACK_HZ = index_tofreq[freq_index];
  FEEDBACK_DUTATION = 1000 /  FEEDBACK_HZ;
  write_bldc(frame);
}

void  setControlMode(unsigned char mode) {  //mode 0x00:RC_mode 0x01:CMD_Mode
  unsigned char frame[10] = { 0xFF, 0x00, mode, 0, 0, 0, 0, 0, 0, 0 };
  write_bldc(frame);
}

//------------------------------------動作関係
void  setSpeed(float left_rpm, float right_rpm) {  //非同期処理 IOcontrolと並行
  rpm_current_R = right_rpm;
  rpm_current_L = left_rpm;
}


/*
  void  accelR(float accel_rpmR, float target_rpm) {  //非同期処理 IOcontrolと並行
    if (rpm_current_R == target_rpm) return;
    accelerationR = rpm_current_R < target_rpm ? accel_rpmR : -accel_rpmR;
    target_rpmR = target_rpm;
  }

  void  accelL(float accel_rpmL, float target_rpm) {  //非同期処理 IOcontrolと並行
    if (rpm_current_L == target_rpm) return;
    accelerationL = rpm_current_L < target_rpm ? accel_rpmL : -accel_rpmL;
    target_rpmL = target_rpm;
  }


  void  move_stop(long int pulsesR, long int pulsesL, float accel_rpmR, float accel_rpmL, float max_rpmR, float max_rpmL, float brake_rpmR, float brake_rpmL) {
    Serial.print(pulsesR);
    Serial.print(",");
    Serial.println(pulsesL);
    Encorder_reset();
    long int stepR = 0, stepL = 0;
    long int getencR = abs(getEncorder_R()), getencL = abs(getEncorder_L());
    long int fugouR = pulsesR / abs(pulsesR);
    long int fugouL = pulsesL / abs(pulsesL);
    pulsesR = fabs(pulsesR);
    pulsesL = fabs(pulsesL);
    accel_rpmR = fabs(accel_rpmR);
    accel_rpmL = fabs(accel_rpmL);
    max_rpmR = fabs(max_rpmR) * fugouR;
    max_rpmL = fabs(max_rpmL) * fugouL;
    brake_rpmR = fabs(brake_rpmR);
    brake_rpmL = fabs(brake_rpmL);
    long int _millis = millis();
    while (getencR < pulsesR || getencL < pulsesL || millis() - _millis < 1000) {

      if ((rpm_current_R * rpm_current_R * PULSE_PERROT / brake_rpmR / 95.0 - 100) <= pulsesR - getencR && stepR == 0 || getencR < pulsesR / 10) {
        accelR(accel_rpmR, max_rpmR);
        stepR = 0;
      } else {
        if (getencR < pulsesR)
          accelR(brake_rpmR, 10 * fugouR);
        else
          accelR(brake_rpmL, 0);
        stepR = 1;
      }

      if ((rpm_current_L * rpm_current_L * PULSE_PERROT / brake_rpmL / 95.0 - 100) <= pulsesL - getencL && stepL == 0 || getencR < pulsesR / 10) {
        accelL(accel_rpmL, max_rpmL);
        stepL = 0;
      } else {
        if (getencL < pulsesL)
          accelL(brake_rpmL, 10 * fugouL);
        else
          accelL(brake_rpmL, 0);
        stepL = 1;
      }
      delay(FEEDBACK_DUTATION);

      getencR = abs(getEncorder_R());
      getencL = abs(getEncorder_L());
    }

    accelR(0, 0);
    accelL(0, 0);
    setSpeed(0, 0);
  }

  void  move_stop(long int pulses, float accel_rpm, float max_rpm, float brake_rpm) {  //同期処理 加速度accel_rpmで加速しmax_rpmで走り加速度-brake_rpmで減速する。道のりはpulses[パルス]になるようにする。
    move_stop(pulses, pulses, accel_rpm, accel_rpm, max_rpm, max_rpm, brake_rpm, brake_rpm);
  }

  void  move_stop(long int milli_meter) {
    move_stop(convert_distanceTopulse(milli_meter), 25, 150, 25);
  }

  void  rotation_stop(long int degree, float accel_rpm, float max_rpm, float brake_rpm) {
    long int length = 1.15 * degree * WIDTH_BLDC * 3.1415926 / 360.0;
    Serial.println(convert_distanceTopulse(length));
    move_stop(convert_distanceTopulse(-length), convert_distanceTopulse(length), accel_rpm, accel_rpm, max_rpm, max_rpm, brake_rpm, brake_rpm);
  }
  void  rotation_stop(long int degree) {
    rotation_stop(degree, 50, 120, 50);
  }

  void  rotation(long int degree, long int radius_milli) {
  }

  void  forward(long int millimeter) {
    Encorder_reset();
    while (convert_distanceTopulse(millimeter) < getEncorder_R()) {
    }
  }

  long int  getEncorder_L() {
    return __encorderL;
  }

  long int  getEncorder_R() {
    return __encorderR;
  }

  long int  getDuration() {
    return FEEDBACK_DUTATION;
  }


 */

 float convert_pulseTodistance(long int pulse){
 return pulse/14516.5464949853;

 }

