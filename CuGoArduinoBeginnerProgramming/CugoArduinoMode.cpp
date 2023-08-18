#include "CugoArduinoMode.h"

/***** ↓必要に応じて各ユーザーごとに設定可能↓ *****/
// シリアル通信での情報の表示有無
bool UDP_CONNECTION_DISPLAY = false;
bool ENCODER_DISPLAY = false;
bool PID_CONTROLL_DISPLAY = false;
bool FAIL_SAFE_DISPLAY = false;
bool INPUT_OUTPUT_DISPLAY = false;

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
int runMode = RC_MODE;
// PID位置制御のデータ格納
float l_count_prev_i_ = 0;
float l_count_prev_p_ = 0;
float r_count_prev_i_ = 0;
float r_count_prev_p_ = 0;
float l_count_gain = 0;
float r_count_gain = 0;

int OLD_PWM_IN_PIN0_VALUE;   // プロポスティック入力値(L)
int OLD_PWM_IN_PIN1_VALUE;   // プロポスティック入力値(MODE)
int OLD_PWM_IN_PIN2_VALUE;   // プロポスティック入力値(R)
volatile unsigned long upTime[PWM_IN_MAX];
volatile unsigned long rcTime[PWM_IN_MAX];
volatile unsigned long time[PWM_IN_MAX];


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

void init_KOPROPO(int runMode, int OLD_PWM_IN_PIN0_VALUE, int OLD_PWM_IN_PIN1_VALUE, int OLD_PWM_IN_PIN2_VALUE)
{
  //Serial.println(F("#   init_KOPROPO"));//確認用
  // ピン変化割り込みの初期状態保存
  runMode = RC_MODE;
  OLD_PWM_IN_PIN0_VALUE = digitalRead(PWM_IN_PIN0);
  OLD_PWM_IN_PIN1_VALUE = digitalRead(PWM_IN_PIN1);
  OLD_PWM_IN_PIN2_VALUE = digitalRead(PWM_IN_PIN2);

  // ピン変化割り込みの設定（D5,D6,D7をレジスタ直接読み取りで割り込み処理）
  pinMode(PWM_IN_PIN0, INPUT);
  pinMode(PWM_IN_PIN1, INPUT);
  pinMode(PWM_IN_PIN2, INPUT);
  PCMSK2 |= B11100000;  // D5,6,7を有効
  PCICR  |= B00000100;  // PCIE2を有効

  pinMode(LED_BUILTIN, OUTPUT); // Arduino/RC MODEの表示
  delay(100);
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
  pinMode(CMD_BUTTON_PIN, INPUT);
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

void display_failsafe(bool FAIL_SAFE_DISPLAY, int runMode)
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
void check_achievement_wait_time_cmd(MotorController motor_controllers[2])
{
  //Serial.println(F("#   check_achievement_wait_time_cmd"));//確認用
  if (target_wait_time < micros())
  {
    stop_motor_immediately(motor_controllers);
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
  //Serial.println("degree: " + String(degree));
  //Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * tread * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
}


void calc_necessary_count(float distance)
{
  //Serial.println(F("#   calc_necessary_count"));//確認用
  //  target_count_L = distance * encoder_resolution / (2 * wheel_radius_l * PI);
  //  target_count_R = distance * encoder_resolution / (2 * wheel_radius_r * PI);

  //target_count_L = distance / (2 * wheel_radius_l * PI);
  //target_count_R = distance / (2 * wheel_radius_r * PI);
  //target_count_L = target_count_L * encoder_resolution;
  //target_count_R = target_count_R * encoder_resolution;
  target_count_L = distance * conversion_distance_to_count;
  target_count_R = distance * conversion_distance_to_count;
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

void display_speed(MotorController motor_controllers[2], bool ENCODER_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
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
    Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    Serial.print(F(","));
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());  //制御量を見るため。
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));


    //Serial.print("PID CONTROL RPM(L/R):");
    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。
    //Serial.print(",");
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());    //制御量を見るため。

    //Serial.println(""); // 改行
  }
}
void display_target_rpm(MotorController motor_controllers[2], bool ENCODER_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{

  if (ENCODER_DISPLAY == true)
  {
    //Serial.println(F("#   display_target_rpm"));//確認用
    Serial.print(F("target_rpm[L]:"));
    Serial.println(String(motor_controllers[MOTOR_LEFT].getTargetRpm()));
    Serial.print(F("target_rpm[R]:"));
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getTargetRpm()));
  }
}
void display_PID(MotorController motor_controllers[2], bool PID_CONTROLL_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{


  if (PID_CONTROLL_DISPLAY == true)
  {
    //Serial.println("#   display_PID");// 確認用
    Serial.print(F("Encoder count (L/R): "));
    Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    Serial.print(F(","));
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));

    Serial.print(F("Target RPM (L/R): "));
    Serial.print(String(motor_controllers[MOTOR_LEFT].getTargetRpm()));
    Serial.print(F(","));
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getTargetRpm()));

    Serial.print(F("PID CONTROL RPM(L/R):"));
    Serial.print(String(motor_controllers[MOTOR_LEFT].getRpm()));
    Serial.print(F(","));
    Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用

    Serial.println(F("PID controll gain = P x kp + I x ki + D x kd"));
    Serial.print(F("[L]: "));
    Serial.print(String(motor_controllers[MOTOR_LEFT].getSpeed()));
    Serial.print(F(" = "));
    Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_P()));
    Serial.print(F(" x "));
    Serial.print(String(L_KP));
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_I()));
    Serial.print(F(" x "));
    Serial.print(String(L_KI));
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_D()));
    Serial.print(F(" x "));
    Serial.println(String(L_KD));
    Serial.print(F("[R]: "));
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getSpeed()));
    Serial.print(F(" = "));
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_P()));
    Serial.print(F(" x "));
    Serial.print(String(R_KP));
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_I()));
    Serial.print(F(" x "));
    Serial.print(String(R_KI));
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_D()));
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

void motor_direct_instructions(int left, int right, MotorController motor_controllers[2]) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println(F("#   motor_direct_instructions"));//確認用
  motor_controllers[0].servo_.writeMicroseconds(left);
  motor_controllers[1].servo_.writeMicroseconds(right);
}
void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX], MotorController motor_controllers[2])
{
  //Serial.println(F("#   rc_mode"));//確認用
  digitalWrite(LED_BUILTIN, LOW); // RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  if((rcTime[0] < CUGO_PROPO_MAX_A && rcTime[0] > CUGO_PROPO_MIN_A) && (rcTime[2] < CUGO_PROPO_MAX_C && rcTime[2] > CUGO_PROPO_MIN_C) ) {
    motor_direct_instructions(rcTime[0], rcTime[2], motor_controllers);
    if(INPUT_OUTPUT_DISPLAY){
      Serial.println("input cmd:" + String(rcTime[0]) + ", " + String(rcTime[2]) + " B-ch: " + String(rcTime[1]));
    }
  }
}

void stop_motor_immediately(MotorController motor_controllers[2])
{
  //Serial.println(F("#   stop_motor_immediately"));//確認用
  motor_controllers[0].setTargetRpm(0.0);
  motor_controllers[1].setTargetRpm(0.0);
  motor_direct_instructions(1500, 1500, motor_controllers);
}

void reset_pid_gain(MotorController motor_controllers[2])
{
  //Serial.println(F("#   reset_pid_gain"));//確認用
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    motor_controllers[i].reset_PID_param();
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
      Serial.print(F("m　前に進む "));
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

void set_go_forward_cmd(MotorController motor_controllers[2])
{
  //Serial.println(F("#   set_go_forward_cmd"));//確認用
  target_count_L = motor_controllers[0].getCount() + arduino_count_cmd_matrix[current_cmd][0];
  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String(motor_controllers[MOTOR_LEFT].getCount()) + " + " + String(arduino_count_cmd_matrix[current_cmd][0]));
  //while(1);
  if (arduino_count_cmd_matrix[current_cmd][0] >= 0) {
    cmd_L_back = false;
  } else {
    cmd_L_back = true;
  }

  target_count_R = motor_controllers[1].getCount() + arduino_count_cmd_matrix[current_cmd][1];
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
  Serial.print(F("button_read: "));
  Serial.println(String(digitalRead(CMD_BUTTON_PIN)));
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

void cmd_end(MotorController motor_controllers[2])
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
    reset_pid_gain(motor_controllers);
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



void cmd_manager_flags_init(MotorController motor_controllers[2])
{
  //Serial.println(F("#   cmd_manager_flags_init"));
  // これからコマンドを実行するときの処理

  reset_pid_gain(motor_controllers);
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
    /*
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
      }*/

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
    stop_motor_immediately(motor_controllers);

    while (1);
  }
}

void check_achievement_go_forward_cmd(MotorController motor_controllers[2])// motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  bool L_done = false;
  bool R_done = false;

  // L側目標達成チェック
  if (cmd_L_back == false) {
    if (target_count_L < motor_controllers[0].getCount())
      L_done = true;
    //Serial.println(F("#   L_done"));
  } else {
    if (target_count_L > motor_controllers[0].getCount())
      L_done = true;
    //Serial.println(F("#   L_done"));
  }

  // R側目標達成チェック
  if (cmd_R_back == false) {
    if (target_count_R < motor_controllers[1].getCount())
      R_done = true;
    //Serial.println(F("#   R_done"));
  } else {
    if (target_count_R > motor_controllers[1].getCount())
      R_done = true;
    //Serial.println(F("#   R_done"));
  }

  if (L_done == true)
  {
    motor_controllers[0].setTargetRpm(0);
  }
  if (R_done == true)
  {
    motor_controllers[1].setTargetRpm(0);
  }

  // L/R達成していたら終了
  if (L_done == true && R_done == true)
  {
    stop_motor_immediately(motor_controllers);
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
      //distance =  (motor_controllers[0].getCount()) * (( 2 * wheel_radius_l * PI) / encoder_resolution);
      distance = motor_controllers[0].getCount() * conversion_count_to_distance;
      Serial.print(String(fabsf(distance)));
      Serial.print(F(" m　進んだ"));
    } else if (arduino_flag_cmd_matrix[current_cmd][2] < 0 && arduino_flag_cmd_matrix[current_cmd][3] > 0) //左回り
    {
      degree = (2 * wheel_radius_l * PI * motor_controllers[0].getCount() * 360) / (encoder_resolution * tread * PI);
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　左回りに回転した"));
    } else if (arduino_flag_cmd_matrix[current_cmd][2] > 0 && arduino_flag_cmd_matrix[current_cmd][3] < 0) //右回り
    {
      degree = (2 * wheel_radius_l * PI * motor_controllers[0].getCount() * 360) / (encoder_resolution * tread * PI);
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　右回りに回転した"));
    } else if (arduino_flag_cmd_matrix[current_cmd][2] < 0 && arduino_flag_cmd_matrix[current_cmd][3] < 0) //後進
    {
      distance =  (motor_controllers[0].getCount()) * (( 2 * wheel_radius_l * PI) / encoder_resolution);
      Serial.print(String(abs(distance)));
      Serial.print(F(" m　後に進んだ"));
    } else {
      Serial.print(F("不明なコマンド"));
    }
    Serial.println(F("  ###"));

  }
}


void cmd_manager(MotorController motor_controllers[2])
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
      cmd_manager_flags_init(motor_controllers);
      // 前後進の指示をセット
      if (count_done == false)
      {
        set_go_forward_cmd(motor_controllers);
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
        //停止しているだけの時
        motor_controllers[0].setTargetRpm(arduino_flag_cmd_matrix[current_cmd][2]);
        motor_controllers[1].setTargetRpm(arduino_flag_cmd_matrix[current_cmd][3]);

      } else {
        // 各制御値の計算
        l_count_p = arduino_count_cmd_matrix[current_cmd][0] - motor_controllers[0].getCount();
        l_count_i = l_count_prev_i_ + l_count_p;
        l_count_d = l_count_p - l_count_prev_p_;
        r_count_p = arduino_count_cmd_matrix[current_cmd][1] - motor_controllers[1].getCount();
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
        motor_controllers[MOTOR_LEFT].setTargetRpm(l_count_gain);
        motor_controllers[MOTOR_RIGHT].setTargetRpm(r_count_gain);
        //Serial.print(F("gain:l/r "));
        //Serial.print(String(l_count_gain));
        //Serial.print(F(","));
        //Serial.println(String(r_count_gain));
      }

      // 成功条件の確認
      // if conuntの成功条件
      if (count_done == false)
        check_achievement_go_forward_cmd(motor_controllers);

      if (wait_done == false)
        check_achievement_wait_time_cmd(motor_controllers);

      if (button_done == false)
        check_achievement_button_cmd(motor_controllers);

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
          Serial.println(F("##########################"));
          Serial.println(F("###   モード:RC_MODE    ###"));
          Serial.println(F("##########################"));
        }
      }
    }
  }
}

void check_achievement_button_cmd(MotorController motor_controllers[2])
{
  if (digitalRead(CMD_BUTTON_PIN) == 0)
  {
    button_push_count++;
  } else {
    button_push_count = 0;
  }

  if (button_push_count >= 5) // 実測で50ms以上長いと小刻みに押したとき反応しないと感じてしまう。
  {
    stop_motor_immediately(motor_controllers);
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
  Serial.println(F("##########################"));
  Serial.println(F("### CugoAruduinoKit起動 ###"));
  Serial.println(F("##########################"));

}



void job_100ms(MotorController motor_controllers[2])//100msごとに必要な情報を表示
{
  display_speed(motor_controllers, ENCODER_DISPLAY);
  display_target_rpm(motor_controllers, ENCODER_DISPLAY);
  display_PID(motor_controllers, PID_CONTROLL_DISPLAY);
  display_failsafe(FAIL_SAFE_DISPLAY, runMode);
  //view_flags();
}

void job_1000ms()//1000msごとに必要な情報があれば表示
{
  display_nothing();
}

void display_detail(MotorController motor_controllers[2])
{
  if (current_time - prev_time_100ms > 100000)
  {
    job_100ms(motor_controllers);
    prev_time_100ms = current_time;
  }

  if (current_time - prev_time_1000ms > 1000000)
  {
    job_1000ms();
    prev_time_1000ms = current_time;
  }
}
