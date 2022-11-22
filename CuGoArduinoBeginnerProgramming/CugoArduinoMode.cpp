#include "Arduino.h"
#include "CugoArduinoMode.h"
#include "MotorController.h"
#include <SPI.h>

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


void init_SPI()
{
  //Serial.println(F("init_SPI()"));// デバッグ用確認
  SPI.begin();
  digitalWrite(SS, HIGH);
}

void send_spi(int mode) {
  //Serial.println(F("send_spi()"));// デバッグ用確認
  digitalWrite(SS, LOW);
  SPI.transfer(mode);
  digitalWrite(SS, HIGH);
}

void init_KOPROPO(int runMode,int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE)
{
  //Serial.println(F("init_KOPROPO()"));// デバッグ用確認
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

void set_arduino_cmd_matrix(long int cmd_0,long int cmd_1,int cmd_2,int cmd_3,int cmd_4,int cmd_5)
{
  //Serial.println("set_arduino_cmd_matrix()");// デバッグ用確認
  
  arduino_count_cmd_matrix[init_current_cmd][0] = cmd_0;
  arduino_count_cmd_matrix[init_current_cmd][1] = cmd_1;
  arduino_flag_cmd_matrix[init_current_cmd][0] = cmd_2;
  arduino_flag_cmd_matrix[init_current_cmd][1] = cmd_3;
  arduino_flag_cmd_matrix[init_current_cmd][2] = cmd_4;
  arduino_flag_cmd_matrix[init_current_cmd][3] = cmd_5;
  
}

void init_ARDUINO_CMD()
{
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
  //Serial.println("view_arduino_cmd_matrix");// デバッグ用確認
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
  //while (1); //matrixの内容を見たいだけ
}

void display_failsafe(bool FAIL_SAFE_DISPLAY,int runMode)
{
  //Serial.println("display_failsafe");// デバッグ用確認
  if (FAIL_SAFE_DISPLAY == true)
  {
    Serial.println(F("DISPLAY FAIL SAFE PARAM"));        
    Serial.print(F("Mode(ARDUINO/RC): "));
    Serial.println(runMode);
    Serial.print(F("UDP recieve fail count: "));
    Serial.println(F(""));
  }
}

void display_nothing(bool UDP_CONNECTION_DISPLAY,bool ENCODER_DISPLAY,bool PID_CONTROLL_DISPLAY)
{
  if (UDP_CONNECTION_DISPLAY == false && ENCODER_DISPLAY == false && PID_CONTROLL_DISPLAY == false)
  {
    Serial.println(F("Display item not set"));
    Serial.println(F("Arduino is working..."));
    Serial.println(F(""));
  }
}

void spi_cmd(int spi_cmd_value)
{
  Serial.println(F("spi_cmd"));// デバッグ用確認
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
    // テスト(ボタンのフラグbutton_enable==1としてテスト）
    // メモリが足りないので、buttonとSPIを共用にする。
    set_arduino_cmd_matrix( EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, spi_cmd_value, 0, 0); // ここではテストで1を使用。
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void set_wait_time_cmd()
{
  //target_wait_time = micros() + arduino_flag_cmd_matrix[current_cmd][0] * 1000;
  target_wait_time = arduino_flag_cmd_matrix[current_cmd][0];
  target_wait_time = target_wait_time * 1000; // こちらで1000倍処理
  target_wait_time = target_wait_time + micros();
  //   Serial.println(String(target_wait_time));
  // while(1);
  //  target_wait_time = micros() + 3000000;
}

void wait_time(int milisec)
{
  //Serial.println("wait_time");// デバッグ用確認
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
    // テスト(3000msの待機を固定して全体の動作テスト）
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    set_arduino_cmd_matrix(EXCEPTION_NO,EXCEPTION_NO, milisec, EXCEPTION_NO, 0, 0); // ここではテストで3000ms間、rpmを0,0(停止)にセット//EXCEPTION_NO
    Serial.print(F("##"));
    Serial.print(String(init_current_cmd+1));
    Serial.print(F("番目のコマンド："));
    Serial.print(String(milisec));
    Serial.println(F("ms待つ"));
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

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

void calc_necessary_rotate(float degree) // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
{
  //Serial.println("calc_necessary_rotate");// デバッグ用確認
  target_count_L =  ((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_l * PI);
  target_count_R = -((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_r * PI);
  //Serial.println("degree: " + String(degree));
  //Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * tread * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
  //while(1);
}


void calc_necessary_count(float distance) // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
{
  //Serial.println("calc_necessary_count");// デバッグ用確認  
  //  target_count_L = distance * encoder_resolution / (2 * wheel_radius_l * PI);
  //  target_count_R = distance * encoder_resolution / (2 * wheel_radius_r * PI);

  target_count_L = distance / (2 * wheel_radius_l * PI);
  target_count_R = distance / (2 * wheel_radius_r * PI);
  target_count_L = target_count_L * encoder_resolution;
  target_count_R = target_count_R * encoder_resolution;

  //long int target_L = distance / (2 * wheel_radius_l * PI);
  //long int target_R = distance / (2 * wheel_radius_r * PI);
  //target_count_L = target_L * encoder_resolution;
  //target_count_R = target_R * encoder_resolution;

  //Serial.println("distance: " + String(distance));
  //Serial.println("distance: " + String(encoder_resolution));
  //Serial.println("2 * wheel_radius_l * PI: " + String(2 * wheel_radius_l * PI));
  //Serial.println("calc: " + String(distance * encoder_resolution / (2 * wheel_radius_l * PI)));

  //Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("distance: " + String(distance));
  //Serial.println("wheel_radius_l: " + String(wheel_radius_l));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
  //while(1);

}


void atamaopen()
{
  //Serial.println("atamaopen");// デバッグ用確認  
  spi_cmd(6);
}

void atamaclose()
{
  //Serial.println("atamaclose");// デバッグ用確認  
  spi_cmd(5);
}

void wait_button()
{
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"));
      while (1);

    }
    // 初回起動時の処理
    // テスト(ボタンのフラグbutton_enable==1としてテスト）
    // メモリが足りないので、buttonとSPIを共用にする。
    set_arduino_cmd_matrix(EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, 255, 0, 0); // ここではテストで1を使用。//
    Serial.print(F("##"));
    Serial.print(String(init_current_cmd+1));
    Serial.print(F("番目のコマンド："));
    Serial.println(F("ボタンの押し待ち"));    
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
//退避しないでよい？
void button()
{
  wait_button();
}


void display_speed(MotorController motor_controllers[2],bool ENCODER_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println("display_speed");// デバッグ用確認    

  if (ENCODER_DISPLAY == true)
  {
    //Serial.println("DISPLAY MOTOR COUNTER & SPEED");
    //Serial.print("Mode:");
    //Serial.println(runMode);

    Serial.print(F("Encoder count (L/R):"));
    //Serial.print(motor_controllers[0].getRpm());   // 制御量を見るため。開発用
    //Serial.print(motor_controllers[0].getSpeed()); // 制御量を見るため。開発用
    Serial.print(String(motor_controllers[0].getCount()));
    Serial.print(F(","));
    //Serial.println(motor_controllers[1].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[1].getSpeed());  //制御量を見るため。デバッグ用
    Serial.println(String(motor_controllers[1].getCount()));


    //Serial.print("PID CONTROL RPM(L/R):");
    //Serial.print(motor_controllers[0].getRpm()); // 制御量を見るため。デバッグ用
    //Serial.print(motor_controllers[0].getSpeed()); // 制御量を見るため。デバッグ用
    //Serial.print(",");
    //Serial.println(motor_controllers[1].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[1].getSpeed());    //制御量を見るため。デバッグ用

    //Serial.println(""); // 改行
  }
}
void display_target_rpm(MotorController motor_controllers[2],bool ENCODER_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{   
  //Serial.println("display_target_rpm");// デバッグ用確認    

  if (ENCODER_DISPLAY == true)
  {
  Serial.print(F("target_rpm[L]:"));
  Serial.println(String(motor_controllers[0].getTargetRpm()));
  Serial.print(F("target_rpm[R]:"));
  Serial.println(String(motor_controllers[1].getTargetRpm()));
  }
}
void display_PID(MotorController motor_controllers[2],bool PID_CONTROLL_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println("display_PID");// デバッグ用確認    

  if (PID_CONTROLL_DISPLAY == true)
  {
    Serial.println(F("DISPLAY PID PRAMETER"));

    Serial.print(F("Encoder count (L/R): "));
    Serial.print(String(motor_controllers[0].getCount()));
    Serial.print(F(","));
    Serial.println(String(motor_controllers[1].getCount()));

    Serial.print(F("Target RPM (L/R): "));
    Serial.print(String(motor_controllers[0].getTargetRpm()));
    Serial.print(F(","));
    Serial.println(String(motor_controllers[1].getTargetRpm()));

    Serial.print(F("PID CONTROL RPM(L/R):")); 
    Serial.print(String(motor_controllers[0].getRpm()));
    Serial.print(F(","));
    Serial.println(motor_controllers[1].getRpm());    //制御量を見るため。デバッグ用

    Serial.println(F("PID controll gain = P x kp + I x ki + D x kd"));
    Serial.print(F("[L]: ")); 
    Serial.print(String(motor_controllers[0].getSpeed())); 
    Serial.print(F(" = "));
    Serial.print(String(motor_controllers[0].getPID_P())); 
    Serial.print(F(" x "));
    Serial.print(String(L_KP));
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[0].getPID_I())); 
    Serial.print(F(" x "));
    Serial.print(String(L_KI)); 
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[0].getPID_D())); 
    Serial.print(F(" x ")); 
    Serial.println(String(L_KD)); 
    Serial.print(F("[R]: ")); 
    Serial.print(String(motor_controllers[1].getSpeed())); 
    Serial.print(F(" = "));
    Serial.print(String(motor_controllers[1].getPID_P()));
    Serial.print(F(" x "));
    Serial.print(String(R_KP)); 
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[1].getPID_I()));
    Serial.print(F(" x ")); 
    Serial.print(String(R_KI));
    Serial.print(F(" + "));
    Serial.print(String(motor_controllers[1].getPID_D()));
    Serial.print(F(" x ")); 
    Serial.println(String(R_KD));
  }
}

int split(String data, char delimiter, String *dst)//dstは参照引き渡し
{
  Serial.println(F("split"));// デバッグ用確認    
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

void motor_direct_instructions(int left, int right,MotorController motor_controllers[2])// motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  motor_controllers[0].servo_.writeMicroseconds(left);
  motor_controllers[1].servo_.writeMicroseconds(right);
}
void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX],MotorController motor_controllers[2])
{
  //Serial.print(F("rc_mode::")));// デバッグ用確認    

  digitalWrite(LED_BUILTIN, LOW); // RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  motor_direct_instructions(rcTime[0], rcTime[2],motor_controllers);
  //Serial.println("input cmd:" + String(rcTime[0]) + ", " + String(rcTime[2]));
}

void stop_motor_immediately(MotorController motor_controllers[2])
{
  //Serial.println("stop_motor_immediately");// デバッグ用確認    

  //set_motorにしないのはセットすることでUDP受け取れないコマンドがリセットされてしまう。
  motor_controllers[0].setTargetRpm(0.0);
  motor_controllers[1].setTargetRpm(0.0);
  motor_direct_instructions(1500, 1500,motor_controllers);
}

void reset_pid_gain(MotorController motor_controllers[2])
{
  //Serial.println("reset_pid_gain");// デバッグ用確認    

  for (int i = 0; i < MOTOR_NUM; i++)
  {
    motor_controllers[i].reset_PID_param();
  }
}

void set_button_cmd()
{
  button_push_count = 0;
  button_enable = 0;
}


void go_backward(float distance,float max_velocity)
{
  if (cmd_init == false)
  {
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
    if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(-target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, -velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    Serial.print(F("##"));
    Serial.print(String(init_current_cmd+1));
    Serial.print(F("番目のコマンド："));
    Serial.print(String(distance));
    Serial.print(F("m　後ろに進む"));
    Serial.print(F("(上限速度："));        
    Serial.print(String(velocity));
    Serial.println(F("rpm )"));        
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
void sagaru(float distance)
{
  go_backward(distance,EXCEPTION_NO);
}
void sagaru(float distance,float max_velocity)
{
  go_backward(distance,max_velocity);
}

void turn_clockwise(float degree,float max_velocity)
{

  if (cmd_init == false)
  {
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
     if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }   
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, -velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    Serial.print(F("##"));
    Serial.print(String(init_current_cmd+1));
    Serial.print(F("番目のコマンド："));
    Serial.print(String(degree));
    Serial.print(F("度　右回り"));
    Serial.print(F("(上限速度："));        
    Serial.print(String(velocity));
    Serial.println(F("rpm )"));        
    init_current_cmd++;
  }
  else
  {
    // 通常ループ時の処理

  }
}

void migimawari(float degree)
{
  turn_clockwise(degree,EXCEPTION_NO);
}
void migimawari(float degree,float max_velocity)
{
  turn_clockwise(degree,max_velocity);
}
void migimawari90()
{
  migimawari(90,EXCEPTION_NO);
}
void migimawari90(float max_velocity)
{
  migimawari(90,max_velocity);
}
void migimawari45()
{
  migimawari(45,EXCEPTION_NO);
}
void migimawari45(float max_velocity)
{
  migimawari(45,max_velocity);
}
void migimawari180()
{
  migimawari(180,EXCEPTION_NO);
}
void migimawari180(float max_velocity)
{
  migimawari(180,max_velocity);
}

void go_forward(float distance,float max_velocity)
{
  if (cmd_init == false)
  {
    // 初回起動時の処理
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_count(distance);
    //Serial.println("target_count_L/R: " + String(target_count_L) + ", " + String(target_count_R));
    float velocity = 0.0;
    if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    //Serial.println("matrix_target_count_L/R: " + String(arduino_count_cmd_matrix[init_current_cmd][0]) + ", " + String(arduino_count_cmd_matrix[init_current_cmd][0]));
    Serial.print(F("##"));
    Serial.print(String(init_current_cmd+1));
    Serial.print(F("番目のコマンド："));
    Serial.print(String(distance));
    Serial.print(F("m　前に進む "));
    Serial.print(F("(上限速度："));        
    Serial.print(String(velocity));
    Serial.println(F("rpm)"));        
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void susumu(float distance)
{
  go_forward(distance,EXCEPTION_NO);
}
void susumu(float distance,float max_velocity){
  go_forward(distance,max_velocity);
  
}

void turn_counter_clockwise(float degree,float max_velocity)
{
  if (cmd_init == false)
  {
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
    if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }   
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(-target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    Serial.print(F("##"));
    Serial.print(String(init_current_cmd+1));
    Serial.print(F("番目のコマンド："));
    Serial.print(String(degree));
    Serial.print(F("度　左回り "));
    Serial.print(F("(上限速度："));        
    Serial.print(String(velocity));
    Serial.println(F("rpm)"));    

    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void hidarimawari(float degree)
{
  turn_counter_clockwise(degree,EXCEPTION_NO);
}
void hidarimawari(float degree,float max_velocity)
{
  turn_counter_clockwise(degree,max_velocity);
}
void hidarimawari90()
{
  hidarimawari(90,EXCEPTION_NO);
}
void hidarimawari90(float max_velocity)
{
  hidarimawari(90,max_velocity);
}
void hidarimawari45()
{
  hidarimawari(45,EXCEPTION_NO);
}
void hidarimawari45(float max_velocity)
{
  hidarimawari(45,max_velocity);
}
void hidarimawari180()
{
  hidarimawari(180,EXCEPTION_NO);
}
void hidarimawari180(float max_velocity)
{
  hidarimawari(180,max_velocity);
}

void reset_arduino_mode_flags()
{
  //Serial.println(F("reset_arduino_mode_flags"));// デバッグ用確認
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
  send_spi(arduino_flag_cmd_matrix[current_cmd][1]);
  spi_done = true;
}

void cmd_end(MotorController motor_controllers[2])  // もっとマシな名前を考える
{

  if (cmd_init == false)
  {
    // 初回起動時の処理
    //Serial.println("CMD_SIZE: " + String(CMD_SIZE));
    // while (1);
    if (init_current_cmd >= CMD_SIZE)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println(F("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。")); // ココには至らないはず
      while (1);
    }


    // 初回起動時の処理をここで無効化
    reset_pid_gain(motor_controllers);
    Serial.println(F("###   コマンド準備完了    ###"));
    Serial.println(F("##########################"));
    Serial.println(F("###   コマンド実行開始    ###"));

    cmd_init = true;   // 最初の一回だけ。全部のコマンドが終了したとき、最初のコマンドに戻すときにリセット。それは未実装。
  }
  else
  {
    // 通常ループ時の処理
    // すべてのコマンドが終了しているか判定
  }
}

void check_achievement_wait_time_cmd(MotorController motor_controllers[2])
{
  //Serial.println("curennt time: " + String(micros()) + ", target_time: " + String(target_wait_time));
  if (target_wait_time < micros())
  {
    stop_motor_immediately(motor_controllers);
    wait_done = true;
  }
}

void cmd_manager_flags_init(MotorController motor_controllers[2])
{
  //Serial.println(F("#     CMD_manager_init     #"));
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
    //end_arduino_mode = true;  // 途中までコマンドを実行する場合はRCMODEに戻す。だが、強制終了することにした。
    Serial.print(F("init_current_cmd: ")); 
    Serial.println(String(init_current_cmd));
    Serial.println(F("コマンドの上限数以上にコマンドを設定しています。強制終了。"));
    while (1);
  }


  // 実行しないコマンドのフラグを処理（カウントのコマンド→時間やボタンはやらない）
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
    //delay(500);

    while (1);
  }
}


//未使用関数：プロトタイプ宣言も未実施
/*
String get_send_cmd_string()
{
  String send_msg = String(motor_controllers[MOTOR_LEFT].getCount()) +
                    "," +
                    String(motor_controllers[MOTOR_RIGHT].getCount());
  //Serial.println(send_msg);
  return send_msg;
}
*/
/*
  void UDP_read_write(int packetSize)
  {
  // 送信用のデータを整理
  char send_buff[UDP_BUFF_SIZE];
  String send_str = get_send_cmd_string();  // Stringクラスを使いたかったもので
  send_str.toCharArray(send_buff, UDP_BUFF_SIZE);
  display_UDP(packetSize, send_buff);
  //Serial.println(send_buff); // 確認用

  // バッファにたまったデータを抜き出して制御に適用
  Udp.read(packetBuffer, UDP_BUFF_SIZE);

  set_motor_cmd(packetBuffer);

  // 送信された相手に対して制御結果を投げ返す。したがって相手のIPアドレスの指定などは不要
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(send_buff);
  Udp.endPacket();
  }
*/
/*
  void UDP_FAIL_CHECK()
  {

  }
*/
/*
  void recieve_serial_cmd()
  {
  reciev_str = Serial.readStringUntil('\n');
  }
*/

/*
void set_motor_cmd(String reciev_str)
{
  if (reciev_str.length() > 0)
  {
    // 2輪の場合
    String sp_reciev_str[2];
    split(reciev_str, ',', sp_reciev_str);

    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(sp_reciev_str[i].toFloat());
    }
    //  モータに指令値を無事セットできたら、通信失敗カウンタをリセット
    //    毎回リセットすることで通常通信できる。
    //    10Hzで通信しているので、100msJOBでカウンタアップ。
    //    UDP_FAIL_COUNT = 0;
  }
  else
  {
    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(0.0);
    }
  }
}

*/
