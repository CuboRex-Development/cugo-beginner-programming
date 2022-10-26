#include "Arduino.h"
#include "CugoArduinoMode.h"
#include "MotorController.h"
#include <SPI.h>

long int arduino_cmd_matrix[CMD_SIZE][6];
int init_current_cmd = 0;


void init_SPI()
{
  Serial.println("init_SPI()");// デバッグ用確認
  SPI.begin();
  digitalWrite(SS, HIGH);
}

void send_spi(int mode) {
  Serial.println("send_spi()");// デバッグ用確認
  digitalWrite(SS, LOW);
  SPI.transfer(mode);
  digitalWrite(SS, HIGH);
}

void init_KOPROPO(int runMode,int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE)
{
  Serial.println("init_KOPROPO()");// デバッグ用確認
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
  arduino_cmd_matrix[init_current_cmd][0] = cmd_0;
  arduino_cmd_matrix[init_current_cmd][1] = cmd_1;
  arduino_cmd_matrix[init_current_cmd][2] = cmd_2;
  arduino_cmd_matrix[init_current_cmd][3] = cmd_3;
  arduino_cmd_matrix[init_current_cmd][4] = cmd_4;
  arduino_cmd_matrix[init_current_cmd][5] = cmd_5;
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
    Serial.println(arduino_cmd_matrix[i][0]);
    Serial.println(arduino_cmd_matrix[i][1]);
    Serial.println(arduino_cmd_matrix[i][2]);
    Serial.println(arduino_cmd_matrix[i][3]);
    Serial.println(arduino_cmd_matrix[i][4]);
    Serial.println(arduino_cmd_matrix[i][5]);
    Serial.println(i);
  }
  //while (1); //matrixの内容を見たいだけ
}

void display_failsafe(bool FAIL_SAFE_DISPLAY,int runMode)
{
  //Serial.println("display_failsafe");// デバッグ用確認
  if (FAIL_SAFE_DISPLAY == true)
  {
    Serial.println("DISPLAY FAIL SAFE PARAM");

    Serial.print("Mode(ARDUINO/RC): ");
    Serial.println(runMode);

    Serial.print("UDP recieve fail count: ");
    //    Serial.println(UDP_FAIL_COUNT);

    Serial.println("");
  }
}

void display_nothing(bool UDP_CONNECTION_DISPLAY,bool ENCODER_DISPLAY,bool PID_CONTROLL_DISPLAY)
{
  //Serial.println("display_nothing");// デバッグ用確認
  if (UDP_CONNECTION_DISPLAY == false && ENCODER_DISPLAY == false && PID_CONTROLL_DISPLAY == false)
  {
    Serial.println("Display item not set");
    Serial.println("Arduino is working...");
    Serial.println("");
  }
}

void spi_cmd(int spi_cmd_value,bool cmd_init)
{
  Serial.println("spi_cmd");// デバッグ用確認
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
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

void wait_time(int milisec,bool cmd_init)
{
  //Serial.println("wait_time");// デバッグ用確認
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);
    }

    // 初回起動時の処理
    // テスト(3000msの待機を固定して全体の動作テスト）
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    set_arduino_cmd_matrix(EXCEPTION_NO,EXCEPTION_NO, milisec, EXCEPTION_NO, 0, 0); // ここではテストで3000ms間、rpmを0,0(停止)にセット//EXCEPTION_NO
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void calc_necessary_rotate(float degree,long int *target_count_L,long int *target_count_R) // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
{
  //Serial.println("calc_necessary_rotate");// デバッグ用確認
  *target_count_L =  ((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_l * PI);
  *target_count_R = -((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_r * PI);
  //Serial.println("degree: " + String(degree));
  //Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * tread * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
  //while(1);
}


void calc_necessary_count(float distance,long int *target_count_L,long int *target_count_R) // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
{
  //Serial.println("calc_necessary_count");// デバッグ用確認  
  //  *target_count_L = distance * encoder_resolution / (2 * wheel_radius_l * PI);
  //  *target_count_R = distance * encoder_resolution / (2 * wheel_radius_r * PI);

  *target_count_L = distance / (2 * wheel_radius_l * PI);
  *target_count_R = distance / (2 * wheel_radius_r * PI);
  *target_count_L = *target_count_L * encoder_resolution;
  *target_count_R = *target_count_R * encoder_resolution;

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


void atamaopen(bool cmd_init)
{
  //Serial.println("atamaopen");// デバッグ用確認  
  spi_cmd(6,cmd_init);
}

void atamaclose(bool cmd_init)
{
  //Serial.println("atamaclose");// デバッグ用確認  
  spi_cmd(5,cmd_init);
}

void wait_button(bool cmd_init)
{
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);

    }
    // 初回起動時の処理
    // テスト(ボタンのフラグbutton_enable==1としてテスト）
    // メモリが足りないので、buttonとSPIを共用にする。
    set_arduino_cmd_matrix(EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, 255, 0, 0); // ここではテストで1を使用。//
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
void display_speed(MotorController motor_controllers[2],bool ENCODER_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println("display_speed");// デバッグ用確認    

  if (ENCODER_DISPLAY == true)
  {
    //Serial.println("DISPLAY MOTOR COUNTER & SPEED");
    //Serial.print("Mode:");
    //Serial.println(runMode);

    Serial.print("Encoder count (L/R):");
    //Serial.print(motor_controllers[0].getRpm());   // 制御量を見るため。開発用
    //Serial.print(motor_controllers[0].getSpeed()); // 制御量を見るため。開発用
    Serial.print(String(motor_controllers[0].getCount()));
    Serial.print(",");
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
  Serial.println("target_rpm[L]:" + String(motor_controllers[0].getTargetRpm()));
  Serial.println("target_rpm[R]:" + String(motor_controllers[1].getTargetRpm()));
  }
}
void display_PID(MotorController motor_controllers[2],bool PID_CONTROLL_DISPLAY) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println("display_PID");// デバッグ用確認    

  if (PID_CONTROLL_DISPLAY == true)
  {
    Serial.println("DISPLAY PID PRAMETER");

    Serial.print("Encoder count (L/R): " + String(motor_controllers[0].getCount()) + ",");
    //Serial.print(String(motor_controllers[0].getCount()));
    //Serial.print(",");
    Serial.println(String(motor_controllers[1].getCount()));

    Serial.print("Target RPM (L/R): " + String(motor_controllers[0].getTargetRpm()) + ",");
    //Serial.print(String(motor_controllers[0].getTargetRpm()));
    //Serial.print(",");
    Serial.println(String(motor_controllers[1].getTargetRpm()));

    Serial.print("PID CONTROL RPM(L/R):" + String(motor_controllers[0].getRpm()) + ",");
    //Serial.print(motor_controllers[0].getRpm()); // 制御量を見るため。デバッグ用
    //Serial.print(motor_controllers[0].getSpeed()); // 制御量を見るため。デバッグ用
    //Serial.print(",");
    Serial.println(motor_controllers[1].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[1].getSpeed());    //制御量を見るため。デバッグ用

    Serial.println("PID controll gain = P x kp + I x ki + D x kd");


    Serial.print("[L]: " + String(motor_controllers[0].getSpeed()) + " = ");
    Serial.print(String(motor_controllers[0].getPID_P()) + " x " + String(L_KP) + " + ");
    Serial.print(String(motor_controllers[0].getPID_I()) + " x " + String(L_KI) + " + ");
    Serial.println(String(motor_controllers[0].getPID_D()) + " x " + String(L_KD) + " + ");
    Serial.print("[R]: " + String(motor_controllers[1].getSpeed()) + " = ");
    Serial.print(String(motor_controllers[1].getPID_P()) + " x " + String(R_KP) + " + ");
    Serial.print(String(motor_controllers[1].getPID_I()) + " x " + String(R_KI) + " + ");
    Serial.println(String(motor_controllers[1].getPID_D()) + " x " + String(R_KD) + " + ");

    Serial.println("");
  }
}

int split(String data, char delimiter, String *dst)//dstは参照引き渡し
{
  Serial.println("split");// デバッグ用確認    
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
  //Serial.print(" motor_direct_instructions:: ");// デバッグ用確認    
  motor_controllers[0].servo_.writeMicroseconds(left);
  motor_controllers[1].servo_.writeMicroseconds(right);
  //Serial.println("l: " + String(left) + ",r " + String(right));
}
void rc_mode(volatile unsigned long rcTime[PWM_IN_MAX],MotorController motor_controllers[2])
{
  //Serial.print("rc_mode::");// デバッグ用確認    

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
