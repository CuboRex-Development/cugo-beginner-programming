#include <Arduino.h>
#include "CugoArduinoMode.h"
#include <Servo.h>
#include "MotorController.h"
#include <SPI.h>

/***** ↓各ユーザーごとに設定してください↓ *****/

// シリアル通信での情報の表示有無
bool UDP_CONNECTION_DISPLAY = false;
bool ENCODER_DISPLAY = true;
bool PID_CONTROLL_DISPLAY = false;
bool FAIL_SAFE_DISPLAY = false;


// PID ゲイン調整
// L側
const float L_KP = 1.0;  //CuGoV3
const float L_KI = 0.02; //CuGoV3
const float L_KD = 0.1;  //CuGoV3
//const float L_KP = 1.0;
//const float L_KI = 0.06;
//const float L_KD = 0.1;

// R側
//const float R_KP = 1.0;
//const float R_KI = 0.06;
//const float R_KD = 0.1;
const float R_KP = 1.0;  //CuGoV3
const float R_KI = 0.02; //CuGoV3
const float R_KD = 0.1;  //CuGoV3

// ローパスフィルタ
const float L_LPF = 0.2;
const float R_LPF = 0.2;

// 回転方向ソフトウェア切り替え
const bool L_reverse = false;
const bool R_reverse = true;

// joshibi: L:True, R:false
// cugo-chan: L:false, R:True

/***** ↑各ユーザーごとに設定してください↑ *****/

unsigned long long current_time = 0, prev_time_10ms = 0, prev_time_100ms, prev_time_1000ms; // オーバーフローしても問題ないが64bit確保

#define MOTOR_NUM 2 // モータ接続数（最大4の予定）
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
MotorController motor_controllers[2];

#define PIN_UP(no)    upTime[no] = micros();
#define PIN_DOWN(no)  time[no] = micros() - upTime[no]

#define PWM_IN_MAX    3


/*    // メモリがカツカツになったときに死ぬ。
  typedef enum {
  RC_MODE = 0,  // RCで動くモード
  ARDUINO_MODE = 1   // Arduinoスケッチで動くモード
  } RUN_MODE;
*/
volatile unsigned long upTime[PWM_IN_MAX];
volatile unsigned long rcTime[PWM_IN_MAX];
volatile unsigned long time[PWM_IN_MAX];
int OLD_PWM_IN_PIN0_VALUE;   // プロポスティック入力値(L)
int OLD_PWM_IN_PIN1_VALUE;   // プロポスティック入力値(MODE)
int OLD_PWM_IN_PIN2_VALUE;   // プロポスティック入力値(R)
//RUN_MODE runMode = RC_MODE;  // 初回起動時はRC_MODE（無意識な暴走を防ぐため）
int runMode = ARDUINO_MODE;

int arduino_cmd_matrix[CMD_SIZE][6];
bool cmd_init = false;
int current_cmd = 0;
int init_current_cmd = 0;

long int target_count_L = 0;
long int target_count_R = 0;
long int target_wait_time = 0;
int button_push_count = 0;
bool button_enable = false;
bool cmd_L_back = false;
bool cmd_R_back = false;

bool cmd_exec = false;
bool count_done  = false;
bool wait_done   = false;
bool button_done = false;
bool spi_done    = false;

bool end_arduino_mode = false;

const float wheel_radius_l = 0.03858;
const float wheel_radius_r = 0.03858;
const float tread = 0.380;
const int encoder_resolution = 2048;

int migimawari_count90 = 70;
int hidarimawari_count90 = 70;
int migimawari_count45 = 33;
int hidarimawari_count45 = 33;
int migimawari_count180 = 160;
int hidarimawari_count180 = 160;

//たたまない？
// ピン変化割り込みの割り込み #TODO まだコードを畳められていない
ISR(PCINT2_vect)
{
  Serial.println("割り込み");
  if (OLD_PWM_IN_PIN0_VALUE != digitalRead(PWM_IN_PIN0))
  {
    if (LOW == OLD_PWM_IN_PIN0_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(0);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(0);
    }
    OLD_PWM_IN_PIN0_VALUE = OLD_PWM_IN_PIN0_VALUE ? LOW : HIGH;
  }

  if (OLD_PWM_IN_PIN1_VALUE != digitalRead(PWM_IN_PIN1))
  {  Serial.println("モード変更割り込み");

    if (LOW == OLD_PWM_IN_PIN1_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(1);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(1);
    }
    OLD_PWM_IN_PIN1_VALUE = OLD_PWM_IN_PIN1_VALUE ? LOW : HIGH;
  }

  if (OLD_PWM_IN_PIN2_VALUE != digitalRead(PWM_IN_PIN2))
  {
    if (LOW == OLD_PWM_IN_PIN2_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(2);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(2);
    }
    OLD_PWM_IN_PIN2_VALUE = OLD_PWM_IN_PIN2_VALUE ? LOW : HIGH;
  }
}

//たたまない？
int split(String data, char delimiter, String *dst)
{
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

//たたまない？
void init_PID()
{
  pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効

  // LEFTインスタンス有効化
  motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, L_reverse);
  // RIGHTインスタンス有効化
  motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, R_reverse);

  // エンコーダカウンタは純正のハードウェア割り込みピンを使用
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), leftEncHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), rightEncHandler, RISING);

  // 初期値でモータ指示。起動時に停止を入力しないと保護機能が働き、回りません。
  motor_direct_instructions(1500, 1500); //直接停止命令を出す
  delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}

//たたまない？
void leftEncHandler()
{
  motor_controllers[MOTOR_LEFT].updateEnc();
}

//たたまない？
void rightEncHandler()
{
  motor_controllers[MOTOR_RIGHT].updateEnc();
}

void display_speed()
{
  if (ENCODER_DISPLAY == true)
  {
    //Serial.println("DISPLAY MOTOR COUNTER & SPEED");
    //Serial.print("Mode:");
    //Serial.println(runMode);

    Serial.print("Encoder count (L/R):");
    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm());   // 制御量を見るため。開発用
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。開発用
    Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    Serial.print(",");
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());  //制御量を見るため。デバッグ用
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));


    Serial.print("PID CONTROL RPM(L/R):");
    Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。デバッグ用
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。デバッグ用
    Serial.print(",");
    Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());    //制御量を見るため。デバッグ用

    Serial.println(""); // 改行
  }
}

void display_target_rpm()
{
  Serial.println("target_rpm[L]:" + String(motor_controllers[0].getTargetRpm()));
  Serial.println("target_rpm[R]:" + String(motor_controllers[1].getTargetRpm()));
}

//たたまない？
void display_PID()
{
  if (PID_CONTROLL_DISPLAY == true)
  {
    Serial.println("DISPLAY PID PRAMETER");

    Serial.print("Encoder count (L/R): " + String(motor_controllers[MOTOR_LEFT].getCount()) + ",");
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    //Serial.print(",");
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));

    Serial.print("Target RPM (L/R): " + String(motor_controllers[MOTOR_LEFT].getTargetRpm()) + ",");
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getTargetRpm()));
    //Serial.print(",");
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getTargetRpm()));

    Serial.print("PID CONTROL RPM(L/R):" + String(motor_controllers[MOTOR_LEFT].getRpm()) + ",");
    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。デバッグ用
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。デバッグ用
    //Serial.print(",");
    Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());    //制御量を見るため。デバッグ用

    Serial.println("PID controll gain = P x kp + I x ki + D x kd");


    Serial.print("[L]: " + String(motor_controllers[MOTOR_LEFT].getSpeed()) + " = ");
    Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_P()) + " x " + String(L_KP) + " + ");
    Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_I()) + " x " + String(L_KI) + " + ");
    Serial.println(String(motor_controllers[MOTOR_LEFT].getPID_D()) + " x " + String(L_KD) + " + ");
    Serial.print("[R]: " + String(motor_controllers[MOTOR_RIGHT].getSpeed()) + " = ");
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_P()) + " x " + String(L_KP) + " + ");
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_I()) + " x " + String(L_KI) + " + ");
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getPID_D()) + " x " + String(L_KD) + " + ");

    Serial.println("");
  }
}

//たたまない？
void arduino_mode()
{
  digitalWrite(LED_BUILTIN, HIGH);  // ARDUINO_MODEでLED点灯
  CMD_EXECUTE();
  for (int i = 0; i < MOTOR_NUM; i++) { // 4輪でも使えるように
    motor_controllers[i].driveMotor();
  }
}

//たたまない？
void rc_mode()
{
  digitalWrite(LED_BUILTIN, LOW); // RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  motor_direct_instructions(rcTime[0], rcTime[2]);
  Serial.println("input cmd:" + String(rcTime[0]) + ", " + String(rcTime[2]));
}

//たたまない？
void check_mode_change()
{
  noInterrupts();      //割り込み停止
  rcTime[0] = time[0];
  rcTime[1] = time[1];
  rcTime[2] = time[2];
  interrupts();     //割り込み開始

  //Serial.println("runMode: " + String(runMode));
  // chBでモードの切り替え
  if (ARDUINO_MODE_IN < rcTime[1])
  {
    if (runMode != ARDUINO_MODE)
    { // モードが変わった時(RC→ARDUINO)
      //motor_direct_instructions(1500, 1500); //直接停止命令を出す
    }
    reset_arduino_mode_flags();
    reset_pid_gain();
    runMode = ARDUINO_MODE;
  }
  else if (ARDUINO_MODE_OUT > rcTime[1])
  {
    if (runMode != RC_MODE)
    { // モードが変わった時(ARDUINO→RC)
    }
    runMode = RC_MODE;
  }

  // モードごとの処理
  //Serial.println("runMode: " + String(runMode));
  //Serial.println("hoge");
  if (ARDUINO_MODE == runMode)
  {
    //Serial.println("### ARDUINO MODE ###");

    arduino_mode();
  }
  else
  {
    //Serial.println("### RC MODE ###");
    reset_arduino_mode_flags();
    rc_mode();
  }
}
//たたまない？
void motor_direct_instructions(int left, int right)
{
  motor_controllers[MOTOR_LEFT].servo_.writeMicroseconds(left);
  motor_controllers[MOTOR_RIGHT].servo_.writeMicroseconds(right);
  //Serial.println("Letf: " + String(left) + ", " + String(right));
}
//たたまない？
void stop_motor_immediately()
{
  //set_motorにしないのはセットすることでUDP受け取れないコマンドがリセットされてしまう。
  motor_controllers[0].setTargetRpm(0.0);
  motor_controllers[1].setTargetRpm(0.0);
  motor_direct_instructions(1500, 1500);
}
//たたまない？
void job_10ms()
{
  check_mode_change();
}
//たたまない？
void job_100ms()
{
  //  check_failsafe();
  //display_speed();
  //display_PID();
  //display_failsafe(FAIL_SAFE_DISPLAY,runMode);
}
//たたまない？
void job_1000ms()
{
  //display_nothing(UDP_CONNECTION_DISPLAY,ENCODER_DISPLAY,PID_CONTROLL_DISPLAY);
}

/* 工事中
  void recieve_serial_cmd()
  {
  reciev_str = Serial.readStringUntil('\n');
  }
*/
//たたまない？
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
    /*  モータに指令値を無事セットできたら、通信失敗カウンタをリセット
        毎回リセットすることで通常通信できる。
        10Hzで通信しているので、100msJOBでカウンタアップ。
    */
    //    UDP_FAIL_COUNT = 0;
  }
  else
  {
    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(0.0);
    }
  }
}
//たたまない？
String get_send_cmd_string()
{
  String send_msg = String(motor_controllers[MOTOR_LEFT].getCount()) +
                    "," +
                    String(motor_controllers[MOTOR_RIGHT].getCount());
  //Serial.println(send_msg);
  return send_msg;
}
//たたまない？
void reset_pid_gain()
{
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    motor_controllers[i].reset_PID_param();
  }
}

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


  void UDP_FAIL_CHECK()
  {

  }
*/
//たたまない？
void view_flags()
{
  //Serial.println("");
  //Serial.println("FLAGS");
  //Serial.println("cmd_init: " + String(cmd_init));
  //Serial.println("current_cmd: " + String(current_cmd));
  //Serial.println("target_count_L: " + String(target_count_L));
  //Serial.println("target_count_R: " + String(target_count_R));
  //Serial.println("cmd_exec: " + String(cmd_exec));
  //Serial.println("count_done: " + String(count_done));
  //Serial.println("wait_done: " + String(wait_done));
  //Serial.println("button_done: " + String(button_done));
  //Serial.println("spi_done: " + String(spi_done));
  //Serial.println("target_wait_time: " + String(target_wait_time));
  //Serial.println("button_push_count: " + String(button_push_count));
  //Serial.println("button_enable: " + String(button_enable));
  //Serial.println("");
}
//たたまない？
void reset_arduino_mode_flags()
{
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
  init_ARDUINO_CMD(arduino_cmd_matrix);
}
//たたまない？
void cmd_manager_flags_init()
{
  //Serial.println("#     CMD_manager_init     #");
  // これからコマンドを実行するときの処理
  cmd_exec = true;
  count_done = false;
  wait_done = false;
  button_done = false;
  spi_done = false;
  cmd_L_back = false;
  cmd_R_back = false;

  //Serial.println("init_current_cmd: " + String(init_current_cmd));
  //while(1);
  if (init_current_cmd >= CMD_SIZE - 1)
  {
    //end_arduino_mode = true;  // 途中までコマンドを実行する場合はRCMODEに戻す。だが、強制終了することにした。
    Serial.println("コマンドの上限数以上にコマンドを設定しています。強制終了。");
    while (1);
  }


  // 実行しないコマンドのフラグを処理（カウントのコマンド→時間やボタンはやらない）
  if (arduino_cmd_matrix[current_cmd][0] == EXCEPTION_NO && arduino_cmd_matrix[current_cmd][1] == EXCEPTION_NO)
    count_done = true;

  if (arduino_cmd_matrix[current_cmd][2] == EXCEPTION_NO)
    wait_done = true;

  if (arduino_cmd_matrix[current_cmd][3] != 255)
    button_done = true;

  Serial.println(arduino_cmd_matrix[current_cmd][3]);
  if (arduino_cmd_matrix[current_cmd][3] < 1 || 7 < arduino_cmd_matrix[current_cmd][3])
    spi_done = true;


  Serial.println("Current cmd: " + String(current_cmd));
  view_flags();
  Serial.println("");


  // ここに入ったら終了。
  if (count_done == true && wait_done == true && button_done == true && spi_done == true)
  {
    Serial.println("すべてのコマンド実行完了。または、初期化のままでコマンド入力できていない。");
    view_flags();
    end_arduino_mode = true;
  }
  // ここに入ったら誤作動
  if (count_done == false && wait_done == false || count_done == false && button_done == false || \
      count_done == false && wait_done == false || wait_done == false && button_done == false || \
      wait_done == false && spi_done == false || button_done == false && spi_done == false)
  {
    Serial.println("## BAD CASE!! ##");
    view_flags();
    view_arduino_cmd_matrix(arduino_cmd_matrix);
    Serial.println("複数コマンド入力。入力関数に不備があるか、コマンドを上書きしている可能性あり。");
    stop_motor_immediately();
    //delay(500);

    while (1);
  }
}
//たたまない？
void set_go_forward_cmd()
{
  target_count_L = motor_controllers[MOTOR_LEFT].getCount() + arduino_cmd_matrix[current_cmd][0];
  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String(motor_controllers[MOTOR_LEFT].getCount()) + " + " + String(arduino_cmd_matrix[current_cmd][0]));
  //while(1);
  if (arduino_cmd_matrix[current_cmd][0] >= 0) {
    cmd_L_back = false;
  } else {
    cmd_L_back = true;
  }

  target_count_R = motor_controllers[MOTOR_RIGHT].getCount() + arduino_cmd_matrix[current_cmd][1];
  if (arduino_cmd_matrix[current_cmd][1] >= 0) {
    cmd_R_back = false;
  } else {
    cmd_R_back = true;
  }
}
//たたまない？
void set_wait_time_cmd()
{
  //target_wait_time = micros() + arduino_cmd_matrix[current_cmd][2] * 1000;
  target_wait_time = arduino_cmd_matrix[current_cmd][2];
  target_wait_time = target_wait_time * 1000; // arduino_cmd_matrix[][]が16bitのため、long intのこちらで1000倍処理
  target_wait_time = target_wait_time + micros();
  //   Serial.println(String(target_wait_time));
  // while(1);
  //  target_wait_time = micros() + 3000000;
}
//たたまない？
void set_button_cmd()
{
  button_push_count = 0;
  button_enable = 0;
}
//たたまない？
void check_achievement_go_forward_cmd()
{
  bool L_done = false;
  bool R_done = false;

  // L側目標達成チェック
  if (cmd_L_back == false) {
    if (target_count_L < motor_controllers[MOTOR_LEFT].getCount())
      L_done = true;
    //Serial.println("L_DONE!");
    //motor_controllers[MOTOR_LEFT].setTargetRpm(0);
  } else {
    if (target_count_L > motor_controllers[MOTOR_LEFT].getCount())
      L_done = true;
    //Serial.println("L_DONE!");
    //motor_controllers[MOTOR_LEFT].setTargetRpm(0);
  }

  // R側目標達成チェック
  if (cmd_R_back == false) {
    if (target_count_R < motor_controllers[MOTOR_RIGHT].getCount())
      R_done = true;
    //Serial.println("R_DONE!");
    //motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
  } else {
    if (target_count_R > motor_controllers[MOTOR_RIGHT].getCount())
      R_done = true;
    //Serial.println("R_DONE!");
    //motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
  }

  if (L_done == true)
  {
    motor_controllers[MOTOR_LEFT].setTargetRpm(0);
    Serial.println("L_DONE!");
  }
  if (R_done == true)
  {
    motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
    Serial.println("R_DONE!");
  }
  Serial.println("target_count L/R: " + String(target_count_L) + " / " + String(target_count_R));
  Serial.println("L/R DONE!: " + String(L_done) + " / " + String(R_done));

  // L/R達成していたら終了
  if (L_done == true && R_done == true)
  {
    stop_motor_immediately();
    count_done = true;
  }
}
//たたまない？
void check_achievement_wait_time_cmd()
{
  //Serial.println("curennt time: " + String(micros()) + ", target_time: " + String(target_wait_time));
  if (target_wait_time < micros())
  {
    stop_motor_immediately();
    wait_done = true;
  }
}
//たたまない？
void cmd_manager()
{
  if (cmd_init == false)
  {
    // 初回起動時の処理
    //Serial.println("cmd_init");

  }
  else
  {
    // 通常ループ時の処理

    // コマンド実行直前処理
    if (cmd_exec == false)
    {
      cmd_manager_flags_init();
      // 前後進の指示をセット
      if (count_done == false)
      {
        set_go_forward_cmd();
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

      //Serial.println("Start command: " + String(current_cmd));
      //view_flags();
      //Serial.println("");

    }

    // コマンド実行中処理
    if (cmd_exec == true) // elseにしないのは、スタートしてから実行したいため
    {
      //Serial.println("#     cmd_exec_main    #");
      // コマンドで設定された速度に設定
      motor_controllers[MOTOR_LEFT].setTargetRpm(arduino_cmd_matrix[current_cmd][4]);
      motor_controllers[MOTOR_RIGHT].setTargetRpm(arduino_cmd_matrix[current_cmd][5]);

      // 成功条件の確認
      // if conuntの成功条件
      // テストで逆回転はなし。別々の判定もなし。
      if (count_done == false)
        check_achievement_go_forward_cmd();

      if (wait_done == false)
        check_achievement_wait_time_cmd();

      if (button_done == false)
        check_achievement_button_cmd();

      if (spi_done == false)
        check_achievement_spi_cmd();


      // if waitの成功条件
      // if buttonの成功条件

      if (count_done == true && wait_done == true && button_done == true)
      {
        // モータを止める
        cmd_exec = false;
        current_cmd++;

        if (end_arduino_mode == true)
        {
          reset_arduino_mode_flags();
          end_arduino_mode = false;
          runMode = RC_MODE;
        }
      }


    }


  }

}
//たたまない？
void check_achievement_button_cmd()
{
  if (digitalRead(CMD_BUTTON_PIN) == 0)
  {
    button_push_count++;
  } else {
    button_push_count = 0;
  }

  if (button_push_count >= 5) // 実測で50ms以上長いと小刻みに押したとき反応しないと感じてしまう。
  {
    stop_motor_immediately();
    button_done = true;
  }
}
//たたまない？
void check_achievement_spi_cmd()
{
  send_spi(arduino_cmd_matrix[current_cmd][3]);
  spi_done = true;
}
//たたまない？
void cmd_end()  // もっとマシな名前を考える
{

  if (cmd_init == false)
  {
    // 初回起動時の処理
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    //Serial.println("CMD_SIZE: " + String(CMD_SIZE));
    // while (1);
    if (init_current_cmd >= CMD_SIZE)
    {
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"); // ココには至らないはず
      while (1);
    }


    // 初回起動時の処理をここで無効化
    cmd_init = true;   // 最初の一回だけ。全部のコマンドが終了したとき、最初のコマンドに戻すときにリセット。それは未実装。
  }
  else
  {
    // 通常ループ時の処理


    // すべてのコマンドが終了しているか判定



  }
}



//たたまない？
void susumu(float distance)
{
  go_forward(distance);
}
//たたまない？
void go_forward(float distance)
{
  if (cmd_init == false)
  {
    // 初回起動時の処理
    calc_necessary_count(distance,&target_count_L,&target_count_R,tread,encoder_resolution,wheel_radius_l,wheel_radius_r);
    float velocity = 90.0;
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(init_current_cmd, target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, velocity, arduino_cmd_matrix); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
//たたまない？
void sagaru(float distance)
{
  go_backward(distance);
}

void go_backward(float distance)
{
  if (cmd_init == false)
  {
    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);
    }

    // 初回起動時の処理
    calc_necessary_count(distance,&target_count_L,&target_count_R,tread,encoder_resolution,wheel_radius_l,wheel_radius_r);
    float velocity = 90.0;
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(init_current_cmd, -target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, -velocity, arduino_cmd_matrix); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void migimawari(float degree)
{
  turn_clockwise(degree);
}

void migimawari90()
{
  migimawari(migimawari_count90);
}

void migimawari45()
{
  migimawari(migimawari_count45);
}

void migimawari180()
{
  migimawari(migimawari_count180);
}

void turn_clockwise(float degree)
{

  if (cmd_init == false)
  {
    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);
    }

    // 初回起動時の処理
    calc_necessary_rotate(degree,&target_count_L,&target_count_R,tread,encoder_resolution,wheel_radius_l,wheel_radius_r);
    float velocity = 90.0;
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(init_current_cmd, target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, -velocity,arduino_cmd_matrix); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void hidarimawari(float degree)
{
  turn_counter_clockwise(degree);
}

void hidarimawari90()
{
  hidarimawari(hidarimawari_count90);
}

void hidarimawari45()
{
  hidarimawari(hidarimawari_count45);
}


void hidarimawari180()
{
  hidarimawari(hidarimawari_count180);
}


void turn_counter_clockwise(float degree)
{
  if (cmd_init == false)
  {
    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);
    }

    // 初回起動時の処理
    calc_necessary_rotate(degree,&target_count_L,&target_count_R,tread,encoder_resolution,wheel_radius_l,wheel_radius_r);
    float velocity = 90.0; // 単純版関数なので、速度は固定
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(init_current_cmd, -target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, velocity,arduino_cmd_matrix); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
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

void wait_time(int milisec)
{
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);
    }

    // 初回起動時の処理
    // テスト(3000msの待機を固定して全体の動作テスト）
    set_arduino_cmd_matrix(init_current_cmd, EXCEPTION_NO, EXCEPTION_NO, milisec, EXCEPTION_NO, 0, 0,arduino_cmd_matrix); // ここではテストで3000ms間、rpmを0,0(停止)にセット
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void botan()
{
  wait_button(&init_current_cmd ,arduino_cmd_matrix[CMD_SIZE][6],cmd_init);

}

void button()
{
  wait_button(&init_current_cmd ,arduino_cmd_matrix[CMD_SIZE][6],cmd_init);
}


void setup()
{
  Serial.begin(115200);
  init_PID();
  init_KOPROPO(runMode,OLD_PWM_IN_PIN0_VALUE,OLD_PWM_IN_PIN1_VALUE,OLD_PWM_IN_PIN2_VALUE);
  //  init_UDP();
  init_ARDUINO_CMD(arduino_cmd_matrix);
  init_SPI();
}

void loop()
{
  current_time = micros();  // オーバーフローまで約40分

  if (current_time - prev_time_10ms > 10000) // TODO 10秒で1msくらいズレる
  {
    job_10ms();
    prev_time_10ms = current_time;
  }

  if (current_time - prev_time_100ms > 100000) // TODO 1秒で1msくらいズレる
  {
    job_100ms();
    prev_time_100ms = current_time;
  }

  if (current_time - prev_time_1000ms > 1000000)
  {
    job_1000ms();
    prev_time_1000ms = current_time;
  }

  /*  // シリアル通信でコマンドを受信するとき #工事中(フラグで22年度中に切り替え可の予定)
    if (Serial.available() > 0)
    {
      recieve_serial_cmd();
    }
  */

  /*
    // UDP通信でコマンドを受信するとき（標準）
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      UDP_read_write(packetSize);
    }
  */
}

/* 使えるコマンドリスト */
/*
 * susumu(1.0);　入力した数字m 分だけ前に進む（テスト時は3m以下にしてください。止まりません）
 * sagaru(1.0);　入力した数字m 分だけ前に進む（テスト時は3m以下にしてください。止まりません）
 * migimawari45(); 45度右に回転する
 * migimawari90(); 90度右に回転する
 * migimawari180(); 180度右に回転する
 * hidarimawari45(); 45度右に回転する
 * hidarimawari90(); 90度右に回転する
 * hidarimawari180(); 180度右に回転する
 * matsu(1000); 入力した数字ミリ秒待機する。1000ミリ秒＝1秒。
 * button(); ボタンを押すまで待ちます。
 * 
*/


void CMD_EXECUTE()
{
  //Serial.println("Current command: " + String(current_cmd));
  cmd_manager();  // おまじない

  // ここから↓を改造していこう！

  button();

  susumu(1.0);
  matsu(1000);
  migimawari90();
  matsu(1000);

  susumu(1.0);
  matsu(1000);
  migimawari90();
  matsu(1000);

  susumu(1.0);
  matsu(1000);
  migimawari90();
  matsu(1000);

  susumu(1.0);
  matsu(1000);
  migimawari90();
  matsu(1000);
  // ここから↑を改造していこう！

  cmd_end();      // おまじない
  //view_arduino_cmd_matrix();
}
