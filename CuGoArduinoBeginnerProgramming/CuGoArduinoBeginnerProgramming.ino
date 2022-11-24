/*
 * Copyright [2022] [CuboRex.Inc]
 * Licensed under the Apache License, Version 2.0 (the “License”);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <Arduino.h>
#include "CugoArduinoMode.h"
#include <Servo.h>
#include "MotorController.h"
#include <SPI.h>

/***** ↓各ユーザーごとに設定してください↓ *****/
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

/***** ↑各ユーザーごとに設定してください↑ *****/


MotorController motor_controllers[2];//利用するモータの数（インスタンス化するオブジェクト数）
//

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


//// PID位置制御のデータ格納
float l_count_prev_i_ = 0;
float l_count_prev_p_ = 0;
float r_count_prev_i_ = 0;
float r_count_prev_p_ = 0;
float l_count_gain = 0;
float r_count_gain = 0;

ISR(PCINT2_vect)
{
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
  {  
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
  motor_direct_instructions(1500, 1500,motor_controllers); //直接停止命令を出す
  delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}

void leftEncHandler()
{
  motor_controllers[MOTOR_LEFT].updateEnc();
}

void rightEncHandler()
{
  motor_controllers[MOTOR_RIGHT].updateEnc();
}

void arduino_mode()
{
  digitalWrite(LED_BUILTIN, HIGH);  // ARDUINO_MODEでLED点灯
  CMD_EXECUTE();
  for (int i = 0; i < MOTOR_NUM; i++) { // 4輪でも使えるように
    motor_controllers[i].driveMotor();
  }
}

void check_mode_change()
{
  noInterrupts();      //割り込み停止
  rcTime[0] = time[0];
  rcTime[1] = time[1];
  rcTime[2] = time[2];
  interrupts();     //割り込み開始

  //Serial.println(F("runMode: ") + String(runMode));
  // chBでモードの切り替え
  if (ARDUINO_MODE_IN < rcTime[1])
  {
    if (runMode != ARDUINO_MODE)
    { // モードが変わった時(RC→ARDUINO)
      Serial.println(F("##########################"));                  
      Serial.println(F("### モード:ARDUINO_MODE ###"));
      Serial.println(F("##########################"));            
      //motor_direct_instructions(1500, 1500,,motor_controllers); //直接停止命令を出す
      stop_motor_immediately(motor_controllers);
      reset_arduino_mode_flags();
      reset_pid_gain(motor_controllers);
    }
    runMode = ARDUINO_MODE;
  }
  else if (ARDUINO_MODE_OUT > rcTime[1])
  {
    if (runMode != RC_MODE)
    { // モードが変わった時(ARDUINO→RC)
      Serial.println(F("##########################"));                  
      Serial.println(F("###   モード:RC_MODE    ###"));
      Serial.println(F("##########################"));            
      reset_arduino_mode_flags();
    }
    runMode = RC_MODE;
  }
  
  if (ARDUINO_MODE == runMode)
  {
    //Serial.println(F("### ARDUINO MODE ###"));
    arduino_mode();
  }
  else
  {
    //Serial.println(F("### RC MODE ###"));
    rc_mode(rcTime,motor_controllers);
  }
}

void job_10ms()
{
  check_mode_change();
}

void job_100ms()
{
  display_speed(motor_controllers,ENCODER_DISPLAY);
  display_target_rpm(motor_controllers,ENCODER_DISPLAY);
  display_PID(motor_controllers,PID_CONTROLL_DISPLAY);
  display_failsafe(FAIL_SAFE_DISPLAY,runMode);
}
void job_1000ms()
{
  //display_nothing(UDP_CONNECTION_DISPLAY,ENCODER_DISPLAY,PID_CONTROLL_DISPLAY);
}

void check_achievement_go_forward_cmd()
{
  bool L_done = false;
  bool R_done = false;

  // L側目標達成チェック
  if (cmd_L_back == false) {
    if (target_count_L < motor_controllers[MOTOR_LEFT].getCount())
      L_done = true;
    //Serial.println(F("L_DONE!"));
    //motor_controllers[MOTOR_LEFT].setTargetRpm(0);
  } else {
    if (target_count_L > motor_controllers[MOTOR_LEFT].getCount())
      L_done = true;
    //Serial.println(F("L_DONE!"));
    //motor_controllers[MOTOR_LEFT].setTargetRpm(0);
  }

  // R側目標達成チェック
  if (cmd_R_back == false) {
    if (target_count_R < motor_controllers[MOTOR_RIGHT].getCount())
      R_done = true;
    //Serial.println(F("R_DONE!"));
    //motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
  } else {
    if (target_count_R > motor_controllers[MOTOR_RIGHT].getCount())
      R_done = true;
    //Serial.println(F("R_DONE!"));
    //motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
  }

  if (L_done == true)
  {
    motor_controllers[MOTOR_LEFT].setTargetRpm(0);
    //Serial.println(F("L_DONE!"));
  }
  if (R_done == true)
  {
    motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
    //Serial.println(F("R_DONE!"));
  }

  // L/R達成していたら終了
  if (L_done == true && R_done == true)
  {
    stop_motor_immediately(motor_controllers);
    count_done = true;
    Serial.print(F("###"));
    if(current_cmd < 9)
        Serial.print(F("0"));

    Serial.print(String(current_cmd+1));
    Serial.println(F("番目のコマンド：終了  ###"));    
  }
}
void cmd_manager()
{
  if (cmd_init == false)
  {
    // 初回起動時の処理
  //if (init_current_cmd == 0)
  //{
    Serial.println(F("##########################"));
    Serial.println(F("###   コマンド準備開始    ###"));
    
  //}  
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
      //view_flags();
      //Serial.println(F(""));
    }

    // コマンド実行中処理
    if (cmd_exec == true) // elseにしないのは、スタートしてから実行したいため
    {
      //Serial.println(F("#     cmd_exec_main    #"));
      // コマンドで設定された速度に設定
      //ここで速度制御：setTargetRpm目標速度の設定

      //// PID位置制御の制御値
      float l_count_p;  // P制御値
      float l_count_i;  // I制御値
      float l_count_d;  // D制御値
      float r_count_p;  // P制御値
      float r_count_i;  // I制御値
      float r_count_d;  // D制御値
      if(arduino_flag_cmd_matrix[current_cmd][2] == 0 && arduino_flag_cmd_matrix[current_cmd][3] == 0)
      {
        //停止しているだけの時
        motor_controllers[MOTOR_LEFT].setTargetRpm(arduino_flag_cmd_matrix[current_cmd][2]);
        motor_controllers[MOTOR_RIGHT].setTargetRpm(arduino_flag_cmd_matrix[current_cmd][3]);

      } else{
        // 各制御値の計算
        l_count_p = arduino_count_cmd_matrix[current_cmd][0] - motor_controllers[MOTOR_LEFT].getCount();
        l_count_i = l_count_prev_i_ + l_count_p;
        l_count_d = l_count_p - l_count_prev_p_;
        r_count_p = arduino_count_cmd_matrix[current_cmd][1] - motor_controllers[MOTOR_RIGHT].getCount();
        r_count_i = r_count_prev_i_ + r_count_p;
        r_count_d = r_count_p - r_count_prev_p_;

        l_count_i = min( max(l_count_i,-L_MAX_COUNT_I),L_MAX_COUNT_I);        
        r_count_i = min( max(r_count_i,-R_MAX_COUNT_I),R_MAX_COUNT_I);
/*
        if(r_count_i > R_MAX_COUNT_I){
          r_count_i = R_MAX_COUNT_I;
        }else if(r_count_i < -R_MAX_COUNT_I){
          r_count_i = -R_MAX_COUNT_I;
        }else{
          //通常ループ
        }
 */      
        // PID制御
        l_count_gain = l_count_p * L_COUNT_KP + l_count_i * L_COUNT_KI + l_count_d * L_COUNT_KD;  
        r_count_gain = r_count_p * R_COUNT_KP + r_count_i * R_COUNT_KI + r_count_d * R_COUNT_KD;  
        // prev_ 更新
        l_count_prev_p_ = l_count_p;
        l_count_prev_i_ = l_count_i;
        r_count_prev_p_ = r_count_p;
        r_count_prev_i_ = r_count_i;

        l_count_gain = min( max(l_count_gain,-MAX_MOTOR_RPM),MAX_MOTOR_RPM);//モーターの速度上限        
        r_count_gain = min( max(r_count_gain,-MAX_MOTOR_RPM),MAX_MOTOR_RPM);//モーターの速度上限        
        l_count_gain = min( max(l_count_gain,-fabsf(arduino_flag_cmd_matrix[current_cmd][2])),fabsf(arduino_flag_cmd_matrix[current_cmd][2]));//ユーザ設定の速度上限        
        r_count_gain = min( max(r_count_gain,-fabsf(arduino_flag_cmd_matrix[current_cmd][3])),fabsf(arduino_flag_cmd_matrix[current_cmd][3]));//ユーザ設定の速度上限        

           
        //位置制御
        motor_controllers[MOTOR_LEFT].setTargetRpm(l_count_gain);
        motor_controllers[MOTOR_RIGHT].setTargetRpm(r_count_gain);
        //Serial.print(F("gain:l/r "));
        //Serial.print(String(l_count_gain));
        //Serial.print(F(","));
        //Serial.println(String(r_count_gain));
        /*
       //使うもの
       arduino_count_cmd_matrix[current_cmd][0] //L側の目標カウント数
       arduino_count_cmd_matrix[current_cmd][1] //R側の目標カウント数
       motor_controllers[MOTOR_LEFT].getCount() //Lの現在のカウント数 
       motor_controllers[MOTOR_LEFT].getCount() //Rの現在のカウント数 
       */
        }
      
      // 成功条件の確認
      // if conuntの成功条件
      // テストで逆回転はなし。別々の判定もなし。
      if (count_done == false)
        check_achievement_go_forward_cmd();

      if (wait_done == false)
        check_achievement_wait_time_cmd(motor_controllers);

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
//
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
    stop_motor_immediately(motor_controllers);
    button_done = true;
    Serial.print(F("###"));
    if(current_cmd < 9)
        Serial.print(F("0"));

    Serial.print(String(current_cmd+1));
    Serial.println(F("番目のコマンド：終了  ###"));    
  }
}
//
void setup()
{
  Serial.begin(115200);

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
  init_PID();
  init_KOPROPO(runMode,OLD_PWM_IN_PIN0_VALUE,OLD_PWM_IN_PIN1_VALUE,OLD_PWM_IN_PIN2_VALUE);
  //init_UDP();//存在しない関数
  init_ARDUINO_CMD();
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
 * susumu(1.0);　入力した()内の数字m 分だけ前に進む
 * //追加コマンド：susumu(1.0,90);速度上限90rpmで設定　距離が短いと上限速度に達さない場合もある
 * sagaru(1.0);　入力した()内の数字m 分だけ前に進む
 * migimawari45(); 45度右に回転する
 * //追加コマンド：migimawari45(90);入力した()内の速度上限で設定　最大180
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
  cmd_manager();  // おまじない

  // ここから↓を改造していこう！

  button();

  susumu(1.0);
  matsu(1000); 
  susumu(1.0,150);
  matsu(1000);
  migimawari90();
  matsu(1000);
  migimawari90(180);
  matsu(1000);
// sagaru(1.0,30);
//  matsu(1000);
//  migimawari(30,60);
//  matsu(1000);
//  sagaru(1.0);
//  matsu(1000);
//  hidarimawari90(10);
//  matsu(1000);
  //hidarimawari90();
  //matsu(1000);


  //susumu(1.0);
  //matsu(1000);
  //migimawari90();
  //matsu(1000);
  // ここから↑を改造していこう！

  cmd_end(motor_controllers);      // おまじない
  //view_arduino_cmd_matrix();
}
