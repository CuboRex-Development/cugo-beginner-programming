/****************************************************************************
 * CugoArduinoBeginnerProgramming                                           *
 *                                                                          *
 * Copyright [2022] [CuboRex.Inc]                                           *
 * Licensed under the Apache License, Version 2.0 (the “License”);          *
 * you may not use this file except in compliance with the License.         *
 * You may obtain a copy of the License at                                  *
 *                                                                          * 
 * http://www.apache.org/licenses/LICENSE-2.0                               *
 *                                                                          *
 * Unless required by applicable law or agreed to in writing, software      *
 * distributed under the License is distributed on an “AS IS” BASIS,        *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 * See the License for the specific language governing permissions and      *
 * limitations under the License.                                           *
 ****************************************************************************/
/*****************CugoArduinoBeginnerProgramming ver1.00*********************/
/****************************************************************************
 * CugoArduinoBeginnerProgrammingをご利用される方へ
 *  スクロールして「Arduino学習用プログラミングはここから」からご確認ください。
 *  詳細はREADME.mdをご確認ください。
 ****************************************************************************/
//#include <Arduino.h>
//#include <Servo.h>
///#include <SPI.h>
#include "CugoArduinoMode.h"
//#include "MotorController.h"

//プロトタイプ宣言
void CMD_EXECUTE();

//利用するモーター数の宣言
//MotorController motor_controllers[2];
RPI_PICO_Timer ITimer0(0);

//初期設定
void setup()
{
  //Serial.begin(115200);
  Serial.begin(115200, SERIAL_8N1);//PCとの通信
  Serial1.begin(115200, SERIAL_8N1);//BLDCとの通信
  //pinMode(CUGO_CMD_BUTTON_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(CUGO_CMD_BUTTON_PIN), cugo_button_interrupt, CHANGE);

  //CBD = new c_BLDC_Driver();
  set_feedback(2, 0b10000001);//freq{0:10[hz] 1:50[hz] 2:100[hz]} kindof_data{0b1:Mode 0b10:CMD_RPM 0b100:CurrentRPM 0b1000:AveCurrentRPM 0b10000000:EncorderData}
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
  delay(1000); 
  if (ITimer0.attachInterruptInterval( FEEDBACK_DUTATION * 1000, TimerHandler0)) {
    Serial.print(F("Starting ITimer0 OK, millis() = "));
    Serial.println(millis());
  } else {
    Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }
  delay(1000);

  setControlMode(1);
  //Serial.println("Set CMD Mode.");
  delay(1000);



  init_display();//起動時モニタ表示
  //init_PID();
  //init_KOPROPO();
  init_ARDUINO_CMD();//ARDUINOコマンド初期化
  //init_SPI();
}

//loop内を繰り返し実行
void loop()
{
  current_time = micros();  // オーバーフローまで約40分
  if (current_time - prev_time_10ms > 10000) 
  {
    job_10ms();
    prev_time_10ms = current_time;
  }
  display_detail( );//必要に応じてCugoArduinoModeの5～8行目を変更
}

//割り込み処理
/*
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
*/

//自動走行モード(arduino_mode)の実行
void arduino_mode()
{
  //digitalWrite(LED_BUILTIN, HIGH);  // ARDUINO_MODEでLED点灯
  CMD_EXECUTE();
  for (int i = 0; i < MOTOR_NUM; i++) { 
    //motor_controllers[i].driveMotor();
  }
}

//モード切り替わり確認
void check_mode_change()
{
  //noInterrupts();      //割り込み停止
  //rcTime[0] = time[0];
  //rcTime[1] = time[1];
  //rcTime[2] = time[2];
  //interrupts();     //割り込み開始

    
    /*if (runMode != ARDUINO_MODE)
    { // モードが変わった時(RC→ARDUINO)
      Serial.println(F("##########################"));                  
      Serial.println(F("### モード:ARDUINO_MODE ###"));
      Serial.println(F("##########################"));            
      stop_motor_immediately( );
      reset_arduino_mode_flags();
      reset_pid_gain( );
    }
    runMode = ARDUINO_MODE;
  //}
  //else if (ARDUINO_MODE_OUT > rcTime[1] && CUGO_PROPO_MIN_B < rcTime[1]) // MR-8の外れ値が入ったときは遷移させない
  //{
    if (runMode != RC_MODE)
    { // モードが変わった時(ARDUINO→RC)
      Serial.println(F("##########################"));                  
      Serial.println(F("###   モード:RC_MODE    ###"));
      Serial.println(F("##########################"));            
      reset_arduino_mode_flags();
    }
    runMode = RC_MODE;
  //}
  */
  if (ARDUINO_MODE == runMode)
  {
    arduino_mode();
  }
  else
  {
    //rc_mode(rcTime);
  }
  
//  arduino_mode();
}

//割り込み時の実行処理関連
void init_PID()
{
  //Serial.println(F("#   init_PID"));//確認用
  //pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  //pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  //pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  //pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効

  // LEFTインスタンス有効化
  //motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, L_reverse);
  // RIGHTインスタンス有効化
  //motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, R_reverse);

  // エンコーダカウンタは純正のハードウェア割り込みピンを使用
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), leftEncHandler, RISING);
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), rightEncHandler, RISING);

  // 初期値でモータ指示。起動時に停止を入力しないと保護機能が働き、回りません。
  //motor_direct_instructions(1500, 1500); //直接停止命令を出す
  //delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}

//割り込み時の実行処理関連
void leftEncHandler()
{
  //motor_controllers[MOTOR_LEFT].updateEnc();
}

//割り込み時の実行処理関連
void rightEncHandler()
{
  //motor_controllers[MOTOR_RIGHT].updateEnc();
}
//10ミリ秒毎に実行
void job_10ms()
{
  check_mode_change();
}

/***** Arduino学習用プログラミングはここから *****/
/* ヒント：使えるコマンドリスト */
/*
 * ★基本コマンド★
 *    button();          //ボタンの押し待ち     
 *    matsu(1000);       //1000ミリ秒待機       □詳細：入力した()内の数字ミリ秒だけ待機する。1000ミリ秒＝1秒。
 *    susumu(1.0);       //1.0m前に進む         □詳細：入力した()内の数字m 分だけ前に進む
 *    sagaru(1.0);       //1.0m後ろに進む       □詳細：入力した()内の数字m 分だけ後ろに進む
 *    migimawari45();    //45度右に回転する 
 *    migimawari90();    //90度右に回転する
 *    migimawari180();   //180度右に回転する
 *    hidarimawari45();  //45度右に回転する
 *    hidarimawari90();  //90度右に回転する
 *    hidarimawari180(); //180度右に回転する
 * 
 * ★★応用コマンド★★
 *    susumu(1.0,90);                //上限速度90rpmで1.0m前に進む         □詳細：()内に移動距離と上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合もある。
 *    sagaru(1.0,90);                //上限速度90rpmで1.0m後ろに進む      　□詳細：()内の設定はsusumu(1.0,90)と同様
 *    migimawari45(90);              //上限速度90rpmで45度右に回転する      □詳細：()内に上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合もある。
 *    migimawari90(90);              //上限速度90rpmで90度右に回転する      □詳細：()内の設定はmigimawari45(90)と同様
 *    migimawari180(90);             //上限速度90rpmで180度右に回転する     □詳細：()内の設定はmigimawari45(90)と同様
 *    hidarimawari45(90);            //上限速度90rpmで45度左に回転する      □詳細：()内の設定はmigimawari45(90)と同様
 *    hidarimawari90(90);            //上限速度90rpmで90度左に回転する      □詳細：()内の設定はmigimawari45(90)と同様
 *    hidarimawari180(90);           //上限速度90rpmで180度左に回転する     □詳細：()内の設定はmigimawari45(90)と同様
 *    turn_clockwise(60,90);         //上限速度90rpmで60度右に回転する      □詳細：()内に回転角度と上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合もある。
 *    turn_counter_clockwise(60,90); //上限速度90rpmで60度左に回転する      □詳細：()内の設定はturn_clockwise(60,90)と同様
*/

//コマンドの実行
void CMD_EXECUTE()
{
  cmd_manager();  // おまじない

  // ここから↓を改造していこう！

  //例：ボタンが押されたら1mの正方形を描く動き
  //button(); 


/*
  float test_degree =540;
  float test_verocity = 40; 

  migimawari(test_degree);
  matsu(1000);
  migimawari(test_degree,test_verocity);
  matsu(1000);
*/

/*
  float test_degree =0.0;
  float test_verocity = 20;

  migimawari90(test_degree);
  matsu(1000);
  migimawari(test_degree);
  matsu(1000);
  migimawari(test_degree,test_verocity);
  matsu(1000);
*/

  float test_degree =3600.0;
  float test_verocity = 60; 

  migimawari(test_degree);
  matsu(1000);
  migimawari(test_degree,test_verocity);
  matsu(1000);



/*
  hidarimawari90();
  matsu(1000);
  hidarimawari45();
  matsu(1000);
  hidarimawari180();
  matsu(1000);
*/

//matsu(1000);
/*
  migimawari90();
  matsu(1000);
  migimawari45();
  matsu(1000);
  migimawari180();
  matsu(1000);
  */
/*
  migimawari90(test_verocity);
  matsu(1000);
  migimawari45(test_verocity);
  matsu(1000);
  migimawari180(test_verocity);
/*
  hidarimawari(test_degree);
  matsu(1000);
  hidarimawari(test_degree,test_verocity);
  matsu(1000);
  hidarimawari90();
  matsu(1000);
  hidarimawari45();
  matsu(1000);
  hidarimawari180();
  matsu(1000);
  hidarimawari90(test_verocity);
  matsu(1000);
  hidarimawari45(test_verocity);
  matsu(1000);
  hidarimawari180(test_verocity);
*/
  /*
  matsu(1000);//1000ミリ秒(1秒) 待つ
  migimawari90();//右回りに90度回転
  matsu(1000);//1000ミリ秒(1秒) 待つ
   
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
  */
  // ここから↑を改造していこう！

  cmd_end();      // おまじない
}
