/****************************************************************************
 * CugoBeginnerProgramming                                           *
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
/*****************CugoBeginnerProgramming ver1.00*********************/
/****************************************************************************
 * CugoBeginnerProgrammingをご利用される方へ
 *  スクロールして「学習用プログラミングはここから」からご確認ください。
 *  詳細はREADME.mdをご確認ください。
 ****************************************************************************/
#include "CugoCommandMode.h"

//プロトタイプ宣言
void CMD_EXECUTE();
void check_mode_change();
void job_10ms();

RPI_PICO_Timer ITimer0(0);

//初期設定
void setup()
{
  Serial.begin(115200, SERIAL_8N1);//PCとの通信
  Serial1.begin(115200, SERIAL_8N1);//ld2との通信
  delay(1000); 

  //ld2関連初期設定
  ld2_set_feedback(2, 0b10000001);//freq{0:10[hz] 1:50[hz] 2:100[hz]} kindof_data{0b1:Mode 0b10:CMD_RPM 0b100:CurrentRPM 0b1000:AveCurrentRPM 0b10000000:EncorderData}
  delay(1000); 
  if (!ITimer0.attachInterruptInterval(ld2_feedback_dutation * 1000, ld2_timer_handler)) {
    Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }
  delay(1000);
  ld2_set_control_mode(RC_MODE);//mode{0:RC_mode 1:CMD_Mode}
  delay(1000);
  //pinMode(CMD_BUTTON_PIN, INPUT_PULLUP);
  init_CMD();//コマンド初期化
  init_display();//起動時モニタ表示
}

//loop内を繰り返し実行
void loop()
{
  current_time = micros();
  if (current_time - prev_time_10ms > 10000) 
  {
    job_10ms();
    prev_time_10ms = current_time;
  }
  display_detail();
}

//モード切り替わり確認
void check_mode_change()
{
  if (COMMAND_MODE == run_mode)
  {
    CMD_EXECUTE();
  }
}

//10ミリ秒毎に実行
void job_10ms()
{
  check_mode_change();
}

/***** 学習用プログラミングはここから *****/
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
  matsu(1000);//1000ミリ秒(1秒) 待つ

  susumu(1.0);//1m前に進む
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
  
  // ここから↑を改造していこう！

  cmd_end();      // おまじない
}
