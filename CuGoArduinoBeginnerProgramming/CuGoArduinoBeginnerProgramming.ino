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

// 回転方向ソフトウェア切り替え
const bool L_reverse = false;
const bool R_reverse = true;

// joshibi: L:True, R:false
// cugo-chan: L:false, R:True

/***** ↑各ユーザーごとに設定してください↑ *****/

unsigned long long current_time = 0, prev_time_10ms = 0, prev_time_100ms, prev_time_1000ms; // オーバーフローしても問題ないが64bit確保

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
//RUN_MODE runMode = RC_MODE;  // 初回起動時はRC_MODE（無意識な暴走を防ぐため）
int runMode = ARDUINO_MODE;

bool cmd_init = false;
int current_cmd = 0;

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

//// PID位置制御のデータ格納
float l_count_prev_i_ = 0;
float l_count_prev_p_ = 0;
float r_count_prev_i_ = 0;
float r_count_prev_p_ = 0;
float l_count_gain = 0;
float r_count_gain = 0;

//プロトタイプ宣言
void susumu(float distance);//前方移動距離のみ指定 速度はデフォルト値
void susumu(float distance,float max_velocity);//前方移動距離と速度指定
void sagaru(float distance);//後方移動距離のみ指定 速度はデフォルト値
void sagaru(float distance,float max_velocity);//後方移動距離と速度のみ指定
void migimawari(float degree);
void migimawari(float degree,float max_velocity);
void migimawari90();
void migimawari90(float max_velocity);
void migimawari45();
void migimawari45(float max_velocity);
void migimawari180();
void migimawari180(float max_velocity);
void hidarimawari(float degree);
void hidarimawari(float degree,float max_velocity);
void hidarimawari90();
void hidarimawari90(float max_velocity);
void hidarimawari45();
void hidarimawari45(float max_velocity);
void hidarimawari180();
void hidarimawari180(float max_velocity);

//割込み処理のためCugoArduinoModeへの移動不可
// ピン変化割り込みの割り込み #TODO まだコードを畳められていない
ISR(PCINT2_vect)
{
  //Serial.println("割り込み");
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
  {  //Serial.println("モード変更割り込み");

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

//移動すべきではない　attachInterruptが入っているため
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

//CugoArduinoModeへの移動不可?githubで確認中
void leftEncHandler()
{
  motor_controllers[MOTOR_LEFT].updateEnc();
}

//CugoArduinoModeへの移動不可?githubで確認中
void rightEncHandler()
{
  motor_controllers[MOTOR_RIGHT].updateEnc();
}

//CMD_EXECUTEが張っているため退避不可
void arduino_mode()
{
  digitalWrite(LED_BUILTIN, HIGH);  // ARDUINO_MODEでLED点灯
  CMD_EXECUTE();
  for (int i = 0; i < MOTOR_NUM; i++) { // 4輪でも使えるように
    motor_controllers[i].driveMotor();
  }
}

//割り込み周りが入っているため退避すべきではない
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
      Serial.println("Mode changed::ARDUINO_MODE");      
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
      Serial.println("Mode changed::RC_MODE");      
      reset_arduino_mode_flags();
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
    rc_mode(rcTime,motor_controllers);
  }
}

//可用性を考えるとjobは退避すべきではない？
void job_10ms()
{
  check_mode_change();
}

//可用性を考えるとjobは退避すべきではない？
void job_100ms()
{
  //check_failsafe();存在しない関数
  //display_speed(motor_controllers,ENCODER_DISPLAY);
  //display_target_rpm(motor_controllers,ENCODER_DISPLAY);
  //display_PID(motor_controllers,PID_CONTROLL_DISPLAY);
  //display_failsafe(FAIL_SAFE_DISPLAY,runMode);
}
//可用性を考えるとjobは退避すべきではない？
void job_1000ms()
{
  //display_nothing(UDP_CONNECTION_DISPLAY,ENCODER_DISPLAY,PID_CONTROLL_DISPLAY);
}

//
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
//
void reset_arduino_mode_flags()
{
  Serial.println("reset_arduino_mode_flags");// デバッグ用確認
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
//
void cmd_manager_flags_init()
{
  //Serial.println("#     CMD_manager_init     #");
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
    Serial.println("init_current_cmd: " + String(init_current_cmd));
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

  //Serial.println(arduino_cmd_matrix[current_cmd][3]);
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
    view_arduino_cmd_matrix();
    Serial.println("複数コマンド入力。入力関数に不備があるか、コマンドを上書きしている可能性あり。");
    stop_motor_immediately(motor_controllers);
    //delay(500);

    while (1);
  }
}
//
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
//
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
//
void set_button_cmd()
{
  button_push_count = 0;
  button_enable = 0;
}
//
void check_achievement_go_forward_cmd()
{
  bool L_done = false;//故障のため・・10/24 
  bool R_done = false;

  // L側目標達成チェック// 位置制御とバッティングする　
  if (cmd_L_back == false) {
    //if (target_count_L < motor_controllers[MOTOR_LEFT].getCount())//退避用
    if (target_count_L < motor_controllers[MOTOR_LEFT].getCount())
      L_done = true;
    //Serial.println("L_DONE!");
    //motor_controllers[MOTOR_LEFT].setTargetRpm(0);
  } else {
    //if (target_count_L > motor_controllers[MOTOR_LEFT].getCount())//退避用
    if (target_count_L > motor_controllers[MOTOR_LEFT].getCount())
      L_done = true;
    //Serial.println("L_DONE!");
    //motor_controllers[MOTOR_LEFT].setTargetRpm(0);
  }

  // R側目標達成チェック
  if (cmd_R_back == false) {
    //if (target_count_R < motor_controllers[MOTOR_RIGHT].getCount())//退避用
    if (target_count_R < motor_controllers[MOTOR_RIGHT].getCount())
      R_done = true;
    //Serial.println("R_DONE!");
    //motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
  } else {
    //if (target_count_R > motor_controllers[MOTOR_RIGHT].getCount())//退避用
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
  //Serial.print("target_count L/R: " + String(target_count_L) + " / " + String(target_count_R));
  //Serial.println("L/R DONE!: " + String(L_done) + " / " + String(R_done));

  // L/R達成していたら終了
  if (L_done == true && R_done == true)
  {
    stop_motor_immediately(motor_controllers);
    count_done = true;
  }
}
//
void check_achievement_wait_time_cmd()
{
  //Serial.println("curennt time: " + String(micros()) + ", target_time: " + String(target_wait_time));
  if (target_wait_time < micros())
  {
    stop_motor_immediately(motor_controllers);
    wait_done = true;
  }
}
//
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
      //ここで速度制御：setTargetRpm目標速度の設定

      //// PID位置制御の制御値
      float l_count_p;  // P制御値
      float l_count_i;  // I制御値
      float l_count_d;  // D制御値
      float r_count_p;  // P制御値
      float r_count_i;  // I制御値
      float r_count_d;  // D制御値
      if(arduino_cmd_matrix[current_cmd][4] == 0.0 && arduino_cmd_matrix[current_cmd][5] == 0.0)
      {
        //停止しているだけの時
        motor_controllers[MOTOR_LEFT].setTargetRpm(arduino_cmd_matrix[current_cmd][4]);
        motor_controllers[MOTOR_RIGHT].setTargetRpm(arduino_cmd_matrix[current_cmd][5]);

      } else{
        // 各制御値の計算
        l_count_p = arduino_cmd_matrix[current_cmd][0] - motor_controllers[MOTOR_LEFT].getCount();
        l_count_i = l_count_prev_i_ + l_count_p;
        l_count_d = l_count_p - l_count_prev_p_;
        r_count_p = arduino_cmd_matrix[current_cmd][1] - motor_controllers[MOTOR_RIGHT].getCount();
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
        l_count_gain = min( max(l_count_gain,-fabsf(arduino_cmd_matrix[current_cmd][4])),fabsf(arduino_cmd_matrix[current_cmd][4]));//ユーザ設定の速度上限        
        r_count_gain = min( max(r_count_gain,-fabsf(arduino_cmd_matrix[current_cmd][5])),fabsf(arduino_cmd_matrix[current_cmd][5]));//ユーザ設定の速度上限        

           
        //位置制御
        motor_controllers[MOTOR_LEFT].setTargetRpm(l_count_gain);
        motor_controllers[MOTOR_RIGHT].setTargetRpm(r_count_gain);
        Serial.println("gain:l/r " + String(l_count_gain) + "," + String(r_count_gain));
        /*
       //使うもの
       arduino_cmd_matrix[current_cmd][0] //L側の目標カウント数
       arduino_cmd_matrix[current_cmd][1] //R側の目標カウント数
       motor_controllers[MOTOR_LEFT].getCount() //Lの現在のカウント数 
       motor_controllers[MOTOR_LEFT].getCount() //Rの現在のカウント数 

       //以降は必要であれば追加//
        // iゲインの発散補償（自動整合）
       //float surplus = limitSpeed();
       //prev_i_ = prev_i_ - surplus * kp_;
       // 停止中のiゲインの定常偏差をリセット（モータの不感地帯での制御がかかってしまうため）
         if(target_rpm_ == 0.0 || p == 0.0){
         stop_cnt++;
         }
        if(stop_cnt > 100){
        prev_i_ = 0.0;
        stop_cnt = 0;
        }
       */
        }
      


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
          reset_arduino_mode_flags();
          end_arduino_mode = false;
          runMode = RC_MODE;
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
  }
}
//
void check_achievement_spi_cmd()
{
  send_spi(arduino_cmd_matrix[current_cmd][3]);
  spi_done = true;
}
//
void cmd_end()  // もっとマシな名前を考える
{

  if (cmd_init == false)
  {
    // 初回起動時の処理
    //Serial.println("CMD_SIZE: " + String(CMD_SIZE));
    // while (1);
    if (init_current_cmd >= CMD_SIZE)
    {
      //Serial.println("init_current_cmd: " + String(init_current_cmd));
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。"); // ココには至らないはず
      while (1);
    }


    // 初回起動時の処理をここで無効化
    reset_pid_gain(motor_controllers);
    cmd_init = true;   // 最初の一回だけ。全部のコマンドが終了したとき、最初のコマンドに戻すときにリセット。それは未実装。
  }
  else
  {
    // 通常ループ時の処理


    // すべてのコマンドが終了しているか判定



  }
}
//退避しないでよい？
void susumu(float distance)
{
  go_forward(distance,EXCEPTION_NO);
}
void susumu(float distance,float max_velocity){
  go_forward(distance,max_velocity);
  
}

//要退避
void go_forward(float distance,float max_velocity)
{
  if (cmd_init == false)
  {
    // 初回起動時の処理
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_count(distance,&target_count_L,&target_count_R);
    Serial.println("target_count_L/R: " + String(target_count_L) + ", " + String(target_count_R));
    float velocity = 0.0;
    if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    //Serial.println("matrix_target_count_L/R: " + String(arduino_cmd_matrix[init_current_cmd][0]) + ", " + String(arduino_cmd_matrix[init_current_cmd][0]));    
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
//退避しないでよい？
void sagaru(float distance)
{
  go_backward(distance,EXCEPTION_NO);
}
void sagaru(float distance,float max_velocity){
  go_backward(distance,max_velocity);
  
}

//要退避
void go_backward(float distance,float max_velocity)
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
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_count(distance,&target_count_L,&target_count_R);
    Serial.println("target_count_L/R: " + String(-target_count_L) + ", " + String(-target_count_R));
    float velocity = 0.0;
    if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(-target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, -velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
//退避しないでよい？
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

//要退避
void turn_clockwise(float degree,float max_velocity)
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
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_rotate(degree,&target_count_L,&target_count_R);
    Serial.println("target_count_L/R: " + String(target_count_L) + ", " + String(target_count_R));

    float velocity = 0.0;
     if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }   
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(target_count_L, target_count_R, EXCEPTION_NO, EXCEPTION_NO, velocity, -velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    init_current_cmd++;
  }
  else
  {
    // 通常ループ時の処理

  }
}

//退避しないでよい？
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

//要退避
void turn_counter_clockwise(float degree,float max_velocity)
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
    //Serial.println("init_current_cmd: " + String(init_current_cmd));
    calc_necessary_rotate(degree,&target_count_L,&target_count_R);
    Serial.println("target_count_L/R: " + String(-target_count_L) + ", " + String(-target_count_R));

    float velocity = 0.0;
    if(max_velocity == EXCEPTION_NO)
    {
      velocity = 90.0;
      }else{
      velocity = max_velocity;
    }   
    // テスト(L/R +4000カウント必要と固定して全体の動作テスト。実際は↑の関数で計算した必要カウント数を使う）
    set_arduino_cmd_matrix(-target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, velocity); // ここではテストで4000カウントまで、L/Rともに50rpmで進む。
    init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
//退避しないでよい？
void matsu(int milisec)
{
  wait_time(milisec,cmd_init);
}
//退避しないでよい？
void matu(int milisec)
{
  wait_time(milisec,cmd_init);
}
//退避しないでよい？
void botan()
{
  wait_button(cmd_init);
}
//退避しないでよい？
void button()
{
  wait_button(cmd_init);
}

void setup()
{
  Serial.begin(115200);
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
 * susumu(1.0);　入力した数字m 分だけ前に進む（テスト時は3m以下にしてください。止まりません）
 * //追加コマンド：susumu(1.0,90);速度上限rpmで設定　0~180 距離が短いと上限に行かない場合もある
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
  //Serial.println("CMD_EXECUTE");// デバッグ用確認
  //Serial.println("Current command: " + String(current_cmd));
  cmd_manager();  // おまじない

  // ここから↓を改造していこう！

  button();

  susumu(5.0);
  matsu(1000); 
  susumu(4.0,150);
  matsu(1000);
  migimawari90();
  matsu(1000);
  migimawari90(180);
  matsu(1000);
  sagaru(1.0,30);
  matsu(1000);
  migimawari(30,60);
  matsu(1000);
  sagaru(2.0);
  matsu(1000);
  hidarimawari90(10);
  matsu(1000);
  //hidarimawari90();
  //matsu(1000);


  //susumu(1.0);
  //matsu(1000);
  //migimawari90();
  //matsu(1000);
  // ここから↑を改造していこう！

  cmd_end();      // おまじない
  //view_arduino_cmd_matrix();
}
