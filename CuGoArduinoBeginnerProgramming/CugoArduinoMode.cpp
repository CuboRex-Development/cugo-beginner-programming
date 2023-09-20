#include "CugoArduinoMode.h"


/***** ↓必要に応じて各ユーザーごとに設定可能↓ *****/
  bool ENCODER_DISPLAY = false;
  bool FAIL_SAFE_DISPLAY = false;
/***** ↑必要に応じて各ユーザーごとに設定可能↑ *****/

//グローバル変数宣言
//cmd_matrix関連
  long int arduino_count_cmd_matrix[CMD_SIZE][2];
  int arduino_flag_cmd_matrix[CMD_SIZE][4];
  int init_current_cmd  = 0;
  int current_cmd = 0;

//カウント関連
  long int target_count_L = 0;
  long int target_count_R = 0;
  long int start_count_L = 0;
  long int start_count_R = 0;
  long int current_count_L = 0;
  long int current_count_R = 0;
  volatile long current_encoder_R = 0;
  volatile long current_encoder_L = 0;
  long int prev_encoder_L = 0;
  long int prev_encoder_R = 0;

//各種フラグ関連
  bool end_arduino_mode = false;
  bool cmd_init = false;
  bool cmd_exec = false;
  bool cmd_L_back = false;
  bool cmd_R_back = false;
  bool count_done = false;
  bool wait_done = false;
  bool button_done = false;
  bool spi_done = false;  
  int run_mode = RC_MODE;
  int old_run_mode  = RC_MODE;
  int button_push_count = 0;

//時間関連
  unsigned long long current_time = 0;
  unsigned long long prev_time_10ms = 0; 
  unsigned long long prev_time_100ms = 0; 
  unsigned long long prev_time_1000ms = 0; 
  long int target_wait_time = 0;

//PID位置制御関連
  float l_count_prev_i_ = 0;
  float l_count_prev_p_ = 0;
  float r_count_prev_i_ = 0;
  float r_count_prev_p_ = 0;
  float l_count_gain = 0;
  float r_count_gain = 0;
  bool PID_CONTROLL_DISPLAY = false;


//ld2関連
  volatile long ld2_id  = 0;
  volatile long ld2_feedback_hz = 0;
  volatile long ld2_feedback_dutation = 0;

//ローカル変数
//ld2関連
  const long int ld2_index_tofreq[3] = { 10, 50, 100 };

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

  for (int i = 0; i < CMD_SIZE; i++)
  {
    init_current_cmd = i;
    set_arduino_cmd_matrix(EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO);
  }
  init_current_cmd = 0;//初期化

}

void set_wait_time_cmd()
{
  //Serial.println(F("#   set_wait_time_cmd"));//確認用
  //target_wait_time = micros() + arduino_flag_cmd_matrix[current_cmd][0] * 1000;
  target_wait_time = arduino_flag_cmd_matrix[current_cmd][0];
  target_wait_time = target_wait_time * 1000; // こちらで1000倍処理
  target_wait_time = target_wait_time + micros();

}

void wait_time(long int milisec)
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

    if (ARDUINO_MODE == run_mode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      Serial.print(String(milisec));
      Serial.println(F("ms待つ"));
    }
    if (0 <= milisec && milisec <= TIME_MAX ){
    set_arduino_cmd_matrix(EXCEPTION_NO, EXCEPTION_NO, milisec, EXCEPTION_NO, 0, 0);
    }else{
        Serial.println(F("## BAD CASE!! ##"));
        Serial.println(F("時間が0以下または、計測上限を超えています。"));
        stop_motor_immediately();
        while (1);
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
    //stop_motor_immediately();
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

void matsu(long int milisec)
{
  wait_time(milisec);
}

void matu(long int milisec)
{
  wait_time(milisec);
}

void calc_necessary_rotate(float degree)
{
  //Serial.println(F("#   calc_necessary_rotate"));//確認用
  target_count_L =  ((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_l * PI);
  target_count_R = -((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_r * PI);
  //Serial.println("target_count_L:" + String(target_count_L));
  //Serial.println("degree: " + String(degree));
  //Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * tread * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
}

void calc_necessary_count(float distance)
{
  //Serial.println(encoder_resolution);//確認用
    target_count_L = distance * encoder_resolution / (2 * wheel_radius_l * PI);
    target_count_R = distance * encoder_resolution / (2 * wheel_radius_r * PI);
  //Serial.println(target_count_L);//確認用


  //target_count_L = distance / (2 * wheel_radius_l * PI);
  //target_count_R = distance / (2 * wheel_radius_r * PI);
  //target_count_L = target_count_L * encoder_resolution;
  //target_count_R = target_count_R * encoder_resolution;

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
    if (ARDUINO_MODE == run_mode) {
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


void motor_direct_instructions(int left, int right  ) // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println(F("#   motor_direct_instructions"));//確認用
    unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
    ld2_float_to_frame(left , 2, frame);
    ld2_float_to_frame(right, 6, frame);
    ld2_write_cmd(frame);        
}


void stop_motor_immediately( )
{
  //Serial.println(F("#   stop_motor_immediately"));//確認用
  unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
  ld2_float_to_frame(0 , 2, frame);
  ld2_float_to_frame(0, 6, frame);
  ld2_write_cmd(frame);        
}

void reset_pid_gain( )
{
  l_count_prev_i_ = 0;
  l_count_prev_p_ = 0;
  r_count_prev_i_ = 0;
  r_count_prev_p_ = 0;
  l_count_gain = 0;
  r_count_gain = 0;
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
    } else{
      velocity = max_velocity;
    }
    set_arduino_cmd_matrix(-target_count_L, -target_count_R, EXCEPTION_NO, EXCEPTION_NO, -velocity, -velocity);
    if (ARDUINO_MODE == run_mode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      
      if(velocity <= 0) {
        Serial.println(F("## BAD CASE!! ##"));
        Serial.println(F("不正な値：上限速度が0以下です"));
        stop_motor_immediately();
        while (1);
      }

      Serial.print(String(distance));
      Serial.print(F("m後ろに進む"));
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
    if (ARDUINO_MODE == run_mode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));

      if(velocity <= 0) {
        Serial.println(F("## BAD CASE!! ##"));
        Serial.println(F("不正な値：上限速度が0以下です"));
        stop_motor_immediately();
        while (1);
      }


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
    if (ARDUINO_MODE == run_mode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));
      
      if(velocity <= 0) {
        Serial.println(F("## BAD CASE!! ##"));
        Serial.println(F("不正な値：上限速度が0以下です"));
        stop_motor_immediately();
        while (1);
      }

      
      
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
    if (ARDUINO_MODE == run_mode) {
      Serial.print(F("###"));
      if (init_current_cmd < 9)
        Serial.print(F("0"));

      Serial.print(String(init_current_cmd + 1));
      Serial.print(F("番目のコマンド："));

      if(velocity <= 0) {
        Serial.println(F("## BAD CASE!! ##"));
        Serial.println(F("不正な値：上限速度が0以下です"));
        stop_motor_immediately();
        while (1);
      }

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
  cmd_exec = false;
  count_done = false;
  wait_done  = false;
  button_done = false;
  spi_done = false;
  target_wait_time = 0;
  button_push_count = 0;
  init_current_cmd = 0;
  current_count_L = 0;
  current_count_R = 0;

  init_ARDUINO_CMD();
  reset_pid_gain();
  ld2_encoder_reset();
}

void set_go_forward_cmd( )
{
  //Serial.println(F("#   set_go_forward_cmd"));//確認用
  start_count_L = current_count_L;
  target_count_L = current_count_L + arduino_count_cmd_matrix[current_cmd][0];//★

  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String(motor_controllers[MOTOR_LEFT].getCount()) + " + " + String(arduino_count_cmd_matrix[current_cmd][0]));
  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String( current_count_L) + " + " + String(arduino_count_cmd_matrix[current_cmd][0]));
  //while(1);
  if (arduino_count_cmd_matrix[current_cmd][0] >= 0) {
    cmd_L_back = false;
  } else {
    cmd_L_back = true;
  }
  start_count_R = current_count_R;
  target_count_R = current_count_R + arduino_count_cmd_matrix[current_cmd][1];//★
  if (arduino_count_cmd_matrix[current_cmd][1] >= 0) {
    cmd_R_back = false;
  } else {
    cmd_R_back = true;
  }
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
    //Serial.println(run_mode);

    if (ARDUINO_MODE == run_mode) {
      Serial.println(F("###   コマンド準備完了          ###"));   
      Serial.println(F("##################################"));
      Serial.println(F("###   コマンド実行開始          ###"));

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

  reset_pid_gain();
  ld2_encoder_reset();
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
      }else if(arduino_count_cmd_matrix[current_cmd][0] >= 0 && arduino_count_cmd_matrix[current_cmd][1] >= 0)//前進
      {
      distance =  (arduino_count_cmd_matrix[current_cmd][0]) * (( 2 * wheel_radius_l * PI) / encoder_resolution );
      Serial.print(String(abs(distance)));
      Serial.print(F(" m  前進"));
      }else if(arduino_count_cmd_matrix[current_cmd][0]  <= 0  && arduino_count_cmd_matrix[current_cmd][1] >= 0)//左回り
      {
      degree = (2 * wheel_radius_l * PI * arduino_count_cmd_matrix[current_cmd][0] * 360) / (encoder_resolution * tread * PI);
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　左回り"));
      }else if(arduino_count_cmd_matrix[current_cmd][0] >= 0 && arduino_count_cmd_matrix[current_cmd][1] <= 0)//右回り
      {
      degree = (2 * wheel_radius_l * PI * arduino_count_cmd_matrix[current_cmd][0] * 360) / (encoder_resolution * tread * PI);
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　右回り"));
      }else if(arduino_count_cmd_matrix[current_cmd][0]  <= 0  && arduino_count_cmd_matrix[current_cmd][1] <= 0)//後進
      {
      distance =  (arduino_count_cmd_matrix[current_cmd][0]) * (( 2 * wheel_radius_l * PI) / encoder_resolution );
      Serial.print(String(abs(distance)));
      Serial.print(F(" m  後進"));
      }else{
      Serial.println(F("不明なコマンド"));
      Serial.println(F("## BAD CASE!! ##"));
      Serial.println(F("入力関数に不備があるか、コマンドを上書きしている可能性あり。"));
      stop_motor_immediately();
      while (1);
      }

    Serial.println(F("   ###"));

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
  //Serial.println("target_count_L: " + String(target_count_L) + " = " + String( current_count_L) + " + " + String(arduino_count_cmd_matrix[current_cmd][0]));
  //Serial.println();
  // L側目標達成チェック
  if (cmd_L_back == false) {
    if (target_count_L <= current_count_L){
      L_done = true;
      //Serial.println(F("#   L_done"));
    }
  }else{
    if (target_count_L >= current_count_L){
      L_done = true;
      //Serial.println(F("#   L_done"));
    }
  }

  // R側目標達成チェック
  if (cmd_R_back == false) {
    if (target_count_R <= current_count_R){
      R_done = true;
      //Serial.println(F("#   R_done"));
    }
  }else{
    if (target_count_R >= current_count_R){
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
    if (arduino_count_cmd_matrix[current_cmd][0] >= 0 && arduino_count_cmd_matrix[current_cmd][1] >= 0) //前進
    {
      distance =  (current_count_L-start_count_L) * (( 2 * wheel_radius_l * PI) / encoder_resolution);
      //distance =  * conversion_count_to_distance;//★
      Serial.print(String(fabsf(distance)));
      Serial.print(F(" m  進んだ"));
    } else if (arduino_count_cmd_matrix[current_cmd][0] <= 0 && arduino_count_cmd_matrix[current_cmd][1] >= 0) //左回り
    {
      degree = (2 * wheel_radius_l * PI * (current_count_L-start_count_L) * 360) / (encoder_resolution * tread * PI);//★
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　左回りに回転した"));
    } else if (arduino_count_cmd_matrix[current_cmd][0] >= 0 && arduino_count_cmd_matrix[current_cmd][1] <= 0) //右回り
    {
      degree = (2 * wheel_radius_l * PI * (current_count_L-start_count_L) * 360) / (encoder_resolution * tread * PI);//★
      Serial.print(String(abs(degree)));
      Serial.print(F(" 度　右回りに回転した"));
    } else if (arduino_count_cmd_matrix[current_cmd][0] <= 0 && arduino_count_cmd_matrix[current_cmd][1] <= 0) //後進
    {
      distance =  (current_count_L-start_count_L) * (( 2 * wheel_radius_l * PI) / encoder_resolution);//★
      Serial.print(String(abs(distance)));
      Serial.print(F(" m 後に進んだ"));
    } else {
      Serial.println(F("不明なコマンド"));
      Serial.println(F("## BAD CASE!! ##"));
      Serial.println(F("入力関数に不備があるか、コマンドを上書きしている可能性あり。"));
      stop_motor_immediately();
      while (1);
    }
    Serial.println(F("  ###"));

  }
}

void cmd_manager( )
{
  if (cmd_init == false)
  {
    Serial.println(F("##################################"));
    Serial.println(F("###   コマンド準備開始          ###"));
  
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
      ld2_float_to_frame(0, 2, frame);
      ld2_float_to_frame(0, 6, frame);
      ld2_write_cmd(frame);        
      } else {
        // 各制御値の計算
        //l_count_p = arduino_count_cmd_matrix[current_cmd][0] - current_count_L;
        l_count_p = target_count_L - current_count_L;
        l_count_i = l_count_prev_i_ + l_count_p;
        l_count_d = l_count_p - l_count_prev_p_;
        //r_count_p = arduino_count_cmd_matrix[current_cmd][1] - current_count_R;
        r_count_p = target_count_R - current_count_R;
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
        ld2_float_to_frame(l_count_gain, 2, frame);
        ld2_float_to_frame(r_count_gain, 6, frame);
        ld2_write_cmd(frame);        

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

        ld2_encoder_reset();
        reset_pid_gain();

        current_cmd++;

        if (end_arduino_mode == true)
        {
          Serial.println(F("###   コマンド実行終了          ###"));
          Serial.println(F("##################################"));
          end_arduino_mode = false;
          run_mode = RC_MODE;
          reset_arduino_mode_flags();          
          ld2_set_control_mode(RC_MODE); //mode{0:RC_mode 1:CMD_Mode}

        }
      }
    }
  }
}

void check_achievement_button_cmd( )
{
  /*
  if (digitalRead(CMD_BUTTON_PIN) == 0)
  {
    button_push_count++;
  } else {
    button_push_count = 0;
  }
  */
  if (button_push_count >= 5) // 実測で50ms以上長いと小刻みに押したとき反応しないと感じてしまう。
  {
    //stop_motor_immediately( );
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


void  ld2_float_to_frame(float data, long int start, unsigned char* index) {  //配列indexの4番目からfloat dataを書き込む場合-> FloatToInt(data, 4, index);
  memcpy(&index[start], &data, 4);
}

void  ld2_frame_to_float(unsigned char* index, long int start, float* data) {  //配列indexの3番目からfloat dataに書き込む場合-> ld2_frame_to_float(index, 3, data);
  memcpy(data, &index[start], 4);
}

void  ld2_frame_to_short(unsigned char* index, long int start, short* data) {  //配列indexの3番目からuint16_t dataに書き込む場合-> ld2_frame_to_short(index, 3, data);
  memcpy(data, &index[start], 2);
}

void  ld2_write_cmd(unsigned char cmd[10]) {  //引数はidとチェックサム以外の配列
  long int i;
  unsigned char checksum = ld2_id;
  for (i = 0; i < 10; i++) {
    Serial1.write(cmd[i]);
    //Serial.print(cmd[i],HEX);
    //Serial.print(",");
    checksum += (unsigned char)(cmd[i]);
  }
  Serial1.write(ld2_id);
  //Serial.print(id,HEX);
  //Serial.print(",");
  Serial1.write(checksum);
  //Serial.println(checksum,HEX);
  ld2_id++;
  if (ld2_id>0xFF) ld2_id = 0;
}

void  ld2_get_cmd() {  //引数はidとチェックサム以外の配列
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
      ld2_set_encoder(frame);

    }*/
    
    if (frame[1] == 0x80){  //5.5.1 Control Mode Feedback
      if(frame[2] == 0x00){
        if(old_run_mode == ARDUINO_MODE){
          run_mode= RC_MODE;
          old_run_mode = RC_MODE;
          Serial.println(F("###   MODE:CUGO_RC_MODE        ###"));
          reset_arduino_mode_flags();
          //ld2_set_control_mode(RC_MODE); //mode{0:RC_mode 1:CMD_Mode}
        }else if(old_run_mode == RC_MODE){

        }else{

        }
      }else if(frame[2] == 0x01){
        if(old_run_mode == RC_MODE){
          run_mode= ARDUINO_MODE;
          old_run_mode = ARDUINO_MODE;
          Serial.println(F("###   MODE:CUGO_ARDUINO_MODE   ###"));          

        }else if(old_run_mode == RC_MODE){

        }else{

        }
      }
    }else if(frame[1] == 0x82){  //5.5.2 Command RPM Feedback

    }else if(frame[1] == 0x84){  //5.5.3 Current RPM Feedback
    //ld2_frame_to_float(frame,2,&rpm_current_L);
    //ld2_frame_to_float(frame,4,&rpm_current_R);

    }else if(frame[1] == 0x86){  //5.5.4 Average RPM Feedback

    }else if(frame[1] == 0x8D){  //5.5.5 SBUS Signal Feedback

    }else if(frame[1] == 0x8E){  //5.5.6 Encoder Feedback
      ld2_set_encoder(frame);
    }else if(frame[1] == 0x8F){  //Data Feedback Config

    }else{

    }

  }
  
}

bool ld2_timer_handler(struct repeating_timer *t) {
  ld2_get_cmd();
  return true;
}


void  ld2_set_encoder(unsigned char frame[12]) {
  short encoderR = 0, encoderL = 0;
  ld2_frame_to_short(frame, 2, &encoderL);
  ld2_frame_to_short(frame, 4, &encoderR);

  if((int)prev_encoder_L > 0 && (int)encoderL < 0 && ((int)prev_encoder_L - (int)encoderL) > (COUNT_MAX/2)){//オーバフローしている場合
    current_count_L = current_count_L + COUNT_MAX - (int)prev_encoder_L + (int)encoderL;
  }else if((int)prev_encoder_L < 0 && (int)encoderL > 0 && ((int)encoderL - (int)prev_encoder_L)>(COUNT_MAX/2)){//アンダーフローしている場合
    current_count_L = current_count_L + COUNT_MAX + (int)prev_encoder_L - (int)encoderL;
  }else {//それ以外（通常時）
    current_count_L = current_count_L + (int)encoderL - (int)prev_encoder_L;
  }
  current_encoder_L = current_encoder_L + (int)encoderL - (int)prev_encoder_L;
  prev_encoder_L = encoderL;

  if((int)prev_encoder_R > 0 && (int)encoderR < 0 && ((int)prev_encoder_R - (int)encoderR) > (COUNT_MAX/2)){//オーバフローしている場合
    current_count_R = current_count_R + COUNT_MAX - (int)prev_encoder_R + (int)encoderR;
  }else if((int)prev_encoder_R < 0 && (int)encoderR > 0 && ((int)encoderR - (int)prev_encoder_R)>(COUNT_MAX/2)){//アンダーフローしている場合
    current_count_R = current_count_R + COUNT_MAX + (int)prev_encoder_R - (int)encoderR;
  }else {//それ以外（通常時）
    current_count_R = current_count_R + (int)encoderR - (int)prev_encoder_R;
  }  
  current_encoder_R = current_encoder_R + (int)encoderR - (int)prev_encoder_R;
  prev_encoder_R = encoderR;
  //Serial.println("L: "+String(current_encoder_L)+"R:  "+String(current_encoder_R));
}

void ld2_encoder_reset() {
  unsigned char frame[10] = { 0xFF, 0x0E, 0x01, 0x01, 0, 0, 0, 0, 0, 0 };
  ld2_write_cmd(frame);
  current_encoder_L =0;
  prev_encoder_L = 0;
  start_count_L = 0;
  //current_count_L = 0;

  current_encoder_R = 0;
  prev_encoder_R = 0;
  start_count_R = 0;
  //current_count_R = 0;

  //delay(200);
}

void ld2_set_feedback(unsigned char freq_index, unsigned char kindof_data) {  //freq 0:10[hz] 1:50[hz] 2:100[hz] kindof_data 1:Mode 2:CMD_RPM 4:CurrentRPM 8:AveCurrentRPM 128:encoderData
  unsigned char frame[10] = { 0xFF, 0x0F, freq_index, kindof_data, 0, 0, 0, 0, 0, 0 };
  ld2_feedback_hz = ld2_index_tofreq[freq_index];
  ld2_feedback_dutation = 1000 /  ld2_feedback_hz;
  ld2_write_cmd(frame);
}

void ld2_set_control_mode(unsigned char mode) {  //mode 0x00:RC_mode 0x01:CMD_Mode
  unsigned char frame[10] = { 0xFF, 0x00, mode, 0, 0, 0, 0, 0, 0, 0 };
  ld2_write_cmd(frame);
}


void set_button_cmd()
{
  //Serial.println(F("#   set_button_cmd"));//確認用
  button_push_count = 0;
}


//未使用変数および関数　※シーケンスに影響あり
//表示関連

void init_SPI(){
    //Serial.println(F("#   init_SPI"));//確認用
    //SPI.begin();
    //digitalWrite(SS, HIGH);
  }

void send_spi(int mode) {
    //Serial.println(F("#   send_spi"));//確認用
    digitalWrite(SS, LOW);
    SPI.transfer(mode);
    digitalWrite(SS, HIGH);
  }

void spi_cmd(int spi_cmd_value){
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

void check_achievement_spi_cmd(){
  //Serial.println(F("#   check_achievement_spi_cmd"));//確認用
  //send_spi(arduino_flag_cmd_matrix[current_cmd][1]);
  spi_done = true;
  Serial.print(F("###"));
  if (current_cmd < 9)
    Serial.print(F("0"));

  Serial.print(String(current_cmd + 1));
  Serial.println(F("番目のコマンド：終了  ###"));
  }

int split(String data, char delimiter, String *dst){//dstは参照引き渡し
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


void view_arduino_cmd_matrix(){
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

void view_flags(){
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
    Serial.println(F(""));
  }

void display_failsafe(bool FAIL_SAFE_DISPLAY){
  //Serial.println(F("#   display_failsafe"));//確認用
  if (FAIL_SAFE_DISPLAY == true)
  {
    Serial.println(F("DISPLAY FAIL SAFE PARAM"));
    Serial.print(F("Mode(ARDUINO/RC): "));
    Serial.println(run_mode);
  }
  }

void display_nothing(){//1000msごとに表示したいものがあれば記載
  //Serial.println(F("#   display_nothing"));//確認用
  if (ENCODER_DISPLAY == false && PID_CONTROLL_DISPLAY == false)
  {
    //Serial.println(F("Display item not set"));
    //Serial.println(F("Arduino is working..."));
    //Serial.println(F(""));
  }
  }


void display_speed( bool ENCODER_DISPLAY){ // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT

  if (ENCODER_DISPLAY == true)
  {
    Serial.println(F("#   display_speed"));//確認用
    Serial.println("DISPLAY MOTOR COUNTER & SPEED");
    Serial.print("Mode:");
    Serial.println(run_mode);

    Serial.print(F("Encoder count (L/R):"));
    Serial.print(current_count_L);
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。開発用
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    Serial.print(F(","));
    Serial.println(current_count_L);    //制御量を見るため。
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

void display_target_rpm( bool ENCODER_DISPLAY){ // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT


  if (ENCODER_DISPLAY == true)
  {
    //Serial.println(F("#   display_target_rpm"));//確認用
    Serial.print(F("target_rpm[L]:"));
    Serial.println(l_count_gain);
    Serial.print(F("target_rpm[R]:"));
    Serial.println(r_count_gain);
  }
  }
/*
void display_PID( bool PID_CONTROLL_DISPLAY){ // motor_controllers[0] MOTOR_LEFT motor_controllers[1] MOTOR_RIGHT



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
*/

void job_100ms(){//100msごとに必要な情報を表示

  display_speed(ENCODER_DISPLAY);
  display_target_rpm(ENCODER_DISPLAY);
  //display_PID(PID_CONTROLL_DISPLAY);
  display_failsafe(FAIL_SAFE_DISPLAY);
  }

void job_1000ms(){//1000msごとに必要な情報があれば表示
  display_nothing();
  }

void display_detail(){
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


/*
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
    run_mode = RC_MODE;  
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
    run_mode = RC_MODE;
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
  run_mode = RC_MODE; //自動走行モードをループしたい場合はCUGO_SELF_DRIVE_MODEに変更
   
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
  /*
  Serial.println(F("自動走行モード終了")); 

  }

}
*/




//利用していない関数

/*





*/

