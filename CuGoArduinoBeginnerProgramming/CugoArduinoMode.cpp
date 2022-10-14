#include "Arduino.h"
#include "CugoArduinoMode.h"
#include "MotorController.h"
#include <SPI.h>


void init_SPI()
{
  SPI.begin();
  digitalWrite(SS, HIGH);
}

void send_spi(int mode) {
  digitalWrite(SS, LOW);
  SPI.transfer(mode);
  digitalWrite(SS, HIGH);
}

void init_KOPROPO(int runMode,int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE)
{
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

void set_arduino_cmd_matrix(int cmd_no, int cmd_0, int cmd_1, int cmd_2, int cmd_3, int cmd_4, int cmd_5,int arduino_cmd_matrix[CMD_SIZE][6])
{
  arduino_cmd_matrix[cmd_no][0] = cmd_0;
  arduino_cmd_matrix[cmd_no][1] = cmd_1;
  arduino_cmd_matrix[cmd_no][2] = cmd_2;
  arduino_cmd_matrix[cmd_no][3] = cmd_3;
  arduino_cmd_matrix[cmd_no][4] = cmd_4;
  arduino_cmd_matrix[cmd_no][5] = cmd_5;
}

void init_ARDUINO_CMD(int arduino_cmd_matrix[CMD_SIZE][6])
{
  pinMode(CMD_BUTTON_PIN, INPUT);
  for (int i = 0; i < CMD_SIZE; i++)
  {
    set_arduino_cmd_matrix(i, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO,arduino_cmd_matrix);
  }
}

void view_arduino_cmd_matrix(int arduino_cmd_matrix[CMD_SIZE][6])
{
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
  if (UDP_CONNECTION_DISPLAY == false && ENCODER_DISPLAY == false && PID_CONTROLL_DISPLAY == false)
  {
    Serial.println("Display item not set");
    Serial.println("Arduino is working...");
    Serial.println("");
  }
}

void spi_cmd(int spi_cmd_value,int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init)
{
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);

    }
    // 初回起動時の処理
    // テスト(ボタンのフラグbutton_enable==1としてテスト）
    // メモリが足りないので、buttonとSPIを共用にする。
    set_arduino_cmd_matrix(init_current_cmd, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, spi_cmd_value, 0, 0,arduino_cmd_matrix); // ここではテストで1を使用。
    *init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}

void calc_necessary_rotate(float degree,long int *target_count_L,long int *target_count_R,float tread,int encoder_resolution,float wheel_radius_l,float wheel_radius_r) // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
{
  *target_count_L =  ((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_l * PI);
  *target_count_R = -((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_r * PI);
  Serial.println("degree: " + String(degree));
  Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * tread * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
  //while(1);
}

//たたまない？
void calc_necessary_count(float distance,long int *target_count_L,long int *target_count_R,float tread,int encoder_resolution,float wheel_radius_l,float wheel_radius_r) // TODO:ベクトルを入れるが、回転や並進で別の関数にならないか確認が必要
{
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

  Serial.println("distance: " + String(distance));
  //Serial.println("distance: " + String(encoder_resolution));
  //Serial.println("2 * wheel_radius_l * PI: " + String(2 * wheel_radius_l * PI));
  //Serial.println("calc: " + String(distance * encoder_resolution / (2 * wheel_radius_l * PI)));

  Serial.println("### target_count_L/R: " + String(*target_count_L) + " / " + String(*target_count_R) + "###");
  //Serial.println("distance: " + String(distance));
  //Serial.println("wheel_radius_l: " + String(wheel_radius_l));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
  //while(1);

}


void atamaopen(int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init)
{
  spi_cmd(6,*init_current_cmd ,arduino_cmd_matrix,cmd_init);
}

void atamaclose(int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init)
{
  spi_cmd(5,*init_current_cmd ,arduino_cmd_matrix,cmd_init);
}

void wait_button(int *init_current_cmd ,int arduino_cmd_matrix[CMD_SIZE][6],bool cmd_init)
{
  if (cmd_init == false)
  {

    if (init_current_cmd >= CMD_SIZE - 1)
    {
      Serial.println("コマンド上限数以上にコマンドを設定しています。意図しない走行をさせないため強制終了。");
      while (1);

    }
    // 初回起動時の処理
    // テスト(ボタンのフラグbutton_enable==1としてテスト）
    // メモリが足りないので、buttonとSPIを共用にする。
    set_arduino_cmd_matrix(init_current_cmd, EXCEPTION_NO, EXCEPTION_NO, EXCEPTION_NO, 255, 0, 0,arduino_cmd_matrix); // ここではテストで1を使用。
    *init_current_cmd++;

  }
  else
  {
    // 通常ループ時の処理

  }
}
