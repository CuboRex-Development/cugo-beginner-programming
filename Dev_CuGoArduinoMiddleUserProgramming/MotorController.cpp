#include "Arduino.h"
#include "MotorController.h"

MotorController::MotorController(){
  initialized_ = false;
}

MotorController::MotorController(int enc_pin_a, int enc_pin_b, int servo_pin, int pulse_per_round, int max_speed, int control_hz, float lpf_rate, float kp, float ki, float kd, bool reverse){
  
  // 引数で変数を初期化
  enc_pin_a_ = enc_pin_a;
  enc_pin_b_ = enc_pin_b;
  servo_pin_ = servo_pin;
  pulse_per_round_ = pulse_per_round;
  max_speed_ = max_speed;
  control_hz_ = control_hz;
  lpf_rate_ = lpf_rate;
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  reverse_ = reverse;

  // 変数の初期化
  speed_ = 0;
  enc_ = 0;
//  enc_rotation_ = 0;    //22/8/2追加 FIX対応：モード切替時のびくつき
  prev_enc_ = 0;
  rpm_ = 0.0;
  prev_rpm_ = 0.0;
  target_rpm_ = 0.0;
  prev_p_ = 0.0;
  prev_i_ = 0.0;
  stop_cnt = 0;

  
  // Servo変数にピンを割り当て
  servo_.attach(servo_pin_);

  //attachInterrupt(digitalPinToInterrupt(enc_pin_a_), &ChangedEncPin, RISING);

  initialized_ = true;
}

void MotorController::driveMotor(){
  if(!initialized_){
    return;
  }

  // rpm_の更新
  calcRpm();
  
  pidControl();
 
  // サーボに書き込み
  servo_.writeMicroseconds(PULSE_CENTRAL + speed_);
  //Serial.println(String(speed_));

/**
  Serial.print("driveMotor: ");
  Serial.print(servo_pin_);
  Serial.print(",");
  Serial.print(target_rpm_);
  Serial.print(",");
  Serial.print(enc_);
  Serial.print(",");
  Serial.print(rpm_);
  Serial.print(",");
  Serial.println(pulse_);
*/  
}

//static void MotorController::ChangedEncPin(){
//}

void MotorController::reset_PID_param()
{
  speed_ = 0;
  enc_ = 0;     //22/10/24　FIX対応：モード切替時のびくつき
  prev_enc_ = 0;//22/10/24  FIX対応：モード切替時のびくつき
  rpm_ = 0.0;
  prev_rpm_ = 0.0;
  target_rpm_ = 0.0;
  prev_p_ = 0.0;
  prev_i_ = 0.0;
  stop_cnt = 0;
}

void MotorController::updateEnc(){
  if(!initialized_){
    return;
  }

  if(LOW == digitalRead(enc_pin_b_)){
    if(!reverse_){
      enc_--;  // PINがLOW & 正転 -> デクリメント
    }else{
      enc_++;  // PINがLOW & 逆転 -> インクリメント
    }
  }else{
    if(!reverse_){
      enc_++;  // PINがHIGH & 正転 -> インクリメント
    }else{
      enc_--;  // PINがHIGH & 逆転 -> デクリメント
    }
  }
}

void MotorController::setTargetRpm(float target_rpm){
  if(!initialized_){
    return;
  }
  target_rpm_ = target_rpm;
}

float MotorController::getTargetRpm(){
    if(!initialized_){
    return;
  }
  return target_rpm_;
}


long int MotorController::getCount(){
  if(!initialized_){
    return 0;
  }
  return enc_;
}

/*
int MotorController::getCounterRotation(){
  if(!initialized_){
    return 0;
  }
//  return enc_rotation;
}
*/

float MotorController::getRpm(){
  if(!initialized_){
    return 0.0;
  }
  
  return rpm_;
}

float MotorController::getSpeed(){
  if(!initialized_){
    return 0.0;
  }
  
  return speed_;
}

float MotorController::getPID_P(){
  if(!initialized_){
    return 0.0;
  }
  return disp_p;
}

float MotorController::getPID_I(){
  if(!initialized_){
    return 0.0;
  }
  return disp_i;
}

float MotorController::getPID_D(){
  if(!initialized_){
    return 0.0;
  }
  return disp_d;
}

void MotorController::calcRpm(){
  if(!initialized_){
    return;
  }
  
  float rps;  // RPS
  int diff; // エンコーダカウントの差分
  float rpm_nolpf;  // RPM(LPF前)

  // エンコーダカウントの差分を生成
  diff = enc_ - prev_enc_;
  // ノイズで車体が動いていない時もカウントアップしてしまうため
  if (-3 < diff && diff < 3 ){
    diff = 0;
    enc_ = prev_enc_;
  }
  //Serial.print(diff);
  //Serial.print(",");

  if(diff == 0){ // 0割り回避
    rpm_nolpf = 0.0;
  }else{
    // RPS 計算
    rps = (float)diff * (float)control_hz_ / (float)pulse_per_round_;
    
    // RPS -> RPM
    rpm_nolpf = rps * 60.0;
  }
  
  // RPM 計算
  //Serial.print("prev_rpm_:" + String(prev_rpm_));
  //Serial.print("lpf_rate_:" + String(lpf_rate_));
  //Serial.print("rpm_nolpf:" + String(rpm_nolpf));

  rpm_ = prev_rpm_ * lpf_rate_ + rpm_nolpf * (1.0 - lpf_rate_);
  //Serial.println(rpm_);

  // prev_ 更新
  prev_rpm_ = rpm_;
  prev_enc_ = enc_;

}

void MotorController::pidControl(){
  if(!initialized_){
    return;
  }
  
  float p;  // P制御値
  float i;  // I制御値
  float d;  // D制御値

  // 各制御値の計算
  p = target_rpm_ - rpm_;
  i = prev_i_ + p;
  d = p - prev_p_;
//  Serial.println("");
//  Serial.println("i: " + String(i));

  // PID制御
  //Serial.print("target_rpm_: " + String(target_rpm_));
  //Serial.print("rpm_: " + String(rpm_));
  //Serial.print("prev_i_: " + String(prev_i_));
  //Serial.println("prev_p_: " + String(prev_p_));  
  speed_ = p * kp_ + i * ki_ + d * kd_;  
  
  // prev_ 更新
  prev_p_ = p;
  prev_i_ = i;

  // 画面表示用
  disp_p = p;
  disp_i = i;
  disp_d = d;

  // iゲインの発散補償（自動整合）
  float surplus = limitSpeed();
  prev_i_ = prev_i_ - surplus * kp_;

  // 停止中のiゲインの定常偏差をリセット（モータの不感地帯での制御がかかってしまうため）
  if(target_rpm_ == 0.0 || p == 0.0){
    stop_cnt++;
  }
  if(stop_cnt > 100){
    prev_i_ = 0.0;
    stop_cnt = 0;
  }

/*
  if(servo_pin_ == 6){
    //Serial.print("pidControl: ");
    //Serial.print(servo_pin_);
    //Serial.print(",");
    Serial.print(p);
    Serial.print(",");
    Serial.print(i);
    //Serial.print(",");
    //Serial.print(d);
    //Serial.print(",");
    //Serial.print(prev_p_);
    //Serial.print(",");
    //Serial.println(prev_i_);
    Serial.print(",");
    Serial.println(rpm_);
  }
  */
}

float MotorController::limitSpeed(){
  if(!initialized_){
    return -1.0;
  }
  float surplus; //Iゲイン自動整合用の余剰制御量を算出
  surplus = speed_ - max_speed_;
  
  // speed_ を範囲内に調整
  if(speed_ > max_speed_){
    speed_ = max_speed_;
    //Serial.println("更新+: " + String(speed_));
    return surplus;
  }else if(speed_ < -max_speed_) {
    speed_ = -max_speed_;
    //Serial.println("更新-: " + String(speed_));
    return surplus;
  } else {
    return 0.0;
  }

}
