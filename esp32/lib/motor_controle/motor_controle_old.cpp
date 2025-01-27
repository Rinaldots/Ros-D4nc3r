/*
int l_pwm , r_pwm,integral_error,base_pwm=200;
float kp=2,ki=0.5,total_error,proportional_error;

void motor_setup(){
  ledcSetup(pwm_channel_mr_a , 5000, 8 ); //2^8 = 256 , 0-255
  ledcSetup(pwm_channel_mr_b , 5000, 8 ); //2^8 = 256 , 0-255
  ledcSetup(pwm_channel_ml_a , 5000, 8 ); //2^8 = 256 , 0-255
  ledcSetup(pwm_channel_ml_b , 5000, 8 ); //2^8 = 256 , 0-255


  ledcAttachPin(motor_right_a,pwm_channel_mr_a);
  ledcAttachPin(motor_right_b,pwm_channel_mr_b);
  ledcAttachPin(motor_left_a,pwm_channel_ml_a);
  ledcAttachPin(motor_left_b,pwm_channel_ml_b);
}

void frente(int speed) {
  ledcWrite(pwm_channel_mr_a, speed);
  ledcWrite(pwm_channel_ml_a, speed);
  ledcWrite(pwm_channel_mr_b, 0);
  ledcWrite(pwm_channel_ml_b, 0);
}


void direita(int speed) {
  ledcWrite(pwm_channel_mr_a, 0);
  ledcWrite(pwm_channel_ml_a, speed);
  ledcWrite(pwm_channel_mr_b, 0);
  ledcWrite(pwm_channel_ml_b, 0);
}*/

