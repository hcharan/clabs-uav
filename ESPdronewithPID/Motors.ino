
void motor_setup()
{
  //Pinouts of PWM motors
  for (int i = 0 ; i < 4 ; i++)
  {
    pinMode(motor[i], OUTPUT);
    esc[i].attach(motor[i], PWM_MIN, PWM_MAX);
   // digitalWrite(motor[i], LOW);
    esc[i].writeMicroseconds(0);
  }
}

void motor_compute_outputs()
{
  //Uncomment for y,p,r,t -> individual LEDs
  //for(int i=0 ; i<4 ; i++)
  //  analogWrite(motor[i], rcInput[i]);
  //if ((rcInput[0] <= 3) || (rcInput[0] >= -3))
 
  

  if (rcInput[3]>1080)
  {
    motorValue[0] = double(rcInput[3]) - pidRateOut[1] - pidRateOut[2] + pidRateOut[0];
    motorValue[1] = double(rcInput[3]) + pidRateOut[1] + pidRateOut[2] + pidRateOut[0];
    motorValue[2] = double(rcInput[3]) - pidRateOut[1] + pidRateOut[2] - pidRateOut[0];
    motorValue[3] = double(rcInput[3]) + pidRateOut[1] - pidRateOut[2] - pidRateOut[0]; //Reversed due to reversed motors
  }
  for (int i = 0 ; i < 4 ; i++)
  {
    if(motorValue[i] < 0)
      motorValue[i] = 0;
  }

  motor_arm_check();
  if(imuStable)
    motor_write();
}

void motor_arm_check()
{
  if (rcInput[3] < ARMING_THR_LIMIT)
  {
    for (int i = 0 ; i < 4 ; i++)
      motorValue[i] = 0;
    //Serial.println("thrInput < ARMING_THR_LIMIT");
  }
}

void motor_write()
{
  for (int i = 0 ; i < 4 ; i++)
  {
    //analogWrite(motor[i], motorValue[i]);
    esc[i].writeMicroseconds(motorValue[i]);
#ifdef DEBUG_MOTOR_PWM
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(pidStabOut[i]);
    Serial.print(pidRateOut[i]);
    Serial.print("\t");
#endif
  }
#ifdef DEBUG_MOTOR_PWM
 Serial.println();
#endif
#ifdef LOG_YPR_AND_PWM
    Serial.print(motorValue[0]);
    Serial.print(" ");
    Serial.print(motorValue[1]);
    Serial.print(" ");
    Serial.print(motorValue[2]);
    Serial.print(" ");
    Serial.print(motorValue[3]);
    
    Serial.print(" ");
    Serial.print(ypr[0]);
    Serial.print(" ");
    Serial.print(ypr[1]);
    Serial.print(" ");
    Serial.println(ypr[2]);
    //Serial.print(" ");
    
#endif
}

