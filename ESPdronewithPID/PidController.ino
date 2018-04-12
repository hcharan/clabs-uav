
//All PID variabless declared in Define file

void pid_setup()
{
  roll_rate_controller.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
  pitch_rate_controller.SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
  yaw_rate_controller.SetOutputLimits(YAW_PID_MIN, YAW_PID_MAX);

  roll_rate_controller.SetMode(AUTOMATIC);
  pitch_rate_controller.SetMode(AUTOMATIC);
  yaw_rate_controller.SetMode(AUTOMATIC);

  roll_rate_controller.SetSampleTime(10);
  pitch_rate_controller.SetSampleTime(10);
  yaw_rate_controller.SetSampleTime(10);

  roll_stab_controller.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
  pitch_stab_controller.SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
  yaw_stab_controller.SetOutputLimits(YAW_PID_MIN, YAW_PID_MAX);

  roll_stab_controller.SetMode(AUTOMATIC);
  pitch_stab_controller.SetMode(AUTOMATIC);
  yaw_stab_controller.SetMode(AUTOMATIC);

  roll_stab_controller.SetSampleTime(10);
  pitch_stab_controller.SetSampleTime(10);
  yaw_stab_controller.SetSampleTime(10);
}

void pid_stab()
{
  //Updating actual ypr
  pidStabIn[2] = ypr[2];
  pidStabIn[1] = ypr[1];
  pidStabIn[0] = ypr[0];

  //Updating desired ypr
  pidStabSetpoint[2] = rcInput[2];
  pidStabSetpoint[1] = rcInput[1];
  pidStabSetpoint[0] = yaw_set;

  pid_stab_compute_outputs();  
}

void pid_stab_compute_outputs()
{
  //Outputs are written to pidOut[]  (declared in Define file)
  roll_stab_controller.SetTunings(prStabKp, prStabKi, prStabKd);
  roll_stab_controller.Compute();
  pitch_stab_controller.SetTunings(prStabKp, prStabKi, prStabKd);
  pitch_stab_controller.Compute();
  yaw_stab_controller.SetTunings(yawStabKp, yawStabKi, yawStabKd);
  yaw_stab_controller.Compute();
}

void pid_rate()
{
  //Updating actual ypr
  pidRateIn[2] = g.x;
  pidRateIn[1] = g.y;
  pidRateIn[0] = g.z;

  //Updating desired ypr
  pidRateSetpoint[2] = pidStabOut[2];
  pidRateSetpoint[1] = pidStabOut[1];

  if (rcInput[0]>5 || rcInput[0]<-5)
  {
    pidRateSetpoint[0] = rcInput[0];//Rate Mode
    yaw_set = ypr[0];
    yaw_rate_controller.SetTunings(4*yawRateKp, yawRateKi, yawRateKd);
  
  }
  else
  {
    pidRateSetpoint[0] = pidStabOut[0];//Stabilize Mode
    yaw_rate_controller.SetTunings(yawRateKp, yawRateKi, yawRateKd);
  
  }
  
  pid_rate_compute_outputs();  
}

void pid_rate_compute_outputs()
{
  //Outputs are written to pidOut[]  (declared in Define file)
  
  roll_rate_controller.SetTunings(prRateKp, prRateKi, prRateKd);
  roll_rate_controller.Compute();
  pitch_rate_controller.SetTunings(prRateKp, prRateKi, prRateKd);
  pitch_rate_controller.Compute();
  yaw_rate_controller.Compute();
}
