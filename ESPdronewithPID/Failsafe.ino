
void failsafe(byte failsafeType)
{
  int longFailsafeTimeout = 0;

  if (failsafeType)
  {
    longFailsafeTimeout = 300;
    //Serial.println("WiFi Failsafe");
  }
    
  else
  {
    longFailsafeTimeout = 150; 
    //Serial.println("LockScreen Failsafe");
  }
    
  for(int i=0; i<3; i++)
    rcInput[i] = 0;

  if(failsafeCounter > longFailsafeTimeout)
  {
    longFailsafeTriggered = 1;
    rcInput[3] -= LAND_RATE;
    if(rcInput[3] <0)
      rcInput[3] = 0;
    delay(10);
    //Serial.println("looooooooooooong failsafe..........................................................");
  }
  else
    failsafeCounter++;

  if(!motorValue[0] && !motorValue[1] && !motorValue[2] && !motorValue[3])
    {
      longFailsafeTriggered = 0;
    }
}

/*
void long_failsafe()
{
  
  int landRate = 1;
  while((motorValue[0]+motorValue[1]+motorValue[2]+motorValue[3]) > 300)
  {
    for(int i=0; i<4; i++)
    {
      motorValue[i] -= landRate;
      delay(10);
    }
    
  }
}
*/

