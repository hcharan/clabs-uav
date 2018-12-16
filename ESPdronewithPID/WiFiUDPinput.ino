
//'Udp' declared in Define file

void udp_setup()
{
  // Setting up Local AP
  WiFi.softAP(WiFiApSsid, WiFiApPwd);
  Serial.println();
  Serial.println("Access Point Starting ...");
  delay(1000);
  Serial.println("Access Point Started");

  //Starting UDP Server
  //Serial.print("UDP server started at Port:");
  //Serial.println(localPort);
  for(int i=0; i<NO_OF_PORTS; i++)
    UdpPort[i].begin(localPort[i]);
    
}

void udp_input()
{
  //UDP Recieving
  int noBytesPort[NO_OF_PORTS];
  
  for(int i=0; i<NO_OF_PORTS; i++)
    noBytesPort[i] = UdpPort[i].parsePacket();
  
  if (noBytesPort[0])  //Controller Inputs
  {
    noPacketCounter = 0; //For failsafe
    read_input(noBytesPort[0], 0);
  }
  else
    noPacketCounter++;

  if (noBytesPort[1])  //PID Inputs
    read_input(noBytesPort[1], 1);
    
  if(noPacketCounter > NO_PACKET_LIMIT)
    failsafe(1);
  
  Serial.flush();
}

void read_input(int noBytes, byte portNumber)
{
    String receivedCommand = "";
    //We've received a packet, read the data from it
    UdpPort[portNumber].read(packetBuffer, noBytes); //Read the packet into the buffer
    
    for (int i = 1; i <= noBytes; i++)
      receivedCommand = receivedCommand + char(packetBuffer[i - 1]);

#ifdef DEBUG_WIFI_INPUT
    if(portNumber == 0)
      Serial.print(receivedCommand);
#endif

#ifdef DEBUG_APP_PID
    if(portNumber == 1)
      Serial.print(receivedCommand);
#endif

    //Decode Json into control variables
    command_decoder(receivedCommand, portNumber);
}


void command_decoder(String input, byte portNumber)
{
  if(!portNumber)
  {
    int i, j;
  
    int typeIndex = input.indexOf(": ");
    int thrIndex = input.indexOf(": ", typeIndex + 1);
    int yawIndex = input.indexOf(": ", thrIndex + 1);
    int pitchIndex = input.indexOf(": ", yawIndex + 1);
    int rollIndex = input.indexOf(": ", pitchIndex + 1);
  
    for (i = (thrIndex + 2), j = 0 ; input[i] != ',' ; i++, j++)
      thrStr[j] = input[i];
    thrStr[j] = '\0';
  
    for (i = (yawIndex + 2), j = 0 ; input[i] != ',' ; i++, j++)
      yawStr[j] = input[i];
    yawStr[j] = '\0';
  
    for (i = (pitchIndex + 2), j = 0 ; input[i] != ',' ; i++, j++)
      pitchStr[j] = input[i];
    pitchStr[j] = '\0';
  
    for (i = (rollIndex + 2), j = 0 ; input[i] != '}' ; i++, j++)
      rollStr[j] = input[i];
    rollStr[j] = '\0';
  
    inputBuffer[0] = atoi(yawStr);
    inputBuffer[1] = atoi(pitchStr);
    inputBuffer[2] = atoi(rollStr);
    inputBuffer[3] = atoi(thrStr);
  
  
    //Triggers failsafe if packets recieved are empty (app is closed or phone locked)
    if ((!inputBuffer[0] && !inputBuffer[1] && !inputBuffer[2] && !inputBuffer[3]) && (yawInput || pitchInput || rollInput || thrInput))
      failsafe(0);
  
    else
    {
      yawInput = inputBuffer[0];
      pitchInput = inputBuffer[1];
      rollInput = inputBuffer[2];
      thrInput = inputBuffer[3];
      if(!longFailsafeTriggered)
      {
        failsafeCounter = 0;
        rc_compute();
      }
      else
        failsafe(0);
    }
  }
  
  else
  {
    int i,j;
    char tempStr[5];
    int typeIndex = input.indexOf(": ");
    typeIndex = input.indexOf(": ", typeIndex + 1);
    
    int kpIndex = input.indexOf(": ", typeIndex + 1);
    int kiIndex = input.indexOf(": ", kpIndex + 1);
    int kdIndex = input.indexOf(": ", kiIndex + 1);
    
    for(i = (typeIndex+3), j = 0 ; input[i] != '"' ; i++, j++)
      pidType[j] = input[i];
    pidType[j] = '\0';

    for(i = (kpIndex+2), j = 0 ; input[i] != ',' ; i++, j++)
      tempStr[j] = input[i];
    tempStr[j] = '\0';
    inputKp = atof(tempStr);

    for(i = (kiIndex+2), j = 0 ; input[i] != ',' ; i++, j++)
      tempStr[j] = input[i];
    tempStr[j] = '\0';
    inputKi = atof(tempStr);
    
    for(i = (kdIndex+2), j = 0 ; input[i] != '}' ; i++, j++)
      tempStr[j] = input[i];
    tempStr[j] = '\0';
    inputKd = atof(tempStr);  

#ifdef DEBUG_APP_PID
    Serial.print(pidType);
    Serial.print("\t");
    Serial.print(inputKp, 3);
    Serial.print("\t");
    Serial.print(inputKi, 3);
    Serial.print("\t");
    Serial.println(inputKd, 3);
#endif
    
    String pidTypeStr(pidType);  //Converting to string format

    if(pidTypeStr.equals("pr_rate"))
    {       
       prRateKp = inputKp;
       prRateKi = inputKi;
       prRateKd = inputKd;
       //Serial.println(prRateKp);
    }
    
    else if(pidTypeStr.equals("pr_stab"))
    {       
       prStabKp = inputKp;
       prStabKi = inputKi;
       prStabKd = inputKd;
       //Serial.println(prStabKp);
    }
    
    else if(pidTypeStr.equals("yaw_rate"))
    {       
       yawRateKp = inputKp;
       yawRateKi = inputKi;
       yawRateKd = inputKd;
       //Serial.println(yawRateKp);
    }

    
    else if(pidTypeStr.equals("yaw_stab"))
    {       
       yawStabKp = inputKp;
       yawStabKi = inputKi;
       yawStabKd = inputKd;
       //Serial.println(yawRateKp);
    }
  
    else{};

  
    if(imuStable)
    {
      digitalWrite(BUILT_IN_LED, HIGH);
      delay(200);
      digitalWrite(BUILT_IN_LED, LOW);
    }
  }
}

void rc_compute()
{

  //YAW
  if (yawInput > -5 && yawInput < 5)
    rcInput[0] = 0;
  else
    rcInput[0] = map(yawInput, -50, 50, -30, 30);

  //PITCH
  if (pitchInput > -5 && pitchInput < 5)
    rcInput[1] = 0;
  else
    rcInput[1] = map(pitchInput, -50, 50, -30, 30);

  //ROLL
  if (rollInput > -5 && rollInput < 5)
    rcInput[2] = 0;
  else
    rcInput[2] = map(rollInput, -50, 50, -30, 30);

  //THROTTLE
  rcInput[3] = map(thrInput, 0, 100, 1000, 1800);   //Throttle limiting and arming is handled by Motors file
}


