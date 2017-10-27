///////////////////////////////


#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <string.h>

#define NO_OF_PORTS 2
#define DEBUG_WIFI_INPUT 1

//Local AP SSID and Password
const char WiFiApPwd[20] = "password";
const char WiFiApSsid[20] = "batman";

//UDP Parameters
unsigned int localPort[NO_OF_PORTS] = {7000, 5000};
byte packetBuffer[512];
WiFiUDP UdpPort[NO_OF_PORTS];

 
//Input Strings
char pidType[10];
char thrStr[4] = "", yawStr[4] = "", pitchStr[4] = "", rollStr[4] = "";
int rcInput[4] = {0};
float inputKp, inputKi, inputKd;

//App input integer value buffer to check for failsafe
int inputBuffer[4] = {0,0,0,0};

//Final Input Values
int yawInput, pitchInput, rollInput, thrInput;

byte userConnected = 0;



//----------FAILSAFE----------//

#define LAND_RATE 1
#define NO_PACKET_LIMIT 150

unsigned int failsafeCounter = 0;
byte longFailsafeTriggered = 0;
unsigned int noPacketCounter = 0;









//////////////////////////////////////
unsigned int SBUS_Channel_Data[18];
byte SBUS_Current_Channel = 0;
byte SBUS_Current_Channel_Bit = 0;
byte SBUS_Current_Packet_Bit = 0;
byte SBUS_Packet_Data[25];
byte SBUS_Packet_Position = 0;

byte SBUS_Failsafe_Active = 0;
byte SBUS_Lost_Frame = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Nae Jagi Saranghae!!");
  Serial1.begin(100000,SERIAL_8E2);
   udp_setup();
  //Serial1.begin(115200);
  
  SBUS_Channel_Data[4] = 500;
  SBUS_Channel_Data[5] = 1250;
  SBUS_Channel_Data[6] = 1500;
  SBUS_Channel_Data[7] = 2000;
  SBUS_Channel_Data[8] = 1024;
  SBUS_Channel_Data[9] = 1024;
  SBUS_Channel_Data[10] = 1024;
  SBUS_Channel_Data[11] = 1024;
  SBUS_Channel_Data[12] = 1792;
  SBUS_Channel_Data[13] = 1024;
  SBUS_Channel_Data[14] = 1024;
  SBUS_Channel_Data[15] = 1024;
  SBUS_Channel_Data[16] = 0;
  SBUS_Channel_Data[17] = 0;
  SBUS_Failsafe_Active = 1;
  SBUS_Lost_Frame = 1;
}

void SBUS_Build_Packet(void)
{
  for(SBUS_Packet_Position = 0; SBUS_Packet_Position < 25; SBUS_Packet_Position++) SBUS_Packet_Data[SBUS_Packet_Position] = 0x00;  //Zero out packet data
  
  SBUS_Current_Packet_Bit = 0;
  SBUS_Packet_Position = 0;
  SBUS_Packet_Data[SBUS_Packet_Position] = 0x0F;  //Start Byte
  SBUS_Packet_Position++;
  
  for(SBUS_Current_Channel = 0; SBUS_Current_Channel < 16; SBUS_Current_Channel++)
  {
    for(SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++)
    {
      if(SBUS_Current_Packet_Bit > 7)
      {
        SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        SBUS_Packet_Position++;       //Move to the next packet byte
      }
      SBUS_Packet_Data[SBUS_Packet_Position] |= (((SBUS_Channel_Data[SBUS_Current_Channel]>>SBUS_Current_Channel_Bit) & 0x01)<<SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data byte
      SBUS_Current_Packet_Bit++;
    }
  }
  if(SBUS_Channel_Data[16] > 1023) SBUS_Packet_Data[23] |= (1<<0);  //Any number above 1023 will set the digital servo bit
  if(SBUS_Channel_Data[17] > 1023) SBUS_Packet_Data[23] |= (1<<1);
  if(SBUS_Lost_Frame != 0) SBUS_Packet_Data[23] |= (1<<2);          //Any number above 0 will set the lost frame and failsafe bits
  if(SBUS_Failsafe_Active != 0) SBUS_Packet_Data[23] |= (1<<3);
  SBUS_Packet_Data[24] = 0x00;  //End byte
}

void loop()
{

  udp_input();
  SBUS_Build_Packet();
  Serial1.write(SBUS_Packet_Data,25);
  //Serial.write(SBUS_Packet_Data,25);
  //delay(7);
}


///////////////////////////////////////////////////////////////////////////////////////////

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
    
  
    
      yawInput = inputBuffer[0];
      pitchInput = inputBuffer[1];
      rollInput = inputBuffer[2];
      thrInput = inputBuffer[3];
      Serial.println(yawInput);
      Serial.println(pitchInput);
      Serial.println(rollInput);
      Serial.println(thrInput);

      
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

    
  
   
  }
  rc_compute();
}

void rc_compute()
{
  Serial.println("came to rc compute\r\n");
  //YAW
  if (yawInput > -5 && yawInput < 5)
    SBUS_Channel_Data[3] = 0;
  else
    SBUS_Channel_Data[3] = map(yawInput, -50, 50, 500, 2000);

  //PITCH
  if (pitchInput > -5 && pitchInput < 5)
    SBUS_Channel_Data[1] = 0;
  else
    SBUS_Channel_Data[1] = map(pitchInput, -50, 50, 500, 2000);

  //ROLL
  if (rollInput > -5 && rollInput < 5)
    SBUS_Channel_Data[0] = 0;
  else
    SBUS_Channel_Data[0] = map(rollInput, -50, 50, 2000, 500);

  //THROTTLE
  SBUS_Channel_Data[2] = map(thrInput, 0, 100, 500, 2000);   //Throttle limiting and arming is handled by Motors file
}
