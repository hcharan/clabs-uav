#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 8, TXPin = 9;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
int arm=5;
int land=6;
int arm_val=0;
int land_val=0;
int arm_flag=0;
int land_flag=0;
int flag=0;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);
}

void loop()
{
  arm_val=digitalRead(arm);
  land_val=digitalRead(land);
  Serial.print("arm button");
  Serial.println(arm_val);
  Serial.print("land button");
  Serial.println(land_val);
  if(arm_val==0){
    arm_flag=1;
  }
  if(arm_flag==1 && arm_val==1){
    arm_flag=2;
  }
  if(land_val==0){
    land_flag=1;
  }
  if(land_flag==1 && land_val==1){
    land_flag=2;
  }
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
   if (gps.encode(Serial1.read())){
      displayInfo();
      if(arm_flag==2)
      {
        for(int i=0;i<10;i++)
        {
          Serial.println("Arm");
          flag=i;
          }
         if(flag==9){
          arm_flag=3; 
        }
      }
      else if(arm_flag==1)
      {
          Serial.println("Disarm");
      }
      if(land_flag==2)
      {
          Serial.println("Land");
       }
    }
 }

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    Serial.print("\n");
  }
  else
  {
    Serial.print("INVALID\n");
  }  
}



