#include <string.h>
#include <Servo.h>
#define IBUS_BUFFSIZE  32
#define IBUS_MAXCHANNELS 5 // we are going to use 4 channels //(ibus protocol suports up to 14 channels // 
#define PIN_SERVO_CAMERA 6
#define PIN_ESC 7
#define PIN_SERVO_DIRECTION 8
#define PIN_RELAY_BACKWORDS 9



static uint8_t ibusIndex = 0;    
static uint8_t ibus[IBUS_BUFFSIZE] = {0};  // will save the receved 32bit packet data 
static uint16_t rcController[IBUS_MAXCHANNELS] = {0};  // will save the channel values 
static bool rxPacketDone = 0; // will check if the full packet is recived 
static uint8_t bitValue = 0;  // Temporatry saved readed bite 

int ch_width_1;
int ch_width_2;
int ch_width_3;
int ch_width_4;



Servo servoCamera;
Servo ESC;
Servo servoDirection;

void setup(){
  
  Serial.begin(115200);     // our recivers suport only 115200 baud rate ( 115200 bits/sec)
  servoCamera.attach(PIN_SERVO_CAMERA);
  ESC.attach(PIN_ESC);
  servoDirection.attach(PIN_SERVO_DIRECTION);
  pinMode(PIN_RELAY_BACKWORDS, HIGH);
}

void loop() {
  
  readData();
  fromArduinoPWM();                                                                       
}                                                                        
                                                                        

void fromArduinoPWM(){
       ch_width_1 = map(rcController[0], 1000, 2000, 1000, 2000);         // writing the data in to the arduino  
       servoCamera.writeMicroseconds(ch_width_1);                         // adding logic for reverce rotation switch because
       ch_width_2 = map(rcController[1], 1000, 2000, 1000, 2000);         // we are using one directional ESC
       ESC.writeMicroseconds(ch_width_2);
       ch_width_3 = map(rcController[3], 1000, 2000, 1000, 2000);
       servoDirection.writeMicroseconds(ch_width_3); 
        ch_width_4 = map(rcController[4], 1000, 2000, 1000, 2000);
       if(ch_width_4 > 1600){
        digitalWrite(PIN_RELAY_BACKWORDS, LOW);
       }
       else{
        digitalWrite(PIN_RELAY_BACKWORDS, HIGH);
       }
}

void readData(){
  rxPacketDone = false;
  if(Serial.available()){
     bitValue = Serial.read();                                              // reading bits
     if( (ibusIndex == 0 && bitValue!=0x20) || (ibusIndex == 1 && bitValue!=0x40) ){  // we are looking for start bites 0x2040 to begin saving data
     return;                                                                
     }

     if(ibusIndex == IBUS_BUFFSIZE){                                       // We already hit the end of the packet so we update the rcController values
      ibusIndex = 0;                                                       // setting the index to 0 so we can start reading the next packet 
      uint8_t high = 3;                                                    
      uint8_t low = 2;
      for(int i = 0 ; i < IBUS_MAXCHANNELS; ++i){
        rcController[i] = (ibus[high]<<8) + ibus[low];
        high+=2;
        low +=2;
        }
        ibusIndex  = true;                                               // Whole packed is readed adn we are done 
        return;
      }
      else{
        ibus[ibusIndex] = bitValue;                                     // We hit the start bits and now we are saving the data 
        ibusIndex++;
      }

  }
}
