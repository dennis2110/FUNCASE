#include<Motor.h>
#define EN_R 9
#define EN_L 6
#define R1 4
#define R2 5
#define L1 2
#define L2 3

#define SEN_R 13
#define SEN_L 11

Motor motorR(EN_R, R1, R2);
Motor motorL(EN_L, L1, L2);
uint8_t readcmd[5];  
uint8_t writecmd[11]={ '$', 0x03, 0,0,0,0,0,0,0, '\r','\n'};
int16_t MotorCMD[4]={0,0,0,0};

void setup() {
///////serial////////////////////
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(10);
/////////////////////////////////

////////////cny70////////////////
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);

  pinMode(SEN_R, INPUT);
  pinMode(SEN_L, INPUT);
/////////////////////////////////
}

void loop() {
////////// read data ////////////
  if(Serial.available()>5){
    size_t sizebyte = Serial.readBytes(readcmd, 5);
    if(readcmd[4]==10){
      for(int i=0; i<4; i++){
        MotorCMD[i] = (int16_t)readcmd[i];
      }
    }else{
      for(int i=0; i<4; i++){
        readcmd[i] = 0;
      }
    }  
  }
  
/////////////////////////////////
//  for(int i=0;i<4;i++){
//    Serial.print(i+1);
//    Serial.print("\t");
//    Serial.print(MotorCMD[i]);
//    Serial.print("\t");
//  }
//  Serial.print("\n");

  switch(MotorCMD[1]){
    case 0:
      motorR.Standby();
      break;
    case 1:
      motorR.Run(MotorCMD[0]);
      break;
    case 2:
      motorR.Run(-MotorCMD[0]);
      break;
    case 3:
      motorR.Stop(MotorCMD[0]);
      break;
    default:
      motorR.Stop(255);
      break;    
  }
  switch(MotorCMD[3]){
    case 0:
      motorR.Standby();
      break;
    case 1:
      motorL.Run(MotorCMD[2]);
      break;
    case 2:
      motorL.Run(-MotorCMD[2]);
      break;
    case 3:
      motorL.Stop(MotorCMD[2]);
      break;
    default:
      motorL.Stop(255);
      break;    
  }

////////// write data ///////////
//  for(int i=0;i<4;i++){
//    writecmd[i+2] = (uint8_t)MotorCMD[i];
//  }

  writecmd[2]= map(analogRead(A0),0,1023,0,255);
  writecmd[3]= map(analogRead(A1),0,1023,0,255);
  writecmd[4]= map(analogRead(A2),0,1023,0,255);
  writecmd[5]= map(analogRead(A3),0,1023,0,255);
  writecmd[6]= map(analogRead(A4),0,1023,0,255);
  writecmd[7]= map(analogRead(A5),0,1023,0,255);
  writecmd[8]= (digitalRead(SEN_R)+(digitalRead(SEN_L)*2));
  Serial.write(writecmd, 11);
/////////////////////////////////
}
