uint8_t readcmd[5];  
uint8_t writecmd[8]={ '$', 0x03, 0,0,0,0,'\r', '\n'};
int16_t MotorCMD[4]={1,0,1,1};


void setup() {
  pinMode(13,OUTPUT);
///////serial////////////////////
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(10);
/////////////////////////////////
  
////////////cny70////////////////
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
/////////////////////////////////
}

void loop() {
////////// read data ////////////
  if(Serial.available()){
    size_t sizebyte = Serial.readBytes(readcmd, 5);
    if(readcmd[4]==10){
      for(int i=0;i<4;i++){
        MotorCMD[i] = (int16_t)readcmd[i];
      }
    }else{
      for(int i=0; i<4; i++){
        readcmd[i] = 0;
      }
    }
  }
  
/////////////////////////////////

  /*Serial.print(map(analogRead(A0),0,1023,0,255));
  Serial.print("\t");
  Serial.print(map(analogRead(A1),0,1023,0,255));
  Serial.print("\t");
  Serial.print(map(analogRead(A3),0,1023,0,255));
  Serial.print("\t");
  Serial.println(map(analogRead(A4),0,1023,0,255));*/

  Serial.print(MotorCMD[0]);
  Serial.print("\t");
  Serial.print(MotorCMD[1]);
  Serial.print("\t");
  Serial.print(MotorCMD[2]);
  Serial.print("\t");
  Serial.println(MotorCMD[3]);

  if(MotorCMD[0]==50){
    digitalWrite(13,HIGH);
  }else{
    digitalWrite(13,LOW);
  }




////////////cny70////////////////


/////////////////////////////////

////////// write data ///////////
//  for(int i=0;i<4;i++){
//    writecmd[i+2] = (uint8_t)MotorCMD[i];
//  }


  writecmd[2]= map(analogRead(A0),0,1023,0,255);
  writecmd[3]= map(analogRead(A1),0,1023,0,255);
  writecmd[4]= map(analogRead(A3),0,1023,0,255);
  writecmd[5]= map(analogRead(A4),0,1023,0,255);
  Serial.write(writecmd, 8);
/////////////////////////////////
}
