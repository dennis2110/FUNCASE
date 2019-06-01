uint8_t readcmd[5];  
uint8_t writecmd[8]={ '$', 0x03, 0,0,0,0,'\r', '\n'};
int16_t MotorCMD[4]={1,1,1,1};

void setup() {
///////serial////////////////////
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(100);
/////////////////////////////////



}

void loop() {
////////// read data ////////////
  Serial.println(Serial.available());
  if(Serial.available()){
    size_t sizebyte = Serial.readBytes(readcmd, 5);
    if(readcmd[4]!=10){
      for(int i=0; i<4; i++){
        readcmd[i] = 0;
      }
    }
    for(int i=0;i<4;i++){
      MotorCMD[i] = (int16_t)readcmd[i];
    }
  }
  
/////////////////////////////////

  for(int i=0;i<4;i++){
    Serial.print(i+1);
    Serial.print("\t");
    Serial.print(MotorCMD[i]);
    Serial.print("\t");
  }
  Serial.print("\n");





////////// write data ///////////
  for(int i=0;i<4;i++){
    writecmd[i+2] = (uint8_t)MotorCMD[i];
  }
  //writecmd[2]= 10;
  Serial.write(writecmd, 8);
/////////////////////////////////
}
