uint8_t readcmd[5];  
uint8_t writecmd[7]={ '$', 0x03, 0,0,0,0, '\n'};
int16_t MotorCMD[4]={0};

void setup() {
///////serial////////////////////
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(10);
/////////////////////////////////



}

void loop() {
////////// read data ////////////
  if(Serial.available()>5){
    size_t sizebyte = Serial.readBytes(readcmd, 5);
    if(readcmd[4]!=36){
      for(int i=0; i<4; i++){
        readcmd[i] = 0;
      }
    }
  }
  for(int i=0;i<4;i++){
    MotorCMD[i] = (int16_t)readcmd[i];
  }
/////////////////////////////////

//  for(int i=0;i<4;i++){
//    Serial.print(i+1);
//    Serial.print("\t");
//    Serial.print(MotorCMD[i]);
//    Serial.print("\t");
//  }
//  Serial.print("\n");





////////// write data ///////////
  for(int i=0;i<4;i++){
    writecmd[i+2] = (uint8_t)MotorCMD[i];
  }
  Serial.write(writecmd, 7);
/////////////////////////////////
}
