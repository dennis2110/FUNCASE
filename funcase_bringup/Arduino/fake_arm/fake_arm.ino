uint8_t testcmd[17] = {101, 0, 102, 0, 103, 0, 104, 0, 105, 0, 51, 0, 52, 0, 53, 0, 54};
uint8_t testprint[4] = {1,2,3,4};

uint8_t readcmd[17];
uint8_t writecmd[5] = {'$', 0x03, 0, '\r', '\n'};
uint8_t right_armcmd[5] = {0, 0, 0, 0, 0};
uint8_t left_armcmd[4] = {0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(10);

}

void loop() {
  ////////// read data ///////////
  if(Serial.available()>17){
    size_t sizebyte = Serial.readBytes(readcmd, 17);
    int count = 0;
    for(int i=0;i<17; i++){
      if(readcmd[i]!=0){
        if(i<10){
          right_armcmd[count] = readcmd[i];
          count++;
          if(i==8){
            count = 0;
          }
        }else{
          left_armcmd[count] = readcmd[i];
          count++;
        }
      }
    }
  }
  printarr(right_armcmd, 5);
  printarr(left_armcmd, 4);
  /////////////////////////////////


  ////////// write data ///////////
  writecmd[2]= 155;
  Serial.write(writecmd, 5);
  /////////////////////////////////
}

/////////////function//////////////
void printarr(uint8_t *arr, size_t num){
  for(int i=0;i<num;i++){
    Serial.print(arr[i]);
    Serial.print("\t");
  }
  Serial.print("\n");
}
