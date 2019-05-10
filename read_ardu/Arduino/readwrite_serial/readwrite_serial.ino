#define LED_PIN 13
uint8_t message[5] = { '$', 0x03, 0,0, '\n'};
char readcmd[6];
char chr;
int i=0;
int ten[3] = {100,10,1};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(7, INPUT);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  Serial.setTimeout(10);
  
}

void loop() {
  int switchStatus = digitalRead(7);
  if(switchStatus == 0){
    message[2] = 79;
    message[3] = 80;
    //Serial.println(switchStatus);
    Serial.write(message, 5);
  }else{
    message[2] = 67;
    message[3] = 76;
    //Serial.println(switchStatus);
    Serial.write(message, 5);
  }
  
//  if(Serial.available()){
//    //read data
//    i=0;
//    Serial.println("read date");
//    while((chr = Serial.read()) != '\n'){
//      if(chr >= '0' && chr <= '9' && i < 6) {
//        readcmd[i] = chr;
//        Serial.print(readcmd[i]);
//        Serial.print("\t");
//        i++;
//      }
//    }
//    Serial.print("\n");
//
//    
//    //data 0~2
//    int sum=0;
//    for(int i=0;i<3;i++){
//      sum += (int)(readcmd[i]-48) * ten[i];
//      message[2] = sum;
//    }
//    //Serial.println(message[2]);
//    //data 3~5
//    sum=0;
//    for(int i=0;i<3;i++){
//      sum += (int)(readcmd[i+3]-48) * ten[i];
//      message[3] = sum;
//    }
//    //Serial.println(message[3]);
//
//    
//    //write data
//    Serial.println("write date");
//    Serial.write(message, 5);
//  }
  
}
