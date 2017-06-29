void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(9600);
}

void sendUBX(byte *UBXmsg, byte msgLength) {
   for(int i = 0; i < msgLength; i++){
      Serial3.write(UBXmsg[i]);
   }
 }
void loop() {
  // put your main code here, to run repeatedly:
  byte initialConf[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB}; 
  byte gpsmsg[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x64, 0x00, 0x01, 0x00};
  sendUBX(initialConf, sizeof(initialConf));
 // sendUBX(request, 8);
  //Serial3.write(8);
  //Serial3.print("wrote");
  //Serial3.flush();
  //Serial.println(Serial.available());
  //Serial3.write("Hello");
  while(Serial3.available()){
    Serial.print((char)Serial3.read());
  }
}
