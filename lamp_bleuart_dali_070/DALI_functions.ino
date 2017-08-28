void sendZero() {
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(delay_1);
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(delay_2);
}

void sendOne() {
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(delay_1);
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(delay_1);
}

void sendBit(int i) {
  if(i) {
    sendOne();
  } else {
    sendZero();
  }
}

void sendByte(uint8_t cmd) {
  for(int i = 7; i >= 0; i--) {
  sendBit( (cmd >> i) & 1);
  }
}

void transmit(uint8_t byte1, uint8_t byte2) {
  sendBit(1);
  sendByte(byte1);
  sendByte(byte2);
  digitalWrite(TX_PIN, LOW);
}


uint8_t receive() {  
  ret = 0;
  int startTime = micros();
  attachInterrupt(digitalPinToInterrupt(RX_PIN), readRX, FALLING);
  while(micros() - startTime < timeoutMicros) {
    if(change) {
      change = false;
      delayMicroseconds(620);
      attachInterrupt(digitalPinToInterrupt(RX_PIN), readRX, CHANGE);
    }
  }
  
  /*
  Serial.print("HEX: ");
  Serial.println(ret, HEX);
  Serial.print("BIN: ");
  Serial.println(ret, BIN);
   */
  detachInterrupt(digitalPinToInterrupt(RX_PIN));
  return ret;
}

void readRX() {
  
  ret = ret << 1;
  ret = ret | ((~digitalRead(RX_PIN)) & 1);
  detachInterrupt(digitalPinToInterrupt(RX_PIN)); 
  change = true;
}

void readChange() {
  change = true;
  detachInterrupt(digitalPinToInterrupt(RX_PIN));
}

void initialisation() {
  delay(500);
  const int delaytime = 20; //ms
  long low_longadd = 0x000000;
  long high_longadd = 0xffffff;
  long longadd = (low_longadd + high_longadd) / 2;
  uint8_t rec;
  
  transmit(broadcastC, 0b00100000); //reset
  delay(delaytime);
  transmit(broadcastC, 0b00100000); //reset
  delay(delaytime);
  transmit(broadcastC, 0b00000000); //turn luminaries off
  delay(delaytime);

  //start initialisaton
  transmit(0b10100101, 0b00000000); //initialize
  delay(delaytime);
  transmit(0b10100101, 0b00000000);
  delay(delaytime);
  transmit(0b10100111, 0b00000000); // randomize addresses
  delay(delaytime);
  transmit(0b10100111, 0b00000000);
  delay(delaytime);



  while(high_longadd - low_longadd > 1) {
    //long address sent to luminaries
    Serial.println(high_longadd, HEX);
    Serial.println(low_longadd, HEX);
    Serial.println(longadd, HEX);
    transmitLongAdd(longadd);
    delay(delaytime);
    //check if any long address <= the one sent
    transmit(0b10101001, 0b00000000);//compare
    delayMicroseconds(1600);
    attachInterrupt(digitalPinToInterrupt(RX_PIN), readChange, FALLING);
    int startTime = micros();
    while(micros() - startTime < timeoutMicros) {
      if(change) {
        break; 
      }
    }
    if(change) {
      change = false;
      high_longadd = longadd;
    } else {
      detachInterrupt(digitalPinToInterrupt(RX_PIN));
      low_longadd = longadd;
    }
    
    longadd = (low_longadd + high_longadd) / 2;
    Serial.println("-----------");
    delay(delaytime);
  }
  detachInterrupt(digitalPinToInterrupt(RX_PIN)); //lisatty

  
  //check if luminary is in high or low longadd
  transmitLongAdd(low_longadd);
  delay(delaytime);
  transmit(0b10101001, 0b00000000); //compare
  delay(delaytime);

  rec = receive();
  if(rec != 0) {
    transmit(0b10110111, 0b00000001); //program short address 000000
    Serial.println("Low Long Address found. Assigning it short address 000000");
    
  } else {
    transmitLongAdd(high_longadd);
    delay(delaytime);
    transmit(0b10110111, 0b00000000); //compare
    if(receive() != 0){
      delay(delaytime);
      transmit(0b10110111, 0b00000001); //program short address 000000
      Serial.println("High Long Address found. Assigning it short address 000000");
    } else {
      Serial.println("Initialisation failed");
    }
  }
  Serial.print("Short address: ");
  transmit(0b10111011, 0b00000000);
  Serial.println(receive(), BIN);
  transmit(0b10101011, 0b00000000); //withdraw the found luminary from initialisation
  transmit(0b10100001, 0b00000000); //terminate initialisation for all luminaries

  //blink found luminary off and on
  transmit(0b00000011, 0b00000000); //off
  delay(500);
  transmit(0b00000011, 0b00000101); //recall max level
}

void transmitLongAdd(long longadd) {
  delay(20);
  transmit(0b10110001, (longadd >> 16));
  delay(20);
  transmit(0b10110011, (longadd >> 8));
  delay(20);
  transmit(0b10110101, longadd);
  delay(20);
}
//Goes through the short address space and returns the first found short address
//returns the valid short address, if no valid short address is found returns -1
uint8_t scan(){
  uint8_t add_byte;   //the actual byte sent to DALI
  uint8_t short_add;  //short address

  //loops through all 64 addresses and sends a query for version number
  //if a return byte is received, the loop has found a valid short address and the function returns it
  for(short_add = 0; short_add < 64; short_add++){
    add_byte = 1 + (short_add << 1); 
    
    delay(10);
    transmit(add_byte, 0x97); //query for version number
    delayMicroseconds(100);
    uint8_t response = receive();
    //Serial.print("Checking short address: " );
    //Serial.print(add_byte >> 1);
    //Serial.print(", received: ");
    //Serial.println(response);
    
    if(response != 0){
      delay(100);
      transmit(add_byte, 0x05);
      delay(500);
      transmit(add_byte, 0x00);
      //Serial.println("Short address found:");
      //Serial.println(short_add, BIN);
      return short_add;
    }
  }
  return -1;
}

//function to check minimun and maximum levels of the lamp
void levelCheck(){
  transmit(0b00000001, 0xA1);
  //delayMicroseconds(100);
  maxLevel = receive();
  delay(100);
  transmit(0b00000001, 0xA2);
  //delayMicroseconds(100);
  minLevel = receive();
  delay(100);
  /*Serial.println("min:");
  Serial.println(minLevel);
  Serial.println("max:");
  Serial.println(maxLevel);*/
}

uint8_t statusCheck(){
  uint8_t status;
  transmit(0b00000001, 0x90);
  delayMicroseconds(100);
  status = receive();
  return status;
}

void fadeToOff(){
    uint8_t on = 1;
    while(on != 0){
      transmit(0x01, 0x93);
      delayMicroseconds(100);
      on = receive();
      delay(1);
      transmit(0x01, 0x07); // dims one level
      delay(3);
       
  }
}

//changes colour temperature to value given as kelvins
void tempChange(int temp){ 
  uint16_t mirek = 1000000 / temp;
  uint8_t mirek_upper = mirek >> 8;
  uint8_t mirek_lower = mirek;

  delay(20);
  transmit(0xC3, mirek_upper); //data to dtr1
  delay(20);
  transmit(0xA3, mirek_lower); //data to dtr0
  delay(20);  
  transmit(0b11000001, 0b00001000); // activate application extended commands
  delay(20);
  transmit(0x01, 231); //set temporary colour temperature
  delay(20);
  transmit(0b11000001, 0b00001000); // activate application extended commands
  delay(20);  
  transmit(0x01, 226); //activate
  delay(20);
  
}

uint16_t queryColourTemp(){
  uint8_t response;
  delay(10);
  transmit(0xA3, 2); //data to dtr
  delay(10);
  transmit(0b11000001, 0b00001000); // activate application extended commands
  delay(10);
  transmit(0x01, 250); //query colour value
  delayMicroseconds(100);
  response = receive();
  
  delay(10);
  transmit(0x01, 0x9C); //read dtr1
  delayMicroseconds(100);
  response = receive();
  temp = response << 8;
  //Serial.print("DTR1: ");
  //Serial.println(response, BIN);
  delay(10);
  transmit(0x01, 152); //read dtr
  delayMicroseconds(100);
  response = receive();
  temp = temp + response;
  //Serial.print("DTR: ");
  //Serial.println(response, BIN);
  
  //Serial.print("temp: ");
  //Serial.println(temp);
  delay(10);

  temp = 1000000 / temp;
  return temp;
}

void queryColourTempLimits(){
  uint8_t response;

  //calucating warmest
  transmit(0xA3, 130); //data to dtr
  delay(10);
  transmit(0b11000001, 0b00001000); // activate application extended commands
  delay(10);
  transmit(0x01, 250); //query colour value
  delayMicroseconds(100);
  response = receive();
  
  delay(10);
  transmit(0x01, 0x9C); //read dtr1
  delayMicroseconds(100);
  response = receive();
  warmest = response * 256;
  delay(10);
  transmit(0x01, 152); //read dtr
  delayMicroseconds(100);
  response = receive();
  warmest = warmest + response;
  warmest = 1000000 / warmest;


  //calculating coolest
   transmit(0xA3, 128); //data to dtr
  delay(10);
  transmit(0b11000001, 0b00001000); // activate application extended commands
  delay(10);
  transmit(0x01, 250); //query colour value
  delayMicroseconds(100);
  response = receive();
  
  delay(10);
  transmit(0x01, 0x9C); //read dtr1
  delayMicroseconds(100);
  response = receive();
  coolest = response * 256;
  delay(10);
  transmit(0x01, 152); //read dtr
  delayMicroseconds(100);
  response = receive();
  coolest = coolest + response;
  coolest = 1000000 / coolest;
  coolest += 50;
}

void queryColourTempCapability() {
  uint8_t response;
  
  transmit(0b11000001, 0b00001000);
  delay(10);
  transmit(0x01, 249);
  delayMicroseconds(100);
  response = receive();
  if(bitRead(response, 1)) {
    colorTempCap = 1;
  }
  else {
    colorTempCap = 0;
  }
}

//function that reads temperature from sensors
float tempRead(int sensor){ 
  sensors.requestTemperatures();
  float temp;
  if(sensor == 0){
    temp = sensors.getTempCByIndex(0);  
  }
  else if(sensor == 1){
    temp = sensors.getTempCByIndex(1);
  }
  else{
    return -200;
  }
  return temp;
}

void updateSensorTemps() {
  float sensor0 = tempRead(0);
  sensor0_int = (uint8_t)sensor0;
  sensor0_frac = (uint8_t)( (sensor0-sensor0_int)*100 );
  if (DEBUG) {
    //Serial.print("Temp 0: ");
    //Serial.println(sensor0);
  }

  float sensor1 = tempRead(1);
  sensor1_int = (uint8_t)sensor1;
  sensor1_frac = (uint8_t)( (sensor1-sensor1_int)*100 );
  if (DEBUG) {
    //Serial.print("Temp 1: ");
    //Serial.println(sensor1);
  }
}

