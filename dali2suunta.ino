const int TX_PIN = 15;
const int RX_PIN = 2;
const int delay_1 = 412;
const int delay_2 = 414;
const int timeoutMicros = 20000;
volatile uint8_t ret = 0;
volatile bool change = false;
const uint8_t broadcastDP = 0b11111110;
const uint8_t broadcastC = 0b11111111;


void setup() {
  // put your setup code here, to run once:
  pinMode(RX_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);
  transmit(0b00000000, 00000111);
  Serial.begin(115200);
  initialisation();
  
}

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

//TODO: korjaa
//JOS LUETTU TAVU 0xFF PALAUTTAA VÄÄRIN 0
uint8_t receive() {  
  
  int startTime = micros();
  attachInterrupt(digitalPinToInterrupt(RX_PIN), readRX, FALLING);
  while(micros() - startTime < timeoutMicros) {
    if(change) {
      change = false;
      delayMicroseconds(620);
      attachInterrupt(digitalPinToInterrupt(RX_PIN), readRX, CHANGE);
    }
  }
  
  Serial.print("HEX: ");
  Serial.println((~ret) & 0xFF, HEX);
  Serial.print("BIN: ");
  Serial.println((~ret) & 0xFF, BIN);
  uint8_t response = 0;
  response = (~ret) & 0xFF;
 
  
  ret = 0;
  return response;  
}

void readRX() {
  
  ret = ret << 1;
  ret = ret | digitalRead(RX_PIN);
  detachInterrupt(digitalPinToInterrupt(RX_PIN)); 
  change = true;
}

void initialisation() {
  delay(500);
  const int delaytime = 20; //ms
  long low_longadd = 0x000000;
  long high_longadd = 0xffffff;
  long longadd = (low_longadd + high_longadd) / 2;
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
  transmit(0b10100111, 0b00000000); // randmomize addresses
  delay(delaytime);
  transmit(0b10100111, 0b00000000);

  while(high_longadd - low_longadd > 1) {
    //long address sent to luminaries
    Serial.println(high_longadd, DEC);
    Serial.println(low_longadd, DEC);
    Serial.println(longadd, DEC);
    transmitLongAdd(longadd);
    //check if any long address <= the one sent
    transmit(0b10101001, 0b00000000); //compare
    if(receive() != 0) {
      high_longadd = longadd;
    } else {
      low_longadd = longadd;
    }
    
    longadd = (low_longadd + high_longadd) / 2;
    Serial.println("-----------");
  }

  
  //check if luminary is in high or low longadd
  transmitLongAdd(low_longadd);
  transmit(0b10101001, 0b00000000); //compare

  if(receive() != 0) {
    transmit(0b10110111, 0b00000011); //program short address 000001
    Serial.println("Low long Address found. Assigning it short address 000001");
    
  } else {
    transmitLongAdd(high_longadd);
    transmit(0b10110111, 0b00000000); //compare
    if(receive() != 0){
      transmit(0b10110111, 0b00000011); //program short address 000001
      Serial.print("High longAddress found. Assigning it short address 000001");
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
void loop() {
  transmit(0b00000001, 0b00000110);
  delay(100);
  transmit(0b00000001, 0x90);
  Serial.println(receive());
  Serial.println("------------");
  delay(1000);
  
  transmit(0b00000001, 0b00000000);
  delay(100);
  transmit(0b00000001, 0x90);
  Serial.println(receive());
  Serial.println("------------");
  delay(1000);
}
