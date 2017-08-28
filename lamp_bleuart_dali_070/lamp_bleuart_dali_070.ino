#include <bluefruit.h>
#include <OneWire.h>
#include <DallasTemperature.h>

bool DEBUG = true;
bool sensors_connected = true;

//Bluetooth variables etc.
int id = -1;
bool query_sent = false;
ble_gap_addr_t own_addr;
unsigned long quer_ref_time;
unsigned long conn_ref_time;
const char* name_of_lamp = "BLEL";
unsigned long update_ref_time;
bool first_update = false;
unsigned long scan_ref_time;
uint16_t c_conn_handle;

//DALI variables etc.
const int TX_PIN = 15;
const int RX_PIN = 2;
const int delay_1 = 412;
const int delay_2 = 414;
const int timeoutMicros = 20000;
volatile uint8_t ret = 0;
volatile bool change = false;
const uint8_t broadcastDP = 0b11111110;
const uint8_t broadcastC = 0b11111111;
uint8_t maxLevel;
uint8_t minLevel;
uint16_t warmest = 0;
uint16_t coolest = 0;
uint8_t colorTempCap = 0;
uint16_t temp = 0;
bool dali_setup = true;

//Sensor variables
const uint8_t onewirebus = 11;
OneWire oneWire(onewirebus);
DallasTemperature sensors(&oneWire);
uint8_t sensor0_int = 0;
uint8_t sensor0_frac = 0;
uint8_t sensor1_int = 0;
uint8_t sensor1_frac = 0;

// BLE Client Service (Connection to Peripheral aka next device)
BLEClientUart clientUart;

// BLE Peripheral Service (Connection to Central aka previous device)
BLEUart bleUart;

void setup()
{
  Serial.begin(115200);
  
  //----- DALI SETUP -----
  pinMode(RX_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);

  //transmit(0x00, 200);
  transmit(0b00000000, 0b00000111);

  //----- BLUETOOTH SETUP -----
  
  // Enable both peripheral and central
  Bluefruit.begin(true, true);

  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("BLEL");
  
  //----- PERIPHERAL CONNECTION SETUP -----
  
  // Callbacks for Peripheral
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start BLE Uart Service
  bleUart.begin();
  //bleUart.setRxCallback(handleDataFromCentral); //Didn't work, has to be called separately

  // Set up Advertising Packet
  setupAdv();

  //----- CENTRAL CONNECTION SETUP -----

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback_c);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback_c);

  // Init BLE Central Uart Service
  clientUart.begin();
  clientUart.setRxCallback(handleDataFromPeripheral);

  /* Setup Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback_c);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(bleUart.uuid);
  Bluefruit.Scanner.useActiveScan(false);

  //Get the bluetooth address of the nRF52 module
  sd_ble_gap_address_get(&own_addr);

  //----- SENSOR SETUP -----
  sensors.begin();
}

void setupAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleUart 128-bit uuid and name
  Bluefruit.Advertising.addService(bleUart);
  
  //Include name
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
}

void setupDALI() {
  delay(100);
  //If the address isn't 0000 0000, initialise 
  if (scan() != B00000000) {
    delay(10);
    initialisation();
  }
  delay(100);
  //Memorize the min and max levels of the lamp
  levelCheck();
  if (DEBUG) {
    Serial.print("MIN: ");
    Serial.println(minLevel);
    Serial.print("MAX: ");
    Serial.println(maxLevel);
  }
  
  //Memorize the coolest and warmest light temperatures of the lamp
  queryColourTempLimits();
  if (DEBUG) {
    Serial.print("WARM: ");
    Serial.println(warmest);
    Serial.print("COOL: ");
    Serial.println(coolest);
  }
  
  //Memorize if the color temperature of the lamp can be changed
  queryColourTempCapability();
  if (DEBUG) {
    Serial.print("TC CAP: ");
    Serial.println(colorTempCap);
  }

  //Memorize current color temperature
  queryColourTemp();
  if (DEBUG) {
    Serial.print("CURRENT COL TEMP: ");
    Serial.println(temp);
  }
  
  dali_setup = false;
}

void loop()
{
  if (dali_setup) setupDALI();
  
  checkConnectionStatuses();
  
  checkUpdateNeed();
  
  //handleDataFromSerial();
  
  handleDataFromCentral();
  
  //handleDataFromPeripheral() works automatically and there is no need to call it separately
}

void checkConnectionStatuses(void)
{
  if (millis()-scan_ref_time > 10000) Bluefruit.Scanner.stop();
  
  //Connection to central lost, disconnect from peripheral
  if ( !Bluefruit.connected()&& Bluefruit.Central.connected() ) {
    Bluefruit.Central.disconnect(c_conn_handle);
  }
  //Connection to central established, start scanning for peripherals & connect
  else if ( !Bluefruit.Scanner.isRunning() && millis()-scan_ref_time > 20000 && Bluefruit.connected() && !Bluefruit.Central.connected() ) {
    Bluefruit.Advertising.stop();
    Bluefruit.Scanner.start(0); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Serial.println("Scanning");
    scan_ref_time = millis();
  }
  //No connections, start advertising
  else if ( !Bluefruit.Advertising.isRunning() && !Bluefruit.connected() && !Bluefruit.Central.connected() ) {
    Bluefruit.Scanner.stop();
    Bluefruit.Advertising.start(0);
    Serial.println("Advertising");
  }

  //Query for id if connected to central and no id and query hasn't been sent
  if (Bluefruit.connected() && id == -1 && !query_sent && millis()-conn_ref_time > 5000) {
    uint8_t query[64];
    memset(query, 0, 63);

    query[0] = 'Q';                 //Query type
    query[1] = own_addr.addr[3];    //Address byte 1
    query[2] = own_addr.addr[4];    //Address byte 2
    query[3] = own_addr.addr[5];    //Address byte 3
    query[4] = '!';                 //End message
    bleUart.write( query, 5 );
    query_sent = true;
    quer_ref_time = millis();
    if (DEBUG) {Serial.println("Query for id sent");}
  }
  //If no answer to query in 5 seconds, send a new one
  if (query_sent && millis()-quer_ref_time > 5000) {
    query_sent = false;
  }
  
  if ( Bluefruit.connected() ) {
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
}

void checkUpdateNeed() {
  if ( first_update || (id > -1 && Bluefruit.connected() && millis()-update_ref_time > 10000) ) {
    uint8_t update[64];
    memset(update, 0, 63);

    //If updating for the first time, send constants first
    if (first_update) {
      update[0] = 'C';                        //Constant type
      update[1] = id;                         //id of the lamp
      update[2] = minLevel;                   //min light level of the lamp
      update[3] = maxLevel;                   //max light level of the lamp
      update[4] = colorTempCap;               //can the color temp of the lamp be changed (1 or 0)
      update[5] = (uint8_t)(coolest/100);     //coolest light temp of the lamp divided by 100
      update[6] = (uint8_t)(warmest/100);     //warmest light temp of the lamp divided by 100
      update[7] = '!';                        //End message

      bleUart.write( update, 8 );

      if (DEBUG) Serial.println("Sent constants");
    }
    memset(update, 0, 63);

    if (sensors_connected) updateSensorTemps();
    
    update[0] = 'U';                  //Update type
    update[1] = id;                   //id of the lamp
    transmit(B00000001, B10010000);
    delayMicroseconds(100);
    update[2] = receive();            //Status byte
    transmit(B00000001, B10100000);
    delayMicroseconds(100);
    update[3] = receive();            //Current level byte
    update[4] = (uint8_t)(temp/100);  //Current color temp byte
    update[5] = sensor0_int;          //Integer part of sensor 0 reading
    update[6] = sensor0_frac;         //Fraction part of sensor 0 reading
    update[7] = sensor1_int;          //Integer part of sensor 1 reading
    update[8] = sensor1_frac;         //Fraction part of sensor 1 reading
    update[9] = '!';                  //End message
    bleUart.write( update, 10 );

    if (DEBUG) Serial.println("Sent update");
    
    first_update = false;
    update_ref_time = millis();
  }
}

void handleDataFromSerial(void)
{
  // Forward data from HW Serial to Central (BLEUART) and Peripheral (CLIENTUART)
  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    
    //Send data to central
    bleUart.write( buf, count );

    if ( Bluefruit.Central.connected() && clientUart.discovered() )
    {
      // Discovered means peripheral connection is in working state
      // Send data to peripheral
      clientUart.write(buf, count);
    }
  }
}

void handleDataFromCentral(void)
{
  // Forward from Central (BLEUART) to HW Serial and possible Peripheral (CLIENTUART)

  uint8_t buf2[64];
  uint16_t idx = 0;
  uint8_t ch;
  memset(buf2, 0, 63);
  
  while ( bleUart.available() )
  {
    ch = (uint8_t) bleUart.read();
    if (ch == '!') {
      buf2[idx] = ch;
      idx++;
      break;
    }
    else {
      buf2[idx] = ch;
      idx++;
    }
  }
  //RESPONSE: Message is a response to a query from a lamp in the chain
  if (buf2[0] == 'R') {
    
    //If receiver adress bytes match, process the message
    if (id == -1 &&
        buf2[1] == own_addr.addr[3] && 
        buf2[2] == own_addr.addr[4] && 
        buf2[3] == own_addr.addr[5]) {
      
      id = buf2[4]; //Memorize the assigned id
      update_ref_time = millis();
      first_update = true;
      
      if (DEBUG) {
        Serial.print("Received ID: ");
        Serial.println(id);
      }
    }
    //Otherwise this is for someone else, forward the message
    else {
      if ( Bluefruit.Central.connected() && clientUart.discovered() )
      {
      // Peripheral connection is in working state
      // Forward data to Peripheral
      clientUart.write(buf2, idx);
      }
    }
  }
  //DALI COMMAND: Transmit the DALI command to lamp if the id matches, otherwise forward it
  else if (buf2[0] == 'D') {
    if (buf2[1] == id) {
      if (DEBUG) {Serial.println("Received DALI Command");}
      transmit(buf2[2], buf2[3]);
      //Check if we should expect a reply from the lamp
      if (bitRead(buf2[2],0) == 1) {
        delayMicroseconds(100);
        receive();
        
        buf2[0] = 'A';
        buf2[1] = ret;
        buf2[2] = '!';
        if ( Bluefruit.connected() ) bleUart.write( buf2, 3 );
      }
    }
    else {
      if ( Bluefruit.Central.connected() && clientUart.discovered() )
      {
      // Peripheral connection is in working state
      // Forward data to Peripheral
      clientUart.write(buf2, idx);
      }
    }
  }
  //BROADCAST: Transmit the DALI command to lamp and forward it to the peripheral
  else if (buf2[0] == 'B') {

    if ( Bluefruit.Central.connected() && clientUart.discovered() )
    {
    // Peripheral connection is in working state
    // Forward data to Peripheral
    clientUart.write(buf2, idx);
    }
    transmit(B10000000, buf2[1]);
    
    if ( Bluefruit.connected() ) bleUart.write( buf2, 3 );
  }
  //TEMPERATURE: Transmit the temperature to lamp if it can change color temp and
  //             the id matches or it's 255, otherwise forward it
  else if (buf2[0] == 'T') {
    if ( colorTempCap == 1 && (buf2[1] == id || buf2[1] == 255) ) {
      temp = buf2[2]*100;
      tempChange( temp );
      if (DEBUG) {
        Serial.print("Changed temp to ");
        Serial.println(temp);
        Serial.print("Actual temp is (roughly) ");
        delay(100);
        Serial.println(queryColourTemp());
        update_ref_time = 0;
      }
    }
    
    if (buf2[1] != id) {
      if ( Bluefruit.Central.connected() && clientUart.discovered() ) {
        // Peripheral connection is in working state
        // Forward data to Peripheral
        clientUart.write(buf2, 4);
      }
    }
  }

  //Forward data to HW Serial
  /*if (DEBUG) {
    Serial.write(buf2, idx);
  }*/
}

void handleDataFromPeripheral(BLEClientUart& cent_uart)
{
  uint8_t buf3[64];
  uint16_t idx = 0;
  uint8_t ch;
  memset(buf3, 0, 63);

  //Read data from Peripheral
  while ( clientUart.available() )
  {
    ch = (uint8_t) clientUart.read();
    if (ch == '!') {
      buf3[idx] = ch;
      idx++;
      break;
    }
    else {
      buf3[idx] = ch;
      idx++;
    }
  }

  //Forward data to Central
  bleUart.write( buf3, idx );
}

void scan_callback_c(ble_gap_evt_adv_report_t* report)
{
  uint8_t len = 0;
  uint8_t const* data = extractScanData(report->data, report->dlen, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &len);
  bool is_lamp = false;

  //Check if the scan report is from a BLE lamp
  while( len ) {
    // found matched
    if ( !memcmp(data, (const uint8_t*)name_of_lamp, sizeof(name_of_lamp)) ) {
      is_lamp = true;
      break;
    }
    else {
      data += sizeof(name_of_lamp);
      len  -= sizeof(name_of_lamp);
    }
  }
  // Check if the lamp advertising contains BleUart service
  if ( is_lamp )
  {
    if (DEBUG) {
      Serial.println("BLE Lamp detected");
      Serial.println("Attempt to connect to peripheral ... ");
    }

    // Connect to device with bleUart service in advertising
    // Use Min & Max Connection Interval default value
    Bluefruit.Central.connect(report);
  }
}

void connect_callback_c(uint16_t conn_handle)
{
  if (DEBUG) {Serial.println("Connected to peripheral");}

  if (DEBUG) {Serial.print("Discovering BLE Uart Service ... ");}

  if ( clientUart.discover(conn_handle) )
  {
    if (DEBUG) {
      Serial.println("Found it");
      Serial.println("Enable TXD's notify");
    }
    clientUart.enableTXD();
    c_conn_handle = conn_handle;
    if (DEBUG) Serial.println("Ready to receive from peripheral");
  }
  else
  {
    if (DEBUG) {Serial.println("Found NONE, disconnecting");}
    Bluefruit.Central.disconnect(conn_handle);
  }
}

void disconnect_callback_c(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  if (DEBUG) {
  Serial.println("Disconnected from peripheral");
  }
}

void connect_callback(uint16_t conn_handle)
{
  if (DEBUG) {Serial.println("Connected to central");}
  conn_ref_time = millis();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) reason;
  id = -1;
  quer_ref_time = millis();
  query_sent = false;
  if (DEBUG) {
  Serial.println();
  Serial.println("Disconnected from central");
  Serial.println("Bluefruit will auto start advertising (default)");
  }
}

uint8_t* extractScanData(uint8_t const* scandata, uint8_t scanlen, uint8_t type, uint8_t* result_len)
{
  *result_len = 0;

  // len (1+data), type, data
  while ( scanlen )
  {
    if ( scandata[1] == type )
    {
      *result_len = scandata[0]-1;
      return (uint8_t*) (scandata + 2);
    }
    else
    {
      scanlen  -= (scandata[0] + 1);
      scandata += (scandata[0] + 1);
    }
  }

  return NULL;
}

void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here

  // Request CPU to enter low-power mode until an event/interrupt occurs
  //waitForEvent();
}
