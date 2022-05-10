/* The example is for CubeCell_GPS,
 * GPS works only before lorawan uplink, the board current is about 45uA when in lowpower mode.
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include "GPS_Air530Z.h"
//#include "cubecell_SSD1306Wire.h"
#include "HT_SSD1306Wire.h"

//if GPS module is Air530, use this
//Air530Class Air530;

//if GPS module is Air530Z, use this
Air530ZClass Air530;

extern SSD1306Wire  display;

//How long to wait for GPS Fix if no fix in 2 minutes send update
#define GPS_UPDATE_TIMEOUT 120000

//Wait 5 Seconds after FIX for GPS to stabalise
#define GPS_CONTINUE_TIME 10000
#define MOVING_UPDATE_RATE 5000 //in addition to GPS_CONTINUE_TIME
#define STOPPED_UPDATE_RATE 60000 //In addition to GPS_CONTINUE_TIME
#define SLEEPING_UPDATE_RATE 21600000 //Update every 6hrs when sleeping
bool sleepMode = false;
int speedcount;
float speedtot;
/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

/* OTAA para*/

//uint8_t devEui[] = { 0x1C, 0x96, 0x17, 0xFD, 0x82, 0x11, 0xC8, 0x64 };
//uint8_t devEui[] = { 0x47, 0xAE, 0x91, 0x73, 0x0E, 0x10, 0xBC, 0x5B };
//uint8_t devEui[] = { 0xC5, 0x13, 0x63, 0x1A, 0xD1, 0xDE, 0x6A, 0xF0 };
uint8_t devEui[] = { 0x84, 0x91, 0x8A, 0x11, 0xA3, 0x1A, 0x85, 0x08 };
uint8_t appEui[] = { 0xD7, 0x52, 0xA4, 0x68, 0x97, 0xE2, 0xA2, 0x34 };
uint8_t appKey[] = { 0x93, 0xE0, 0x7D, 0x4A, 0x93, 0x80, 0x8E, 0xD8, 0xCA, 0xD7, 0x05, 0x8A, 0x60, 0xEA, 0xB6, 0x26 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t devAddr =  ( uint32_t )0x00000000;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:
  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)
  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

const uint8_t helium_logo_bmp[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0xff, 0x0f, 0x00, 0x78, 0xc0, 0x07, 0xf0, 0xf1, 0xff, 0x87, 0x07, 0x78, 0xf0, 0xff, 0xff, 
0xff, 0xff, 0x3f, 0x00, 0xf8, 0xc0, 0x07, 0xf0, 0xf0, 0xff, 0x87, 0x0f, 0x78, 0xf0, 0xff, 0xff, 
0xff, 0xff, 0x7f, 0x00, 0xfc, 0x80, 0x07, 0xf8, 0xf0, 0xff, 0x87, 0x1f, 0x78, 0xf0, 0xff, 0xf7, 
0xff, 0xff, 0xff, 0x00, 0xfc, 0x81, 0x0f, 0x78, 0xf0, 0xff, 0x87, 0x3f, 0x78, 0x70, 0xff, 0xf7, 
0xff, 0x1f, 0xf8, 0x00, 0xfe, 0x81, 0x0f, 0x7c, 0xf0, 0x01, 0x80, 0x7f, 0x78, 0x70, 0x1f, 0x86, 
0xf1, 0x1f, 0xf0, 0x00, 0xfe, 0x03, 0x1f, 0x7c, 0xf0, 0x01, 0x80, 0x7f, 0x78, 0x70, 0xff, 0xb6, 
0xfc, 0x1f, 0xf0, 0x00, 0xdf, 0x03, 0x1f, 0x3c, 0xf0, 0x01, 0x80, 0xff, 0x78, 0x70, 0x1f, 0x36, 
0xf1, 0xff, 0xff, 0x00, 0xcf, 0x03, 0x3e, 0x3e, 0xf0, 0x7f, 0x80, 0xff, 0x79, 0x70, 0xdf, 0x36, 
0xe7, 0xff, 0x7f, 0x80, 0x8f, 0x07, 0x3e, 0x1e, 0xf0, 0x7f, 0x80, 0xff, 0x7b, 0x70, 0x10, 0x86, 
0xe0, 0xff, 0x7f, 0x80, 0x87, 0x07, 0x3c, 0x1f, 0xf0, 0x7f, 0x80, 0xef, 0x7f, 0xf0, 0xff, 0xff, 
0xff, 0xff, 0x1f, 0xc0, 0x87, 0x0f, 0x7c, 0x0f, 0xf0, 0x01, 0x80, 0xcf, 0x7f, 0xf0, 0xff, 0xff, 
0xff, 0x1f, 0x3e, 0xc0, 0xff, 0x0f, 0xf8, 0x0f, 0xf0, 0x01, 0x80, 0x8f, 0x7f, 0xf0, 0xff, 0xff, 
0xff, 0x1f, 0x3c, 0xc0, 0xff, 0x1f, 0xf8, 0x07, 0xf0, 0x01, 0x80, 0x0f, 0x7f, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0x7c, 0xe0, 0xff, 0x1f, 0xf0, 0x07, 0xf0, 0xff, 0x87, 0x0f, 0x7f, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0xf8, 0xe0, 0x03, 0x3e, 0xf0, 0x03, 0xf0, 0xff, 0x87, 0x0f, 0x7e, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0xf8, 0xf0, 0x01, 0x3e, 0xe0, 0x03, 0xf0, 0xff, 0x87, 0x0f, 0x7c, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0xf0, 0xf1, 0x00, 0x7c, 0xe0, 0x01, 0xf0, 0xff, 0x87, 0x0f, 0x78, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, // 'flipped2'
};


int32_t fracPart(double val, int n)
{
  return (int32_t)((val - (int32_t)(val))*pow(10,n));
}

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void displayStatus(String status)
{
  VextON();// oled power on;
  delay(10); 
  display.init();
  display.clear();
  display.drawXbm(0, 0, 128, 42, helium_logo_bmp);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  
  //get Battery Level 1-254 Returned by BoardGetBatteryLevel
  float batteryLevel = (BoardGetBatteryLevel());
  //Convert to %
  batteryLevel = (batteryLevel/254)*100;
  String str = "BATTERY ";
  str.concat(batteryLevel);
  str.concat("%");
  uint16_t batteryVoltage = getBatteryVoltage();
  str.concat(" ");
  str.concat(batteryVoltage);
  str.concat(" mV");
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 50-16/2, status);
  display.drawString(64, 60-16/2, str);
  display.display();  
}

void displayGPSInof()
{
  char str[30];
  display.clear();
  display.setFont(ArialMT_Plain_10);
  int index = sprintf(str,"%02d-%02d-%02d",Air530.date.year(),Air530.date.day(),Air530.date.month());
  str[index] = 0;
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, str);
  
  index = sprintf(str,"%02d:%02d:%02d",Air530.time.hour(),Air530.time.minute(),Air530.time.second(),Air530.time.centisecond());
  str[index] = 0;
  display.drawString(60, 0, str);

  if( Air530.location.age() < 1000 )
  {
    display.drawString(120, 0, "A");
  }
  else
  {
    display.drawString(120, 0, "V");
  }


  if( (speedtot/speedcount) > 1.2 )
  {
    display.drawString(107, 0, "M");
  }
  else
  {
    display.drawString(107, 0, "S");
  }
  
  index = sprintf(str,"alt: %d.%d",(int)Air530.altitude.meters(),fracPart(Air530.altitude.meters(),2));
  str[index] = 0;
  display.drawString(0, 16, str);
   
  index = sprintf(str,"hdop: %d.%d",(int)Air530.hdop.hdop(),fracPart(Air530.hdop.hdop(),2));
  str[index] = 0;
  display.drawString(0, 32, str); 
 
  index = sprintf(str,"lat :  %d.%d",(int)Air530.location.lat(),fracPart(Air530.location.lat(),4));
  str[index] = 0;
  display.drawString(60, 16, str);   
  
  index = sprintf(str,"lon: %d.%d",(int)Air530.location.lng(),fracPart(Air530.location.lng(),4));
  str[index] = 0;
  display.drawString(60, 32, str);

  index = sprintf(str,"speed: %d.%d km/h",(int)Air530.speed.kmph(),fracPart(Air530.speed.kmph(),2));
  str[index] = 0;
  display.drawString(0, 48, str);
  display.display();

  index = sprintf(str,"sats: %d",(int)Air530.satellites.value());
  str[index] = 0;
  display.drawString(88, 48, str);
  display.display();
}

void printGPSInof()
{
  Serial.print("Date/Time: ");
  if (Air530.date.isValid())
  {
    Serial.printf("%d/%02d/%02d",Air530.date.year(),Air530.date.day(),Air530.date.month());
  }
  else
  {
    Serial.print("INVALID");
  }

  if (Air530.time.isValid())
  {
    Serial.printf(" %02d:%02d:%02d.%02d",Air530.time.hour(),Air530.time.minute(),Air530.time.second(),Air530.time.centisecond());
  }
  else
  {
    Serial.print(" INVALID");
  }
  Serial.println();
  
  Serial.print("LAT: ");
  Serial.print(Air530.location.lat(),6);
  Serial.print(", LON: ");
  Serial.print(Air530.location.lng(),6);
  Serial.print(", ALT: ");
  Serial.print(Air530.altitude.meters());

  Serial.println(); 
  
  Serial.print("SATS: ");
  Serial.print(Air530.satellites.value());
  Serial.print(", HDOP: ");
  Serial.print(Air530.hdop.hdop());
  Serial.print(", AGE: ");
  Serial.print(Air530.location.age());
  Serial.print(", COURSE: ");
  Serial.print(Air530.course.deg());
  Serial.print(", SPEED: ");
  Serial.println(Air530.speed.kmph());
  Serial.print("SPEED Count: ");
  Serial.println(speedcount);
  Serial.print("SPEED Total: ");
  Serial.println(speedtot);
  Serial.print("SPEED Average: ");
  Serial.println(speedtot/speedcount);



  
  Serial.println();
}

static void prepareTxFrame( uint8_t port )
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  uint32_t lat, lon;
  int  alt, course, speed, hdop, sats;
  float batteryLevel;

  
  Serial.println();
  Serial.println("Waiting for GPS FIX ...");

  displayStatus("GPS Searching");
  Serial.println("GPS Searching...");

  Air530.begin(); 
  
  uint32_t start = millis();
  while( (millis()-start) < GPS_UPDATE_TIMEOUT )
  {
    while (Air530.available() > 0)
    {
      Air530.encode(Air530.read());
    }
   // gps fixed in a second
    if( Air530.location.age() < 1000 )
    {
      break;
    }
  }
  
  if(Air530.location.age() < 1000)
  {
    start = millis();
    uint32_t printinfo = 0;
    speedcount = 0;
    speedtot = 0;
    while( (millis()-start) < GPS_CONTINUE_TIME )
    {
      while (Air530.available() > 0)
      {
        Air530.encode(Air530.read());
      }

      if( (millis()-start) > printinfo )
      {
        speedcount +=1;
        speedtot += Air530.speed.kmph();
        printinfo += 1000;
        printGPSInof();
        displayGPSInof();
      }
    }
  }
  else
  {
    display.clear();  
    delay(10); 
    display.drawXbm(0, 0, 128, 42, helium_logo_bmp);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 54-16/2, "No GPS signal");
    display.display();
    delay(2000);
  }
    
    Air530.end(); 
    display.clear();
    display.display();
    display.stop();
    VextOFF(); //oled power off
  
  lat = (uint32_t)(Air530.location.lat()*1E7);
  lon = (uint32_t)(Air530.location.lng()*1E7);
  alt = (uint16_t)Air530.altitude.meters();
  course = Air530.course.deg();
  if (speedcount > 0)
  {
    speed = (speedtot/speedcount);
  }
  else speed = 0;
  sats = Air530.satellites.value();
  hdop = Air530.hdop.hdop();

  uint16_t batteryVoltage = getBatteryVoltage();
  
  //get Battery Level 1-254 Returned by BoardGetBatteryLevel
  batteryLevel = (BoardGetBatteryLevel());
  //Convert to %
  batteryLevel = (batteryLevel/254)*100;
  
  if ((lat==0)&&(lon==0))
  {
  Serial.println("No GPS Lock No Uplink were on NULL Island");
  appDataSize = 0; 
  }
  else
  {
    //Build Payload
  
  unsigned char *puc;
  appDataSize = 0;

  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[3];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[3];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];
  
  /*puc = (unsigned char *)(&alt);
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];
  */
  
  puc = (unsigned char *)(&speed);
  appData[appDataSize++] = puc[0];
  
  appData[appDataSize++] = (uint8_t)((hdop*10) & 0xFF);

  appData[appDataSize++] = (uint8_t)((batteryVoltage/20) & 0xFF);
  
  Serial.print("Speed ");
  Serial.print(speed);
  Serial.println(" kph");


  Serial.print("Battery Level ");
  Serial.print(batteryLevel);
  Serial.println(" %");
  
  Serial.print("BatteryVoltage:");
  Serial.println(batteryVoltage);

  Serial.print("SleepMode = ");
  Serial.println(sleepMode);
  Serial.println();
  }


  
  
}


void setup() {
  boardInitMcu();
  Serial.begin(115200);
  Air530.setmode(MODE_GPS_GLONASS);      // was added to enable GLONASS and GPS GNSS networks 
  
  //setup user button
  pinMode(P3_3,INPUT);
  attachInterrupt(P3_3,userKey,FALLING); 
  
#if(AT_SUPPORT)
  enableAt();
#endif
  LoRaWAN.displayMcuInit();    
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
  LoRaWAN.setDataRateForNoADR(0);
}

void userKey(void)
{
  delay(10);
  if(digitalRead(P3_3)==0)
  {
    uint16_t keyDownTime = 0;
    while(digitalRead(P3_3)==0)
    {
      delay(1);
      keyDownTime++;
      if(keyDownTime>=700)
      break;
    }
    if(keyDownTime<700)
    {
      if (sleepMode)
      {
        sleepMode = false;
        LoRaWAN.cycle(2000);
        displayStatus("Waking...");
        delay(4000);
                
      }
      else
      {
        sleepMode = true;
        Serial.print("SleepMode = ");
        Serial.println(sleepMode);
        LoRaWAN.cycle(2000);
        displayStatus("Sleeping...");
        delay(4000);
      }
    }
  }
}

void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass,loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      //LoRaWAN.displayJoining();
      displayStatus("JOINING...");
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      prepareTxFrame( appPort );
      Serial.print("appDataSize = ");
      Serial.println(appDataSize);
      if  (appDataSize > 0)
      {
        displayStatus("SENDING...");
        LoRaWAN.send();
        delay(1000);
        display.stop();
        VextOFF();// oled power off;
        deviceState = DEVICE_STATE_CYCLE;
      }
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      if (sleepMode) appTxDutyCycle = SLEEPING_UPDATE_RATE;
      else
      {
        if ( (speedtot/speedcount) > 1.2) 
        {
        appTxDutyCycle = MOVING_UPDATE_RATE;
        Serial.println();
        Serial.print("Speed = ");
        Serial.print((speedtot/speedcount));
        Serial.println(" MOVING");
        }
  
      else 
        {
        appTxDutyCycle = STOPPED_UPDATE_RATE;
        Serial.println();
        Serial.print("Speed = ");
        Serial.print((speedtot/speedcount));
        Serial.println(" STOPPED");
        }
      }
  
  txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep();   
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}