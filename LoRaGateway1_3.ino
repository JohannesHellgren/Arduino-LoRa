/* The ESP32 has four SPi buses, however as of right now only two of
 * them are available to use, HSPI and VSPI. Simply using the SPI API 
 * as illustrated in Arduino examples will use VSPI, leaving HSPI unused.
 * 
 * However if we simply intialise two instance of the SPI class for both
 * of these buses both can be used. However when just using these the Arduino
 * way only will actually be outputting at a time.
 * 
 * created 30/04/2018 by Alistair Symonds
 */
 
 /*
  1.8" 128x160 TFT ST7735
  Landscape alignment (SPI pins to the right)
   _______________________
  /                       /
  /                       /
  /                       /   /
  /                       /  128
  /                       /   /
  /_______________________/
            - 160 -


*/
/*
   LORA CS     pin15
   LoRa RES\   pin4
   LoRa IRQ    pin2
   TFT  CS     pin5
   TFT  RES\   pin25
   TFT  A0     pin26
   SD   CS     pin27
   SD   DETECT pin?
   VSPI CLK    pin18
   VSPI MOSI   pin23
   VSPI MISO   pin19
   HSPI CLK    pin14
   HSPI MOSI   pin13
   HSPI MISO   pin12
   Battery     pin36
   Buzzer      pin?

   --------------------------------------
SPI   MOSI    MISO    SCLK    CS
VSPI  GPIO 23 GPIO 19 GPIO 18 GPIO 5    Default SPI
HSPI  GPIO 13 GPIO 12 GPIO 14 GPIO 15
   --------------------------------------
 */

#define NODE_ID 0x1A01 
 
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <LoRa.h>  // by Sendeep Mistry 0.8.0
#include <WiFi.h>
#include "time.h" // https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/

#define u8 uint8_t
#define s8 int8_t
#define u16 uint16_t
#define s16 int16_t
#define u32 uint32_t
#define s32 int32_t
#define BATT_PIN  36

//  -------- TFT Display ----------
#define TFT_CS   5  // Default CS for VSPI
#define TFT_RST  25
#define TFT_DC   26 

//  -------- LoRa ----------
#define LORA_CS  15             // LoRa radio chip select (efault CS for VSPI)
#define LORA_RES 4             // LoRa radio reset on M0 Feather
#define LORA_IRQ 2             // Hardware interrupt pin for LoRa on M0 Feather
#define MAXPACKETSIZE 30        // LoRa packet should never be larger that this
const long LoRa_freq = 868.2E6;
const long LoRa_bw = 125E3;      // Bandwith  125 - 500 kHz
const int  LoRa_SF = 9;           // Spreading Factor 6-12  ( higher gives longer range and longer transmition time )
const int  LoRa_SyncWord = 0x34;  // ranges from 0-0xFF, default 0x34, see API docs
const int  RFM_TX_POWER = 13;     // 5..23 dBm, 13 dBm is default
u16 destination_id;
u16 senders_id;
u8  incomming_message;
u16 incomming_Battery;
u8  radiopacket[MAXPACKETSIZE];
s16 incomming_rssi;

uint32_t prev_time;

/*
 *  13dB    mW                      I=29mA
 *  14dB  25mW  ( max in Europe )  
 *  20dB  100mW                     I=120mA
 */
/*
    https://www.thethingsnetwork.org/docs/lorawan/frequency-plans.html
    Uplink:  
    Channel
      0    868.1 - SF7BW125 to SF12BW125
      1    868.3 - SF7BW125 to SF12BW125 and SF7BW250
      2    868.5 - SF7BW125 to SF12BW125
      3    867.1 - SF7BW125 to SF12BW125
      4    867.3 - SF7BW125 to SF12BW125
      5    867.5 - SF7BW125 to SF12BW125
      6    867.7 - SF7BW125 to SF12BW125
      7    867.9 - SF7BW125 to SF12BW125
      8    868.8 - FSK
   */
   
   /*
    * Paketuppbyggnad
    * 2B  Sender ID MSB
    * 2B  Receiver ID MSB
    * 1B  Batterispänning  ( addera denna byte med 200 samt dividera med 100 ex  175 -> 375 -> 3.75V
    * 2B  PaketID MSB, börjar på 1 och inkrementeras för varje paket. 
    * 4B  Data ( sänder texten AFRY )
    */

char textbuffer[40];

//  ------------------- Logger via SD card ------------------
/*
/TEMPLOG1.CSV.csv
Date,Value1,Value2
2022-06-27 09:30:20,15,20
2022-06-29 21:30:34,3,9
SDmissing will be low when SDcard is iserted and High when missing
*/
//#define SDmissing ?   //   https://www.youtube.com/watch?v=GQjtG1MeVs4&ab_channel=RalphSBacon
#define SD_CS 27  // SD-card  https://learn.adafruit.com/adafruit-adalogger-featherwing/pinouts
File root;

const char* ssid     = "NETGEAR38";
const char* password = "mightywater039";

long timezone = 1;     // 1h offset i förhållande till GMT
byte daysavetime = 1;  // 1h offset för sommartid

u8 battery;

char timeString[30];
struct tm tmstruct ;
uint8_t previous_second;

Adafruit_ST7735 tft  = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


void setup() 
{
    // VSPI ( Default SPI in Arduino)
    // TFT-display
    // SD card
    //SPIClass  SPI1(VSPI);
    //Adafruit_ST7735 tft  = Adafruit_ST7735(&SPI1, TFT_CS, TFT_DC, TFT_RST);

    
    // HSPI ( NOT default SPI in Arduino)
    // LoRa
    SPIClass  SPI2(HSPI);
    LoRa.setSPI(SPI2);  // Change LoRas spi to HSPI
    LoRa.setPins(LORA_CS, LORA_RES, LORA_IRQ);

    //pinMode(SD_CS, OUTPUT);  // initialize digital pin 13 (red led) as an output.
    
    Serial.begin(115200);
    while (!Serial) { ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println(F("\nLoRa Gateway  SW V1.3\n"));
    // Use this initializer if using a 1.8" TFT screen:
    
    tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
    tft.setRotation(1);             // Landscape alignment (SPI pins on rigth )
    tft.setTextSize(1);             // Smallest textsize
    
    tft.fillScreen(ST77XX_BLACK);   // Erase screen
    //tft.setTextColor(ST77XX_WHITE); // Start with white text
    //tft.setCursor(0, 40); // X,Y
    //tft.println(F("  LoRa Gateway  SW V1.3")); 
    tftPrint("  LoRa Gateway  SW V1.3\n",ST77XX_WHITE,0,40);
    
    Serial.println(F("Gateway ID=0x1A01"));
    //tft.setTextColor(ST77XX_WHITE);
    //tft.println(F("\n  Gateway ID=0x1A01"));
    tftPrint("\n  Gateway ID=0x1A01\n",ST77XX_WHITE,-1,0);
    
    // OBS ADC input impedance is around 200kohm
    float battery = (float)analogRead(BATT_PIN)/4096*3.3;
    //battery=GetBatteryVoltage();
    //display.println("VBat: " + String(((float)battery+200)/100) + "V");
    Serial.print(F("Batt="));
    Serial.print(battery,2);
    Serial.println("V");
    tft.print(F("\n  Battery: "));
    tft.print(battery,2);
    tft.println("V");
    delay(6000);  
   
    tft.fillScreen(ST77XX_BLACK);   // Erase screen
     
    //  ------------------- LoRa ----VSPI------------------
    LoRa.setPins(LORA_CS, LORA_RES, LORA_IRQ); // set CS, reset, IRQ pin
    delay(10);
    Serial.println(F("------------------------------"));
    Serial.println(F("LoRa Initiating"));
    if (!LoRa.begin(LoRa_freq)) 
    {
        tft.setCursor(0,0);
        tft.setTextColor(ST77XX_RED);
        tft.println("LoRa failed!");
        Serial.println("   LoRa init failed!");
        while (0);
        delay(2000);
    }
    delay(1000);
    //LoRa.setSpreadingFactor(LoRa_SF);           // ranges from 6-12,default 7 see API docs
    //LoRa.setSyncWord(LoRa_SyncWord);           // ranges from 0-0xFF, default 0x34, see API docs
    //LoRa.setTxPower(17, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
    //LoRa.setTxPower(RFM_TX_POWER);
    //Serial.println("LoRa Dump Registers");
    //LoRa.dumpRegisters(Serial);

// ------------- Connect to WiFi network  -----------------
    Serial.println(F("------------------------------"));
    Serial.print("Connecting to ");
    Serial.println(ssid);
    Serial.print("   ");
    tft.setTextColor(ST77XX_BLUE);
    tft.print("Connecting to ");
    tft.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        tft.print(".");
    }
    Serial.println(" WiFi connected");
    Serial.print("   IP address: ");
    Serial.print(WiFi.localIP());
    Serial.print(" RSSI:");
    Serial.println(WiFi.RSSI());
    
    Serial.print("Contacting Time Server");
    tft.setTextColor(ST77XX_GREEN);
    tft.println(" WiFi connected");
    tft.print("IP address: ");
    tft.println(WiFi.localIP());
    tft.print("RSSI:");
    tft.println(WiFi.RSSI());
// ------------- NPT server -----------------
    tft.setTextColor(ST77XX_BLUE);
    tft.print("Contacting Time Server");

    /*
        Note: Bundled Arduino lwip supports only ONE ntp server, 2nd and 3rd options are silently ignored
        see CONFIG_LWIP_DHCP_MAX_NTP_SERVERS define in ./tools/sdk/esp32/sdkconfig
     */
    configTime(3600*timezone, daysavetime*3600, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
    
    uint16_t time_year;
    while (!getLocalTime(&tmstruct,1000))
    {
        Serial.print("."); //Serial.println("Could not obtain time info");
        tft.print(".");
        //time_year = (tmstruct.tm_year)+1900;
        //Serial.printf("%d ",time_year);      
    } //while(time_year < 2022);
    //disconnect WiFi as it's no longer needed
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    getESP32time();
    Serial.print("\n   ");
    Serial.println(timeString);
    tft.println();
    tft.setTextColor(ST77XX_GREEN);
    tft.println(timeString);
    
 //   getLocalTime(&tmstruct);
 //   sprintf(timeString,"%d-%02d-%02d %02d:%02d:%02d\0",(tmstruct.tm_year)+1900,( tmstruct.tm_mon)+1, tmstruct.tm_mday,tmstruct.tm_hour , tmstruct.tm_min, tmstruct.tm_sec);
 //   Serial.println(timeString);
    Serial.println(F("------------------------------"));

   getESP32time();
   previous_second = tmstruct.tm_sec;
   
    //  PinMode(SDmissing, INPUT_PULLUP);
    initSDCard(); // Connect to SD card
    Serial.println("   Log file: TEMPLOG0.CSV");
    Serial.println(F("------------------------------")); 
    tft.setTextColor(ST77XX_GREEN);
    tft.println("TEMPLOG0.CSV"); 
    delay(5000);
    tft.fillScreen(ST77XX_BLACK);   // Erase screen
    prev_time = millis();
}



void loop() 
{
    // parse for a LoRa packet, and call onReceive with the result:
    u8 status = 1;//onReceive(LoRa.parsePacket());
    if ((millis() - prev_time) > 4000)
    {
        //Serial.println(millis() - prev_time);
        if (status)
            parsePacket();
         prev_time = millis();   
    }
    
      
    getESP32time();   // Tar 0mS
    
    if (previous_second != tmstruct.tm_sec)
    {
      u32 prev2_time = millis();
          // Nedan tar 418mS
          previous_second = tmstruct.tm_sec;
          tft.fillScreen(ST77XX_BLACK);   // Erase screen
          tft.setTextColor(ST77XX_WHITE);
          tft.fillRect(40,0,128,10,ST77XX_BLACK); // Erase
          tft.setCursor(40,0);
          tft.print(timeString);

          // open the file. note that only one file can be open at a time,
          File dataFile = SD.open("/TEMPLOG0.CSV", FILE_APPEND); //FILE_APPEND
          
          // Check to see if the file exists: 
          if (dataFile)
          {
              //sprintf(dataString,"%02X%02X %02X%02X %02X %02X%02X ",radiopacket[0],radiopacket[1],radiopacket[2],radiopacket[3],radiopacket[4],radiopacket[
              sprintf(textbuffer,"%s,%d,%d",timeString,1,234);
              dataFile.println(textbuffer);
              // Force data to SD and update the directory entry to avoid data loss.
              //if (!dataFile.sync() || dataFile.getWriteError()) {
                //error("write error");
              //  Serial.println("write error");
              //}
              //dataFile.flush();
              dataFile.close();
              Serial.print(textbuffer);
              Serial.println("  to TEMPLOG0.CSV");
              tft.setTextColor(ST77XX_YELLOW);
              tft.setCursor(0,50);
              tft.println(textbuffer);             
          } 
          else 
          {
              Serial.println("TEMPLOG0.CSV doesn't exist.");
              //initSDCard(); // Connect to SD card
          }
          
          Serial.println(millis() - prev2_time);
          
    }
  
   // If the SD card has been detected / insterted  
    /*
    if () //!digitalRead(SDmissing))
    {
        // open the file. note that only one file can be open at a time,
        File dataFile = SD.open("/TEMPLOG0.CSV", FILE_APPEND); //FILE_APPEND
        
        // Check to see if the file exists: 
        if (dataFile)
        {
            //sprintf(dataString,"%02X%02X %02X%02X %02X %02X%02X ",radiopacket[0],radiopacket[1],radiopacket[2],radiopacket[3],radiopacket[4],radiopacket[
            sprintf(textbuffer,"%s,%d,%d",timeString,1,234);
            dataFile.println(textbuffer);
            // Force data to SD and update the directory entry to avoid data loss.
            //if (!dataFile.sync() || dataFile.getWriteError()) {
              //error("write error");
            //  Serial.println("write error");
            //}
            //dataFile.flush();
            Serial.print(textbuffer);
            Serial.println("  to TEMPLOG0.CSV");
            tft.setTextColor(ST77XX_YELLOW);
            tft.println(textbuffer);
        } 
        else 
        {
            Serial.println("TEMPLOG0.CSV doesn't exist.");
            initSDCard(); // Connect to SD card
        }
        dataFile.close();
        //digitalWrite(SD_CS, HIGH); // deactivate CS
    }
    else 
        initSDCard(); // Connect to SD card
    //Serial.println(FreeRam());
    Serial.println("Sleep 5sek.");  // print to the serial port too:
    delay(5000);
    */

}

/*
 * Returnerar batteriets spänning som en byte.
 * 
 */
uint8_t GetBatteryVoltage(void) 
{
      float measuredvbat = 3.6;//analogRead(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
      measuredvbat = 360;
      return (measuredvbat * 100 - 200);   // ex 3.72V -> 372 -> 172
}

//-----------------------------------
// SD Card initialisation routine
//-----------------------------------

  
void initSDCard()
{
     while(!SD.begin(SD_CS))
     {
          Serial.println(F("------------------------------"));
          Serial.println(F("SD Card not found / responding"));
          Serial.println(F("Insert a formatted SD card    "));
          Serial.println(F("------------------------------")); 
          tft.setTextColor(ST77XX_RED); 
          tft.println(F("SD Card not found / responding"));
          SD.end();
          delay(1000);  
     }

     Serial.println(F("SD Card Initiating"));  
     tft.setTextColor(ST77XX_BLUE); 
     tft.println(F("SD Card Initiating"));

     uint8_t cardType = SD.cardType();

     if(cardType == CARD_NONE){
        Serial.println("   No SD card attached");
        return;
     }
     Serial.print("   SD Card Type: ");
     if(cardType == CARD_MMC){
        Serial.println("MMC");
     } else if(cardType == CARD_SD){
        Serial.println("SDSC");
     } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
     } else {
        Serial.println("UNKNOWN");
     }
     uint64_t cardSize = SD.cardSize() / (1024 * 1024);
     Serial.printf("   SD Card Size: %lluMB\n", cardSize);
     tft.setTextColor(ST77XX_GREEN);
     tft.printf("SD Card Size: %lluMB\n", cardSize);
  //   root = SD.open("/");
  //   printDirectory(root, 0);
  //   Serial.println(F("------------------------------"));
  //   root.close();
     return;
}

void printDirectory(File dir, int numTabs) 
{
    while (true) 
    {
        File entry =  dir.openNextFile();
        if (! entry) {
            // no more files
            break;
        }
        for (uint8_t i = 0; i < numTabs; i++) {
            Serial.print('\t');
        }
        Serial.print(entry.name());
        if (entry.isDirectory()) {
            Serial.println("/");
            printDirectory(entry, numTabs + 1);
        } else {
            // files have sizes, directories do not
            Serial.print("\t\t");
            Serial.println(entry.size(), DEC);
        }
        entry.close();
    }
}

void getESP32time(void)
{
      getLocalTime(&tmstruct, 100);
      sprintf(timeString,"%d-%02d-%02d %02d:%02d:%02d\0",(tmstruct.tm_year)+1900,( tmstruct.tm_mon)+1, tmstruct.tm_mday,tmstruct.tm_hour , tmstruct.tm_min, tmstruct.tm_sec);         
}
/*
      radiopacket[0] = DESTINATION_ID >> 8;
      radiopacket[1] = DESTINATION_ID & 0x00ff;
      radiopacket[2] = NODE_ID >> 8;
      radiopacket[3] = NODE_ID & 0x00ff;
      radiopacket[4] = MESSAGE_ID;
      radiopacket[5] = battery;
      radiopacket[6] = DATA_LENGTH;    // Data length
      radiopacket[7] = 0x01;
      radiopacket[8] = 0x02;
      radiopacket[9] = 0x03;
      radiopacket[10] = 0x04;
 */
u8 onReceive(int packetSize) 
 {
      String message = "";
      float f_batt;
      if (packetSize == 0) return 0;            // if there's no packet, return
      if (packetSize > MAXPACKETSIZE) return 0; // Something is wrong, return

      // read packet header bytes:
  //    int recipient = LoRa.read();          // recipient address
  //    byte sender = LoRa.read();            // sender address
  //    byte incomingMsgId = LoRa.read();     // incoming msg ID
  //    byte incomingLength = LoRa.read();    // incoming msg length

      uint8_t cntr=0; 
      while (LoRa.available()) 
      {
            radiopacket[cntr++] = (char)LoRa.read();
      }
      destination_id = radiopacket[0]*256+radiopacket[1];
      if (destination_id != NODE_ID) return 0;  // Is the incomming packet for this node?
  
      senders_id = radiopacket[2]*256+radiopacket[3];
      incomming_message = radiopacket[4];
      incomming_Battery = 10*(radiopacket[5]+200); // Batteri
      incomming_rssi = LoRa.packetRssi();
 
      return 1;
 }

 void parsePacket(void)
 {
      u8 dataLength;
      float f_batt;
      f_batt = incomming_Battery;
      f_batt = f_batt/1000;
      dataLength = radiopacket[6];
  
      // Fejk!
      destination_id = 0x1A01;
      senders_id = 0x0001;       // My soil sensor
      incomming_message = 0x03;  // A soil sensor
      incomming_Battery = 3670; // [mV]
      dataLength = 5; 
      radiopacket[7] = 25;
      radiopacket[8] = 17;
      radiopacket[9] = 15;
      radiopacket[10] = 13;
      radiopacket[11] = 11;  // -2m 
      
      Serial.print("0x");
      Serial.print(destination_id,HEX);  // Destination ID 
      Serial.print(" 0x");
      Serial.print(senders_id,HEX);  // Senders ID
      Serial.print(" ");
      Serial.print(incomming_message,HEX);  // Message ID
      Serial.print(" ");
      Serial.print(incomming_Battery);  // Batteri i mV
      Serial.print("mV ");
      Serial.print(dataLength,DEC);  // Data length
      Serial.print(" [");
      for (uint8_t i=0;i<dataLength;i++)
      {
            Serial.print(radiopacket[7+i]);
            Serial.print(" ");
      }
      Serial.print("]");     
      Serial.print("' with RSSI " + String(incomming_rssi));
      Serial.println();

      /*
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(F("0x000"));
      display.println(senders_id,HEX);  // Senders ID      
      display.print(String(radiopacket[7]/2) + "C  ");
      display.println(String(radiopacket[8]) + "%RH");
      display.print(f_batt,1);
      display.println("V");
      display.print("rssi:" + String(rssi) );
      display.display();
      IncommingPacketTimer = millis(); // reset the timer
      digitalWrite(RED_LED, LOW);   // turn the LED off
      */
 }

void tftPrint(char *text, uint16_t colour, int x, int y)
{   
    if (x>=0)
        tft.setCursor(x, y);
    tft.setTextColor(colour);
    tft.fillScreen(ST77XX_BLACK);   // Erase screen
    // tft.setTextWrap(true); // Set whether text that is too long for the screen width should automatically wrap around to the next line (else clip right).
    tft.println(text); 
}
  
