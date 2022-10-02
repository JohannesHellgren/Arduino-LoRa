
#define NODE_ID 0x0002
#define DESTINATION_ID 0x1A01
#define MESSAGE_ID  0x01      // Temp/Humid

#if  MESSAGE_ID == 0x01 
    #define DATA_LENGTH  2
#endif

// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

/*
   SDA AM2305 Pin11 1-Wire   temp/fukt-sensorn  DS18b20-temp
   RED_LED    pin 13
   LORA CS    pin8
   LoRa RES\  pin4
   LoRa IRQ   pin3
 */

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <LoRa.h>  // by Sendeep Mistry 0.8.0

#include <RTClib.h> // Date and time functions using a PCF8523 RTC connected via I2C and Wire lib

#define DHTPIN 11     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

#define u32 uint32_t

//  -------- Logger ----------
const int SDchipSelect = 10;  // SD-card  https://learn.adafruit.com/adafruit-adalogger-featherwing/pinouts

// --------- RTC -----------
RTC_PCF8523 rtc;

uint32_t timer = millis();

uint32_t RTCtime_in_seconds;
uint32_t GPStime_in_seconds;
int32_t timeDiff;

//  -------- LoRa ----------
const int csPin = 8;             // LoRa radio chip select on M0 Feather
const int resetPin = 4;          // LoRa radio reset on M0 Feather
const int irqPin = 3;            // Hardware interrupt pin for LoRa on M0 Feathe

const long LoRa_freq = 868.2E6;
const long LoRa_bw = 125E3;      // Bandwith  125 - 500 kHz
const int LoRa_SF = 9;           // Spreading Factor 6-12  ( higher gives longer range and longer transmition time )
const int LoRa_SyncWord = 0x34;  // ranges from 0-0xFF, default 0x34, see API docs
const int RFM_TX_POWER = 13;     // 5..23 dBm, 13 dBm is default
uint8_t radiopacket[12];
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

#define VBATPIN A7  // Measure the battery voltage
#define RED_LED 13

uint8_t battery;

int32_t msgCount = 1;

void setup() 
{
      Serial.begin(115200);
      delay(3000);
      delay(delayMS);
      // Initialize device.
      LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin
      delay(10);
      if (!LoRa.begin(LoRa_freq)) 
      {
          Serial.println("Starting LoRa failed!");
          while (1);
      }
      delay(1000);
      //LoRa.setSpreadingFactor(LoRa_SF);           // ranges from 6-12,default 7 see API docs
      //LoRa.setSyncWord(LoRa_SyncWord);           // ranges from 0-0xFF, default 0x34, see API docs
      // As for SyncWord values, 0x12 is the default after chip reset, and 0x34 is used for LoRaWAN ( kolla upp detta !!!!!) 
      //LoRa.setTxPower(17, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
      //LoRa.setTxPower(RFM_TX_POWER);
      //LoRa.setSignalBandwidth(125E3);  // RFM95 handles BW 125kHz to 500kHz
      //enableCrc(); 

      Serial.println("LoRa Transmitter      SW V1.0");
      Serial.print("F=");Serial.print(LoRa_freq/1E6); Serial.print("MHz  BW=");Serial.print((int)(LoRa_bw/1E3)); Serial.print("kHz  SF="); Serial.print(LoRa_SF);
      Serial.print("  P=");Serial.print(RFM_TX_POWER);
      Serial.print("dBm  SynkW=0x");Serial.println(String(LoRa_SyncWord,HEX));
      
      battery=GetBatteryVoltage();
//    display.println("VBat: " + String(((float)battery+200)/100) + "V");
      Serial.println("VBat: " + String(((float)battery+200)/100) + "V");
      //Transmission interval=" + String(packetInterval) + "sec.");
      Serial.println("Sender ID=0x0001    Receiver ID=0x0002");

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






      dht.begin();
      
      if (MESSAGE_ID == 0x01 )
      {
          Serial.println(F("Temp/Humid sensor"));
      }
  
      sensor_t sensor;    
      dht.temperature().getSensor(&sensor);
      dht.humidity().getSensor(&sensor);
      // Set delay between sensor readings based on sensor details.
      delayMS = sensor.min_delay / 1000;
}

void loop() 
{
      delay(delayMS);   // Delay between measurements.
      
      sensors_event_t event;  // Get temperature event and print its value.
      dht.temperature().getEvent(&event);
      if (isnan(event.temperature)) {
            Serial.println(F("Error reading temperature!"));  //  F() macro saves SRAM by moving string from SRAM to FLASH
      }
      else 
      {
            Serial.print(F("T: "));
            Serial.print(event.temperature, 1); // print with 1 decimals
            //Serial.print(("°C"));
            Serial.print(F(" C   "));
            radiopacket[7] = (int8_t)(event.temperature*2);   // x2 ger bättre upplösning 0.5deg
      }
      // Get humidity event and print its value.
      dht.humidity().getEvent(&event);
      if (isnan(event.relative_humidity)) {
            Serial.println(F("Error reading humidity!"));
      }
      else
      {
            Serial.print(F("H: "));
            Serial.print(event.relative_humidity, 1); // print with 1 decimals
            Serial.print(F(" %RH"));
            radiopacket[8] = (int8_t)(event.relative_humidity);
      }

      battery=GetBatteryVoltage();
      radiopacket[5] = battery;
    
      // send packet
      sendLoRaMessage();
      Serial.println("    VBat: " + String(((float)battery+200)/100) + "V");
}

void sendLoRaMessage() 
{
      LoRa.beginPacket();                   // start packet
      LoRa.write(radiopacket,7+DATA_LENGTH);
      LoRa.endPacket();                     // finish packet and send it          
}

/*
 * Returnerar batteriets spänning som en byte.
 * 
 */
uint8_t GetBatteryVoltage(void) 
{
      float measuredvbat = analogRead(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
      return (measuredvbat * 100 - 200);   // ex 3.72V -> 372 -> 172
 }
