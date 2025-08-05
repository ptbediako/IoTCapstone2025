/* 
 * Project Capstone Smart Bag
 * Author: Phylicia Bediako
 * Date: 8/4/25
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "IoTTimer.h"
#include "credentials.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_BME280.h"

//PUBLISH code here
//TCPClient TheClient;
//Adafruit_MQTT_Publish inTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.insidebagtemp");
//Adafruit_MQTT_Publish outTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.outsidebagtemp");
//Adafruit_MQTT_Publish leak = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.leakindicator");

float pubValue;
void MQTT_connect();
void MQTT_ping();
unsigned int lastPubTime; //maybe for printing at regular intervals?

//Water Sensor
const int WATERSENSOR=D5; //is more responsive at 5V than 3.3V
int waterVal;
const char* waterMsg;
bool waterChange;

//BME Variables--NEED 2 (ADDRESSES x76 AND x77)
float inTempC,inTempF,outTempC,outTempF;
const char DEGREE=0xF8;
const char PCT=0x25;
const int BME280=0x76;
// const int BMEXXXX=0x77;
bool status;
Adafruit_BME280 bmeInner;
//2ndBME bmeOuter;
bool tempProblem, leakProblem;

//OLED
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);
const int OLEDADDR=0x3C;

//Timer
IoTTimer timer; 

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

/*******************************************************************/
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  status = bmeInner.begin(BME280);
   if (status == false){
     Serial.printf("BME280 at address 0x%02x failed to start",BME280);
   }

  //  status = bmeOuter.begin(XXXXXXX);
  //  if (status == false){
  //    Serial.printf("BMEXXXXX at address 0x%02x failed to start",BMEXXXXX);
  //  }

  display.begin(SSD1306_SWITCHCAPVCC,OLEDADDR);
  display.display();

  pinMode(WATERSENSOR,INPUT);

  timer.startTimer(10);
}

/********************************************************************/
void loop() {
  MQTT_connect();
  MQTT_ping();

  inTempC=bmeInner.readTemperature();
  inTempF=(inTempC*1.8)+32;

  outTempC=bmeOuter.readTemperature();
  outTempF=(outTempC*1.8)+32;

  waterVal=digitalRead(WATERSENSOR);
  if ((waterChange =! waterChange)){
    if ((waterVal == 0)){
      waterMsg = "No leak or spill detected";
    }
    else{
      waterMsg = "Possible leak or spill";
    }
      // Serial.printf("%s\n",waterMsg);
  }


  if((millis()-lastPubTime)>30000){
    if(mqtt.Update()){
      inTemp.publish(inTempF,1);
      outTemp.publish(outTempF,1);
      leak.publish(waterVal,1);
    }
    lastPubTime = millis();
  }

  //Serial.printf("%f\n",waterVal);

  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0,0);
  // display.clearDisplay();
  // display.printf("Danger Zone:\n40F-140F\n",inTempF,outTempF,waterVal);
  // display.display();

  //if(timer.isTimerReady()){
    //  ;
    //}
}

/*********************************************************************/
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

/************************************************************/
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
