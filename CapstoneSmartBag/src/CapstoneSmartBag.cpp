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

//I2C Device addresses
// I2c device found at address 0x3C - OLED
// I2c device found at address 0x68 - MPU
// I2c device found at address 0x76 - BME1 (outer)
// I2c device found at address 0x77 - BME2 (inner)

//Adafruit Publishing
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish inTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.insidebagtemp");
Adafruit_MQTT_Publish outTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.outsidebagtemp");
Adafruit_MQTT_Publish leak = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.leakindicator");
Adafruit_MQTT_Publish tempMsg = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.tempstatusmsg");
Adafruit_MQTT_Publish dzTimeMins = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.dangerzonemins");
Adafruit_MQTT_Publish hiDzTimeMins = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.hidangerzonemins");
Adafruit_MQTT_Publish bagShake = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.bagshakeindicator");
Adafruit_MQTT_Publish fallLean = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/iotcapstone.bagfallleanindicator");

void MQTT_connect();
bool MQTT_ping();
unsigned int lastPubTime, lastPubTime2; //maybe for printing at regular intervals?

//Water Sensor
const int WATERSENSOR=D5; //is more responsive at 5V than 3.3V
int waterVal;
const char* waterMsg;
bool waterChange;

//BME Variables
float inTempC,inTempF,outTempC,outTempF;
const char DEGREE=0xF8;
const char PCT=0x25;
const int OUTBME280=0x76;
const int INBME280=0x77;
bool statusOut, statusIn;
int dangerZone;
Adafruit_BME280 bmeInner;
Adafruit_BME280 bmeOuter;
float dangerTimeMins, hiDangerTimeMins, dangerTimeSec, hiDangerTimeSec;
const char* dangerMsg;

//OLED
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);
const int OLEDADDR=0x3C;
int lastDisplayTime;

//MPU
const int MPU_ADDR=0x68;
byte accel_x_h, accel_x_l;
byte accel_y_h, accel_y_l;
byte accel_z_h, accel_z_l;
int16_t accel_x, accel_y, accel_z;
float accelXG, accelYG, accelZG, accelXGSq, accelYGSq,accelZGSq, aTot;
float pitchDeg, pitchRad, rollDeg, rollRad, toppleDeg, toppleRad;
const float CONVFACTOR= 0.0000612061;
int leanTopple, shakerCount;
bool bagShaken;
const char* shakeMsg;

//Timer
IoTTimer tempDangerTimer, hiTempDangerTimer, shakenTimer; 

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

/*********************************************************************/
/*********************************************************************/
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  
  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission(true);

  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  statusOut = bmeOuter.begin(OUTBME280);
   if (statusOut == false){
     Serial.printf("OuterBME280 at address 0x%02x failed to start",OUTBME280);
   }

   statusIn = bmeInner.begin(INBME280);
   if (statusIn == false){
      Serial.printf("InnerBME280 at address 0x%02x failed to start",INBME280);
   }

  display.begin(SSD1306_SWITCHCAPVCC,OLEDADDR);
  display.display();

  pinMode(WATERSENSOR,INPUT);

}

/*********************************************************************/
/*********************************************************************/
void loop() {
  MQTT_connect();
  MQTT_ping();

  inTempC=bmeInner.readTemperature();
  inTempF=(inTempC*1.8)+32;

  outTempC=bmeOuter.readTemperature();
  outTempF=(outTempC*1.8)+32;

  waterVal=digitalRead(WATERSENSOR);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,6,true);

//Danger Zone Var's
//Safe zone
  if ((inTempF <= 35) || (inTempF >= 150)){
    dangerZone = 0;
    dangerMsg = "Safe Zone Temp";
  }

//Caution Zone
  if (((inTempF > 35) && (inTempF < 40)) || ((inTempF > 140) && (inTempF < 150))){
    dangerZone = 1;
    dangerMsg = "Caution Zone Temp";
  }

//Danger Zone- Reg Temp Outside
  if ((inTempF >= 40) && (inTempF <= 140) && (outTempF < 90)){
    dangerZone=2;
    dangerMsg = "Danger Zone Temp";
    // tempDangerTimer.startTimer(20000);
    //Serial.printf("Danger Zone Time: %f\n",dangerTimer);

  }
  if ((dangerZone == 2)){
   dangerTimeSec = millis()/1000.0;
   dangerTimeMins = (dangerTimeSec/60.0);
   if((dangerTimeMins > 999)){
    dangerTimeMins = 999;
   }
  }
  // if (tempDangerTimer.isTimerReady()){
  //   // display.setTextSize(1);
  //   // display.setTextColor(WHITE);
  //   // display.setCursor(0,32);
  //   // display.setRotation(2);
  //   // display.printf("Danger Time Exceeded");
  // }

//Danger Zone- High Temp Outside
  if (((inTempF >= 40) && (inTempF <= 140)) && (outTempF >=90)) {
    dangerZone=3;
    dangerMsg = "HiTemp Danger Zone";
    // hiTempDangerTimer.startTimer(10000);
    // Serial.printf("HiTemp Danger Zone Time: %f\n", dangerTimer);
  }
  if ((dangerZone == 3)){
    hiDangerTimeSec = millis()/1000.0;
    hiDangerTimeMins = (hiDangerTimeSec/60.0);
    if((hiDangerTimeMins > 999)){
      hiDangerTimeMins = 999;
    }
  }
  // if (hiTempDangerTimer.isTimerReady()){
  //   // display.setTextSize(1);
  //   // display.setTextColor(WHITE);
  //   // display.setCursor(0,32);
  //   // display.setRotation(2);
  //   // display.printf("Danger Time Exceeded");
  // }

  Serial.printf("Danger (mins) %.0f HiDanger (mins) %.0f\n",dangerTimeMins,hiDangerTimeMins);

//Spill-Leak Detection
  if ((waterChange =! waterChange)){
    if ((waterVal == 0)){
      waterMsg = "No spills detected";
    }
    else{
      waterMsg = "Possible spill";
    }
  }


//Accelerometer Pt 1- Topple Detector
  accel_x_h = Wire.read();
  accel_x_l = Wire.read();

  accel_y_h = Wire.read();
  accel_y_l = Wire.read();

  accel_z_h = Wire.read();
  accel_z_l = Wire.read();

  accel_x = accel_x_h<<8 | accel_x_l;
  accel_y = accel_y_h<<8 | accel_y_l;
  accel_z = accel_z_h<<8 | accel_z_l;

//Measuring roll based on the z axis
  accelXG= (CONVFACTOR * accel_x);
  accelYG= (CONVFACTOR * accel_y);
  accelZG= (-CONVFACTOR * accel_z);

  toppleRad = -asin(accelZG);
  toppleDeg = (360/(2*M_PI)) * pitchRad;


  if ((toppleDeg <= -75)){
    leanTopple=0; //Standing
  }
  if ((toppleDeg > -75) && (toppleDeg <= -65 )){
    leanTopple=1; //Leaning
  }
  if ((toppleDeg > -65)){
    leanTopple=2; //Falling
  }
  //Serial.printf("Topple Degrees: %.0f, Fall Status: %i\n", toppleDeg,leanTopple);

  pitchRad = -asin(accelXG);
  pitchDeg = (360/(2*M_PI)) * pitchRad;

  rollRad = atan2(accelYG,accelZG);
  rollDeg = (360/(2*M_PI)) * rollRad;

    //Serial.printf("Raw Data Acceleration: X %i, Y %i, Z %i\n", accel_x, accel_y, accel_z);
    //Serial.printf("Converted Acceleration: X %fg, Y %fg, Z %fg\n", accelXG, accelYG, accelZG);
    //Serial.printf("Pitch Radians: %f, Degrees: %f\nRoll Radians: %f, Degrees: %f\n", pitchRad, pitchDeg, rollRad, rollDeg);
    // Serial.printf("Topple Degrees: %f, Fall Status: %i\n", toppleDeg,leanTopple);

//Accelerometer Pt 2- Shaken Goodies Detector (use code like the shock assignment)
  accelXGSq=pow(accelXG,2);
  accelYGSq=pow(accelYG,2);
  accelZGSq=pow(accelZG,2);
  aTot=sqrt(accelXGSq + accelYGSq + accelZGSq);

  if((aTot > 2.2 )){
    shakerCount++;
  }

  if((shakerCount == 5)){
    shakenTimer.startTimer(7000);
    bagShaken = 1;
  }

  if ((bagShaken==1)){
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,48);
      display.setRotation(2);
      display.printf("CONTENTS SHAKEN!");

    if((shakenTimer.isTimerReady())){
      bagShaken = 0;
      shakerCount = 0;
    }
  }
  Serial.printf("Total Accel %.01f, Times Shaken %i\n",aTot, shakerCount);

//Publish to Adafruit
  if((millis()-lastPubTime)>300000){
    if(mqtt.Update()){
      inTemp.publish(inTempF); //in-bag temp
      outTemp.publish(outTempF); //outside bag temp
      tempMsg.publish(dangerMsg); //safe, caution, danger, or hiTemp danger zone
      dzTimeMins.publish(dangerTimeMins);
      hiDzTimeMins.publish(hiDangerTimeMins);
    }
    lastPubTime = millis();
  }

  if((millis()-lastPubTime2)>120000){
    if(mqtt.Update()){
      leak.publish(waterVal);
      bagShake.publish(bagShaken);
      fallLean.publish(leanTopple);
      }
    lastPubTime2 = millis();
  }

//Display on OLED
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setRotation(2);
  display.printf("Inner Temp: %.01f %cF\nOuter Temp: %.01f %cF\n%s\nDZ: %.0f of 120m max\nHiDZ: %.0f of 60m max\n%s",inTempF, DEGREE, outTempF, DEGREE, dangerMsg, dangerTimeMins, hiDangerTimeMins, waterMsg);
  display.display();
  display.clearDisplay();
}

/*********************************************************************/
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

/*********************************************************************/
/*********************************************************************/
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
