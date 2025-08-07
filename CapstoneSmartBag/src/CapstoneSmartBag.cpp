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

//BME Variables
float inTempC,inTempF,outTempC,outTempF;
const char DEGREE=0xF8;
const char PCT=0x25;
const int OUTBME280=0x76;
const int INBME280=0x77;
bool statusOut, statusIn, dangerZone, cautionZone;
Adafruit_BME280 bmeInner;
Adafruit_BME280 bmeOuter;

//OLED
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);
const int OLEDADDR=0x3C;

//MPU
const int MPU_ADDR=0x68;
byte accel_x_h, accel_x_l;
byte accel_y_h, accel_y_l;
byte accel_z_h, accel_z_l;
int16_t accel_x, accel_y, accel_z;
float accelXG, accelYG, accelZG;
float accelXGSq, accelYGSq,accelZGSq;
float aTot;
const float CONVFACTOR= 0.0000612061;
int leanTopple, shakerCount;
bool bagShaken;

float pitchDeg, pitchRad;
float rollDeg, rollRad;
float toppleDeg, toppleRad;

//Timer
IoTTimer tempDangerTimer, hiTempDangerTimer, shakenTimer, postShakeTimer, fallTimer; 

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

/*******************************************************************/
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

  tempDangerTimer.startTimer(10);
  hiTempDangerTimer.startTimer(10);
  shakenTimer.startTimer(10);
  postShakeTimer.startTimer(10);

}

/********************************************************************/
void loop() {
  // MQTT_connect();
  // MQTT_ping();

  inTempC=bmeInner.readTemperature();
  inTempF=(inTempC*1.8)+32;

  outTempC=bmeOuter.readTemperature();
  outTempF=(outTempC*1.8)+32;

  waterVal=digitalRead(WATERSENSOR);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,6,true);

//Danger Zone- Reg Temp
  if ((inTempF >= 40) && (inTempF <= 140)){
    dangerZone=1;
  }
  else {
    dangerZone=0;
  }

//Spill-Leak Detection
  if ((waterChange =! waterChange)){
    if ((waterVal == 0)){
      waterMsg = "No spills detected";
    }
    else{
      waterMsg = "Possible spill";
    }
      // Serial.printf("%s\n",waterMsg);
  }

//Publish to Adafruit
  // if((millis()-lastPubTime)>30000){
  //   if(mqtt.Update()){
  //     inTemp.publish(inTempF);
  //     outTemp.publish(outTempF);
  //     leak.publish(waterVal);
  //   }
  //   lastPubTime = millis();
  // }

  //Serial.printf("%f\n",waterVal);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setRotation(2);
  // display.clearDisplay();
  display.printf("Danger Zone: 40-140%cF\nInside Bag: %0.1f %cF\nOutside Bag: %0.1f %cF\n%s\n",DEGREE, DEGREE, inTempF, DEGREE, outTempF, DEGREE, waterMsg);
  display.display();

  //if(timer.isTimerReady()){
    //  ;
    //}

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
      leanTopple=1; //Leaning or falling 
    }
    if ((toppleDeg > -65)){
      leanTopple=2; //Leaning or falling
    }
  //Serial.printf("Topple Degrees: %.0f, Fall Status: %i\n", toppleDeg,leanTopple);

  pitchRad = -asin(accelXG);
  pitchDeg = (360/(2*M_PI)) * pitchRad;

  rollRad = atan2(accelYG,accelZG);
  rollDeg = (360/(2*M_PI)) * rollRad;

  //if((millis()-lastPrint)>5000){
    //Serial.printf("Raw Data Acceleration: X %i, Y %i, Z %i\n", accel_x, accel_y, accel_z);
    //Serial.printf("Converted Acceleration: X %fg, Y %fg, Z %fg\n", accelXG, accelYG, accelZG);
    //Serial.printf("Pitch Radians: %f, Degrees: %f\nRoll Radians: %f, Degrees: %f\n", pitchRad, pitchDeg, rollRad, rollDeg);
    // Serial.printf("Topple Degrees: %f, Fall Status: %i\n", toppleDeg,leanTopple);

    //lastPrint=millis();
  //}

//Accelerometer Pt 2- Shaken Goodies Detector (use code like the shock assignment)
  accelXGSq=pow(accelXG,2);
  accelYGSq=pow(accelYG,2);
  accelZGSq=pow(accelZG,2);
  aTot=sqrt(accelXGSq + accelYGSq + accelZGSq);

  if((aTot > 2.2 )){
    shakerCount++;
  }

  if((shakerCount > 5)){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.setRotation(2);
    // display.clearDisplay();
    display.printf("Warning: Contents\nRecently Shaken!");
    display.display();
  }

  //Serial.printf("AccelXSq %.01f, AccelYSq %.01f, AccelZSq %.01f, Total Accel %.01f\n",accelXGSq,accelYGSq,accelZGSq,aTot);
  Serial.printf("Total Accel %.01f, Times Shaken %i\n",aTot, shakerCount);
}

// /*********************************************************************/
// void MQTT_connect() {
//   int8_t ret;
 
//   // Return if already connected.
//   if (mqtt.connected()) {
//     return;
//   }
 
//   Serial.print("Connecting to MQTT... ");
 
//   while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
//        Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
//        Serial.printf("Retrying MQTT connection in 5 seconds...\n");
//        mqtt.disconnect();
//        delay(5000);  // wait 5 seconds and try again
//   }
//   Serial.printf("MQTT Connected!\n");
// }

// /************************************************************/
// bool MQTT_ping() {
//   static unsigned int last;
//   bool pingStatus;

//   if ((millis()-last)>120000) {
//       Serial.printf("Pinging MQTT \n");
//       pingStatus = mqtt.ping();
//       if(!pingStatus) {
//         Serial.printf("Disconnecting \n");
//         mqtt.disconnect();
//       }
//       last = millis();
//   }
//   return pingStatus;
// }
