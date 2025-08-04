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

//Water Sensor
const int WATERSENSOR=D5; //is more responsive at 5V than 3.3V
float waterVal;

//OLED
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);
const int OLEDADDR=0x3C;

//Timer
IoTTimer timer; 

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  display.begin(SSD1306_SWITCHCAPVCC,OLEDADDR);
  display.display();

  pinMode(WATERSENSOR,INPUT);

  timer.startTimer(10);
}


void loop() {
  waterVal=digitalRead(WATERSENSOR);
  //Serial.printf("%f\n",waterVal);

  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0,0);
  // display.clearDisplay();
  // display.printf("");
  // display.display();

  //if(timer.isTimerReady()){
    //  ;
    //}
}
