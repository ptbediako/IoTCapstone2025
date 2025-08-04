/* 
 * Project Water Sensor Test
 * Author: Phylicia Bediako
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

const int WATERSENSOR=D5;
float waterVal;

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);


void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  pinMode(WATERSENSOR,INPUT);


}


void loop() {
  waterVal=digitalRead(WATERSENSOR);
  Serial.printf("%f\n",waterVal);
  //delay(1000);
}
