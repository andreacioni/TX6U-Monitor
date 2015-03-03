/* Author: Andrea Cioni, 3/3/2015 */

#include <TX6U.h>
TX6U t(2,0); //Arduino UNO pin 2 -> interrupt 0

void setup() {
  Serial.begin(9600);
  t.setCelsius(true);
  t.setup();
}

void loop() {
  if(t.available()) {
    struct msg_map msg = t.get();
    Serial.println("Message recognized from LaCrosse TX6U device!");
    Serial.print("ID: ");
    Serial.println(msg.id);
    Serial.print("Temperature: ");
    Serial.print(msg.temperature);
    Serial.println(" °C");
    Serial.println();
  }
}
