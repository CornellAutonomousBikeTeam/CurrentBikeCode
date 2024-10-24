
#include "TinyGPS++.h"

TinyGPSPlus gps;
//make sure to wire up the gps so that TX on gps goes to RX on PCB.

void setup() {
  Serial.begin(115200); //PC
  Serial3.begin(9600); //GPS
  }

void loop() {
      while (Serial3.available() > 0) 
        gps.encode(Serial3.read());
        
        Serial.print("Latitude                      "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude                     "); Serial.println(gps.location.lng(), 6);
        Serial.print("Hours                         "); Serial.println(gps.time.hour()); // Hour (0-23)
        Serial.print("Minutes                       "); Serial.println(gps.time.minute()); // Minute (0-59)
        Serial.print("Seconds                       "); Serial.println(gps.time.second()); // Second (0-59)
        Serial.print("Centiseconds                  "); Serial.println(gps.time.centisecond()); // 100ths of a second (0-99)
        Serial.print("Date                          "); Serial.println(gps.date.value()); // Raw date in DDMMYY format
        Serial.print("Number of Satelites in use    "); Serial.println(gps.satellites.value()); // Number of satellites in use
        Serial.print("Course in degrees             "); Serial.println(gps.course.deg());        
        delay(100);
}
