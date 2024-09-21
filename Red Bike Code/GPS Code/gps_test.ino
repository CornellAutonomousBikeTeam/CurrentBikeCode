#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <Wire.h> //Needed for I2C to GPS

SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);

  while(!Serial);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  Serial.println("Latitude/Longitude, Acquisition Time");
}

void loop()
{
  unsigned long time_beforelat = micros();
  int latitude = myGNSS.getLatitude();
  unsigned long time_afterlat = micros();

  Serial.print("LA: "); Serial.print(latitude/10e6, 6); Serial.print(","); Serial.println(time_afterlat-time_beforelat);

  unsigned long time_beforelong = micros();
  int longitude = myGNSS.getLongitude();
  unsigned long time_afterlong = micros();

  Serial.print("LO:"); Serial.print(longitude/10e6, 6); Serial.print(","); Serial.println(time_afterlong-time_beforelong);
}
