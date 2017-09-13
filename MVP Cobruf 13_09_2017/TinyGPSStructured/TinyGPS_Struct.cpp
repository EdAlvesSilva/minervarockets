// Tiny GPS Structured Library
// Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 5, 2017
// Using Tiny GPS lilbrary.
// Contact : marcelomaronas at poli.ufrj.br
// For more codes : github.com/engmaronas

#include <TinyGPS_Struct.h>
TinyGPS gps;

int Get_GPS (GPS_s *GPSstruct, SoftwareSerial &ss)
{
  float flat, flon;
  unsigned long age; 
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 100;)
  {
    while (ss.available())
    {
      char c = ss.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    //Serial.print(F("LAT="));
    //Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    //Serial.print(F(" LON="));
    //Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //Serial.print(F(" SAT="));
    //Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    //Serial.print(F(" PREC="));
    //Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
//  Serial.print(" CHARS=");
//  Serial.print(chars);
//  Serial.print(" SENTENCES=");
//  Serial.print(sentences);
//  Serial.print(" CSUM ERR=");
//  Serial.println(failed);
  if (chars == 0)
  Serial.println(F("** No characters received from GPS: check wiring **"));
  
  GPSstruct->flat = flat;
  GPSstruct->flon = flon;
  GPSstruct->sat = gps.satellites();
  GPSstruct->height = gps.f_altitude();
  GPSstruct->fspeed = gps.f_speed_kmph();

  if (GPSstruct->sat == 0) {
  return 1;
}
  else {
  return 0;
}
}