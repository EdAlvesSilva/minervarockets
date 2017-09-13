#include <SoftwareSerial.h>
#include <GY80IMU.h>
#include <TinyGPS.h>
//#include <SD.h>
//#include <SPI.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

/* This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** SCK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
*/

TinyGPS gps;
SoftwareSerial ss(3, 2);
bool DebugSerial = 0;
bool DebugSDSerial = 1;
IMU_s IMUs1;
IMU_s *pIMUs1;
//File myFile;
float flat, flon;
unsigned long age;

void setup()
{
  Serial.begin(115200);
  ss.begin(9600);
  Wire.begin();
  // Inicializa o BMP085
  bmp085Calibration();
  
//  Serial.print("Initializing SD card...");
//
//  if (!SD.begin(4)) {
//    Serial.println("initialization failed!");
//    return;
//  }
//  Serial.println("initialization done.");
//    // open the file. note that only one file can be open at a time,
//  // so you have to close this one before opening another.
//  myFile = SD.open("LOG.txt", FILE_WRITE);
//
//  // if the file opened okay, write to it:
//  if (myFile) {
//    Serial.print("Writing to LOG.csv...");
//    myFile.println("sep =,");
//    myFile.println("Pressao, Altitude, Temperatura, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ, GPS Lat, GPS Long, Satellites"); 
//    // close the file:
//    myFile.close();
//    Serial.println("Escreveu cabecalho.");
//}
  Serial.println("sep =, ");
  Serial.println("Pressao, Altitude, Temperatura, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ, GPS Lat, GPS Long, Satellites"); 
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 50;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
//    Serial.print("LAT=");
//    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
//    Serial.print(" LON=");
//    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
//    Serial.print(" SAT=");
//    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
//    Serial.print(" PREC=");
//    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
//  Serial.print(" CHARS=");
//  Serial.print(chars);
//  Serial.print(" SENTENCES=");
//  Serial.print(sentences);
//  Serial.print(" CSUM ERR=");
//  Serial.println(failed);
  if (chars == 0)
  Serial.println("** No characters received from GPS: check wiring **");

  Get_IMU(pIMUs1, DebugSerial);

//  myFile = SD.open("LOG.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (DebugSDSerial) {
    Serial.print(pIMUs1->barometro[0]);Serial.print(" ,");
    Serial.print(pIMUs1->barometro[1]);Serial.print(" ,");
    Serial.print(pIMUs1->barometro[2]);Serial.print(" ,");
    Serial.print(pIMUs1->acelerometro[0]);Serial.print(" ,");
    Serial.print(pIMUs1->acelerometro[1]);Serial.print(" ,");
    Serial.print(pIMUs1->acelerometro[2]);Serial.print(" ,");
    Serial.print(pIMUs1->giroscopio[0]);Serial.print(" ,");
    Serial.print(pIMUs1->giroscopio[1]);Serial.print(" ,");
    Serial.print(pIMUs1->giroscopio[2]);Serial.print(" ,");
    Serial.print(pIMUs1->magnetometro[0]);Serial.print(" ,");
    Serial.print(pIMUs1->magnetometro[1]);Serial.print(" ,");
    Serial.print(pIMUs1->magnetometro[2]);Serial.print(" ,");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);Serial.print(" ,");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);Serial.print(" ,");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());Serial.println();
}
}
