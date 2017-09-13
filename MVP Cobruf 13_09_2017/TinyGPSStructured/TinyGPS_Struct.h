// Tiny GPS Structured Library
// Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 5, 2017
// Using Tiny GPS lilbrary.
// Contact : marcelomaronas at poli.ufrj.br
// For more codes : github.com/engmaronas

#include "Arduino.h" 
#include "TinyGPS.h"
#include <SoftwareSerial.h>

#ifndef TinyGPS_Struct
#define TinyGPS_Struct

typedef struct
{
 float flat, flon;
 int sat;
 float height, fspeed;
} GPS_s; // GPS Structure

int Get_GPS (GPS_s *GPSstruct, SoftwareSerial &ss);

#endif