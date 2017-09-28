// Receptor de telemetria
#include <SoftwareSerial.h>

char Dados; 

//SoftwareSerial ss(5, 7); //RX, TX

void setup()
{
  Serial.begin(9600);
  Serial.println("Recepcao APC220 - Aguardando...");
}

void loop()
{
  while (Serial.available())
    {
      Dados = Serial.read();
      Serial.print(Dados);
    }
}

