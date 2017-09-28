/*  Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 6, 2017
 *  This is a basic example that uses the function Get_IMU to store the GY-80 variables in a structure.
 *  You can choose to print values to debug and test in the serial monitor.
 *  The data is printed in a CSV way, so you can copy and paste the serial monitor info into a notepad file and save as a CSV that can be opened in Excel or other CSV softwares.
 *  The structure IMU_s is given by :
 *      IMU_s->double acelerometro[2]; Where positions 0, 1 and 2 in the array are acelerometer x, y and z values respectively, in m/s².
 *      IMU_s->int magnetometro[2]; Where positions 0, 1 and 2 in the array are magnetic field x, y and z values respectively, in vector form.
 *      IMU_s->int giroscopio[2]; Where positions 0, 1 and 2 in the array are gyroscope x, y and z values respectively, in angular acceleration.
 *      IMU_s->double barometro[2]; Where positions 0, 1 and 2 in the array are pressure(in Pa), altitude(in Meters) and temperature(in Celsius) respectively.    
 *  Contact : marcelomaronas at poli.ufrj.br
 *  For more codes : github.com/engmaronas
 */

/* GY-80 Pins
 *  Vcc_In <----------------------> Arduino 5v
 *  Gnd    <----------------------> Arduino Gnd
 *  SDA    <----------------------> A4
 *  SCL    <----------------------> A5
 */

#include <GY80IMU.h> //Include the library GY80IMU
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

#define RUIDO   0.05
#define SQUIB_PIN     4 //PINO QUE, QUANDO EM HIGH, IGNITA O SQUIB
#define LED_PIN   8 // PINO DO LED QUE INDICA QUE A ELETRONICA TA FUNCIONANDO
#define WINDOW_LENGTH 16 //TAMANHO DA JANELA DO MOVING AVERAGE
#define SQUIB_LED_PIN 6 //PINO DO LED QUE ACENDE QUANDO O SQUIB DEVERIA ACENDER
#define INTERVAL      1000

int ledState = LOW;
long previousMillis = 0;

float janela[WINDOW_LENGTH];
float cumulative=0;
float prev_cumulative=0;
float altura_max;


File myFile;
String filename = "DADOS0.CSV";
int numArquivo = 0;

//Structure declaration
  IMU_s IMUs1; 
  IMU_s *pIMUs1 = &IMUs1;

 SoftwareSerial ss(3, 5); //RX, TX

//Use this variables to enable debugging via Serial Monitor
bool DebugSerial = 0;  //Prints the GY80 values from inside the "Get_IMU()" function
bool DebugSDSerial = 1; //Prints the values stored in the structure IMU_s

//Modify the Delay_Time variable to control how much info is printed on the serial monitor
float Delay_Time = 0;


int validation_counter=0;

void PushPull(float vetor[WINDOW_LENGTH], float value) {
  int iterador;
  for (iterador=0;iterador< WINDOW_LENGTH-1;iterador++) {
    vetor[iterador] = vetor[iterador+1];  
    }
    vetor[iterador] = value;
  }


void setup() {
  Serial.begin(9600); //Starts the serial port at 115200 baud rate
  ss.begin(9600);
  Wire.begin();
  pinMode(LED_PIN, OUTPUT); 
  pinMode(SQUIB_PIN, OUTPUT); 
  pinMode(SQUIB_LED_PIN, OUTPUT); 


  //
  Serial.println(F("GY-80 IMU library "));
  Serial.println(F("by Marcelo Maronas"));
  Serial.println();
  //adição Mirlene
  ss.println(F("GY-80 IMU library "));
  ss.println(F("by Marcelo Maronas"));
  ss.println();
  //

  ss.println(F("sep =, ")); //This line handles Excel CSV configuration.
  ss.println(F("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ"));
  Serial.println(F("sep =, ")); //This line handles Excel CSV configuration.
  Serial.println(F("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ"));

  if (!SD.begin(10)) {
    Serial.println(F("initialization failed!"));
    ss.println(F("initialization failed!")); //Adição Mirlene
    return;
  }
  Serial.println(F("initialization done."));
  ss.println(F("initialization done.")); //Adição Mirlene

  for (int ff = 0; ff < 100; ff++){
      numArquivo++;
      filename = "DADOS" + String(numArquivo, DEC) + ".CSV";
      if (!SD.exists(filename)){
        break;
      }
   }

   myFile = SD.open(filename, FILE_WRITE);
  
  if (myFile) {
  myFile.println(F("sep =, ")); //This line handles Excel CSV configuration.
  myFile.println(F("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ"));
  myFile.close();
   } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
    ss.println(F("error opening test.txt")); //Adição Mirlene
  }

  for (int t =0;t<(sizeof(janela)/sizeof(janela[0]));t++) {
  janela[t] = 0.0;
}

}


int i=0;

void loop() {  

// --------------------------------------------------LED---------------------------//
  unsigned long currentMillis = millis(); 
  if(currentMillis - previousMillis > INTERVAL) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(LED_PIN, ledState);
  }
  
//-----------------------------------------FIM DO LED ------------------------------//

//--------------------------------------------IMU & SD------------------------------//
  //Stores the GY80 values into the pIMUs1 pointer
   Get_IMU(pIMUs1, DebugSerial);
   
   myFile = SD.open(filename, FILE_WRITE);
  //Delay time
   delay(Delay_Time);

     // if the file opened okay, write to it:
  if (myFile) {
    myFile.print(millis());myFile.print(F(" ,"));
    myFile.print(pIMUs1->barometro[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->barometro[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->barometro[2]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->acelerometro[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->acelerometro[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->acelerometro[2]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->giroscopio[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->giroscopio[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->giroscopio[2]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->magnetometro[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->magnetometro[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->magnetometro[2]);myFile.print(F(" ,"));
    myFile.close();
    Serial.println(F("Dados gravados"));
    ss.println(F("Dados gravados")); //Adição Mirlene
  }
  else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
    ss.println(F("error opening test.txt")); //Adição Mirlene
  }
   
  //if DebugSDSerial = 1, write into the Serial Monitor
  if (DebugSDSerial) {
//    ss.print(millis());ss.print(" , ");
//    ss.print(pIMUs1->barometro[0]);ss.print(" , ");
//    ss.print(pIMUs1->barometro[1]);ss.print(" , ");
//    ss.print(pIMUs1->barometro[2]);ss.print(" , ");
//    ss.print(pIMUs1->acelerometro[0]);ss.print(" , ");
//    ss.print(pIMUs1->acelerometro[1]);ss.print(" , ");
//    ss.print(pIMUs1->acelerometro[2]);ss.print(" , ");
//    ss.print(pIMUs1->giroscopio[0]);ss.print(" , ");
//    ss.print(pIMUs1->giroscopio[1]);ss.print(" , ");
//    ss.print(pIMUs1->giroscopio[2]);ss.print(" , ");
//    ss.print(pIMUs1->magnetometro[0]);ss.print(" , ");
//    ss.print(pIMUs1->magnetometro[1]);ss.print(" , ");
//    ss.print(pIMUs1->magnetometro[2]);ss.print(" , ");
    ss.print(F("Tempo em millis : "));
    ss.print(millis());ss.print(F(" Pressão Atual : "));
    ss.print(pIMUs1->barometro[0]);ss.print(" Altitude Atual : ");
    ss.print(pIMUs1->barometro[1]);ss.print("");
    if ((pIMUs1->barometro[1])>=altura_max) {
    altura_max = pIMUs1->barometro[1];
    }
    ss.print(F(" Altitude Maxima : "));ss.println(altura_max);
    Serial.print(millis());Serial.print(F(" ,"));
    Serial.print(pIMUs1->barometro[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->barometro[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->barometro[2]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->acelerometro[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->acelerometro[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->acelerometro[2]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->giroscopio[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->giroscopio[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->giroscopio[2]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->magnetometro[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->magnetometro[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->magnetometro[2]);Serial.print(F(" ,"));
}


//-------------------------------FIM DO IMU & SD-----------------------------------//

//------------------------------------MA & SQUIB ----------------------------------//
if (i<16){
  janela[i] = pIMUs1->barometro[1];  
}

if (i==16){
  for (int s=0;s<16;s++) {
    cumulative += janela[s];
  }
}
if (i>16) {
  prev_cumulative = cumulative;
  cumulative -= janela[0];
  cumulative += pIMUs1->barometro[1];
  PushPull(janela,pIMUs1->barometro[1]);
  if (((abs(pIMUs1->acelerometro[0]) < 1) && (abs(pIMUs1->acelerometro[1]) < 1) && (abs(pIMUs1->acelerometro[2]) < 1))&&(janela[1] - pIMUs1->barometro[1] >= RUIDO)){
    validation_counter++;
  }
  
  if (validation_counter>=1){
    digitalWrite(SQUIB_PIN,HIGH);
    digitalWrite(SQUIB_LED_PIN,HIGH);
  }
}
//---------------------------------FIM DO MA & SQUIB -------------------------------//
Serial.println(i);
ss.println(i); //Adição Mirlene
i++;
Serial.println("Debug do squib launcher : ");
Serial.print("VALIDATION COUNTER : ");
Serial.println(validation_counter);
Serial.println("CUMULATIVE");
Serial.println(cumulative);
Serial.println("PREVIOUS CUMULATIVE");
Serial.println(prev_cumulative);

//Adições da Mirlene
ss.println("Debug do squib launcher : ");
ss.print("VALIDATION COUNTER : ");
ss.println(validation_counter);
ss.println("CUMULATIVE");
ss.println(cumulative);
ss.println("PREVIOUS CUMULATIVE");
ss.println(prev_cumulative);

}
