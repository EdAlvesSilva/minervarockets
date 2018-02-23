// COMENTARIOS MINERVA ROCKETS

// Código Feito para testar a taxa de transmissao
// e a taxa de perda de pacotes do LoRa.
// Esse eh o codigo de envio de pacotes. 

//Conexoes feitas :

//ARDUINO <--------------> LORA
// 5V     <--------------> VIN
// GND    <--------------> GND
// 13     <--------------> SCK 
// 12     <--------------> MISO
// 11     <--------------> MOSI
// 04*    <--------------> CS
// 03*    <--------------> G0
// 02     <--------------> RST


#include <SPI.h>
#include <RH_RF95N.h>
#include <time.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Frequência do LoRa! 
#define RF95_FREQ 915.0

// Criacao do objeto da classe RH_RF95
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);
  // Reinicialização Manual
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa nao inicializou");
    Serial.println("Realizando nova tentativa...");
  }
  Serial.println("LoRa inicializado!");

  // Apos reinicializacao, eh necessesario setar o valor da frequencia
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // contador do numero de pacotes enviados

void loop()
{
  Serial.println("Enviando Pacote para o o servidor");
  // Send a message to rf95_server
  
  char * radiopacket;
  float tempo = (double)time(NULL);
   snprintf(radiopacket,1000,"ID:%d\ttimestamp:%.2f\0",packetnum++,tempo);
  
  Serial.print("ENVIANDO "); Serial.println(radiopacket);

  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, sizeof(radiopacket)/sizeof(char));
 
  Serial.println("Esperando o pacote ser enviado..."); delay(10);
  rf95.waitPacketSent();
  // Espera o pacote ser enviando
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Esperando uma resposta..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Resposta obtida : ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Resposta nao foi recebida corretamente");
    }
  }
  else
  {
    Serial.println("Nao houve resposta. Verifique o modulo TX");
  }
  delay(1000);
}
