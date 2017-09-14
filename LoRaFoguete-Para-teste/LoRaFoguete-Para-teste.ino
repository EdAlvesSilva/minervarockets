// COMENTARIOS MINERVA ROCKETS

// Esse codigo eh um dos codigos de exemplo feitos para o LoRa
// Procure o codigo LoRaTX para poder realizar o teste
// Comentarios em portugues foram adicionados e o codigo 
// foi levemente alterado. 

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

// * podem ser alterados. Os demais pinos sao particulares do Arduino UNO
// [COBRUF 2017 : Serao utilizadas as conexões feitas acima]

#include <SPI.h>
#include <RH_RF95.h>
#include <EEPROM.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define EEPROMADDRESS 0

// Frequência do LoRa! [COUBRUF 2017: USAR 915.0]
#define RF95_FREQ 915.0

// Criacao do objeto da classe RH_RF95
RH_RF95 rf95(RFM95_CS, RFM95_INT);


int16_t packetnum = 0;  // contador do numero de pacotes enviados

void setup() 
{
  packetnum = EEPROM.read(EEPROMADDRESS);
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

void loop()
{
  Serial.println("Enviando Pacote para o o servidor");
  // Send a message to rf95_server
  
  char radiopacket[20] = "ROCKETS!!! #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("ENVIANDO "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Esperando o pacote ser enviado..."); delay(10);
  
  // Espera o pacote ser enviando
  rf95.waitPacketSent();
  Serial.print("Pacote enviado! Tempo : ");
  delay(100);
}
