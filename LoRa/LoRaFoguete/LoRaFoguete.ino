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


void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  delay(100);
  
  // Reinicialização Manual
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) { 
    //loop eterno enquanto LoRa nao inicializa.[ COBRUF 2017] : NAO UTILIZAR ISSO! 
    }
  // Apos reinicializacao, eh necessesario setar o valor da frequencia
  rf95.setFrequency(RF95_FREQ);
  
  //Definindo a potência da transmissao para 23dBm
  rf95.setTxPower(23, false);
}

void loop()
{  
    //Verifica o numero do pacote que sera enviado 
  //Assim, eh possivel garantir que o numero do pacote
  //enviado sera sempre o numero certo
  long packetnum = EEPROMReadlong();
   
  unsigned char radiopacket[20] = "ROCKETS!!! #      ";
  itoa(packetnum++, radiopacket+13, 10);
  radiopacket[19] = 0;
  
  rf95.send((uint8_t *)radiopacket, 20);

  // Espera o pacote ser enviando
  rf95.waitPacketSent();

  
  //Escreve na EEPROM o ultimo pacote enviado
  EEPROMWritelong((packetnum));
  delay(100);
}

void EEPROMWritelong(long value)
      {

      //Escreve os 4 bytes no endereco de referencia + os 3 consecutivos
      EEPROM.write(EEPROMADDRESS, value & 0xFF);//four
      EEPROM.write(EEPROMADDRESS + 1, (value >> 8) & 0xFF); //three
      EEPROM.write(EEPROMADDRESS + 2, (value >> 16) & 0xFF); //two
      EEPROM.write(EEPROMADDRESS + 3, (value >> 24) & 0xFF); //one
      }

long EEPROMReadlong()
      {
      //Le o endereco + os 3 consecutivos
      return ((EEPROM.read(EEPROMADDRESS) << 0) & 0xFF) + (EEPROM.read(EEPROMADDRESS + 1 << 8) & 0xFFFF) + ((long(EEPROM.read(EEPROMADDRESS + 2)) << 16) & 0xFFFFFF) + ((long(EEPROM.read(EEPROMADDRESS + 3)) << 24) & 0xFFFFFFFF);
      }

