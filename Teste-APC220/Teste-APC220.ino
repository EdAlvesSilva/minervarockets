//ESSE É UM CODIGO PARA TESTE DO APC220
//PARA O PROJETO CLASSE E - COBRUF 2017


int16_t packetnum = 0;  // contador do numero de pacotes enviados

void setup() 
{

  while (!Serial);
  Serial.begin(9600);
}

void loop()
{
  
  char radiopacket[] = "ROCKETS!!! MEU NOME EH JONAS #      ";
  sprintf(radiopacket,"%s%i",radiopacket,packetnum++);
  Serial.println(radiopacket);
  delay(100);
}
