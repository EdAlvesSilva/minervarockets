Esse arquivo ReadMe explica o que cada c�digo faz :

MVPCobruf_0_1 : Esse � o c�digo do MVP Curitiba:
		GPS -> Funciona, via SoftwareSerial usando a biblioteca TinyGPS;
		Vari�veis do GPS -> Latitude, Longitude, Altitude, Satellites, Speed;
		IMU -> Funciona, via biblioteca GY80IMU;
		SD -> Funciona, via biblioteca SD.h do arduino;
		Consumo de SRAM : 69%
		Pode debugar via SerialMonitor

MVPCobruf_0_1_com_NeoGPS : Esse � o c�digo do MVP Curitiba com a NeoGPS no lugar da TinyGPS :
		GPS -> Funciona, via HardwareSerial, usando a biblioteca NeoGPS;
		Vari�veis do GPS -> Latitude, Longitude, Altitude;
		IMU -> Funciona, via biblioteca GY80IMU;
		SD -> Funciona, via biblioteca SD.h do arduino;
		Consumo de SRAM : 60%
		NAO pode debugar via SerialMonitor, apenas as vari�veis do GY80IMU s�o debug�veis and quando est� debugando os dados do GPS s�o corrompidas

MVPCobruf_0_2 : Esse c�digo � o MVPCobruf_0_1 com o c�digo do LoRa:
		Nada funciona, o arduino d� crash com a falta de mem�ria.
		Consumo de SRAM : 89%
	        Pode debugar via SerialMonitor

MVPCobruf_0_4_com_NeoGPS : Esse c�digo � o MVPCobruf_0_2 com o NeoGPS no lugar da TinyGPS:
		Arduino n�o d� crash mas nao consegue guardar nenhum dado no SD e nem receber sinal do GPS
		Consumo de SRAM : 80%
		NAO pode debugar via SerialMonitor, apenas as vari�veis do GY80IMU s�o debug�veis and quando est� debugando os dados do GPS s�o corrompidas

