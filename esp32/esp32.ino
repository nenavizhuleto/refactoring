//Serial.println("-- ESP32-Dev Module sketch_oct03a.ino ver 30.12.2022  10:24 --");  
//Serial.print("04.05.2023 ESPectro32 Esp32.ino Ver(html load from webpage.h");

//-- ADS1115 four inputs example -------------------------------------
	// добавить ADS1115  через менеджер библиотек
	//C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\Wire\src  - i2c
	//C:\GlobalCentr\Arduino\program\libraries\ADS1X15
	/* SDA	SDA (default is GPIO 21)	SCL	SCL (default is GPIO 22) 
	подключить контакт SDA модуля I2C к контакту SDA (D21) модуля ESP32;
	подключить контакт SCL модуля I2C к контакту SCL (D22) модуля ESP32.*/
#include <ADS1X15.h>
//-- initialize ADS1115 on I2C bus 1 with default address 
ADS1115 ADS(0x48); 	// ADDR:gnd 0100 1000
	//--------------
int16_t val_0, val_1, val_2, val_3;   
uint8_t flADC=1;
float f,a0,a1,a2,a3;  
 
#include "webpageLi.h"			//-- 16.02.2023 сам html сервер -------

//-- Wi-Fi -----------------------------------------------------------------------
#include <WiFi.h>
//-- Wi-Fi вводим имя и пароль точки доступа http://wiki.amperka.ru/products:esp32-wroom-wifi-devkit-v1#%D1%80%D0%B0%D1%81%D0%BF%D0%B8%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0
const char* ssid     = "Gt74";
const char* password = "GeneralGT2018";
//-- WiFi ------------------------------------------------------------
WiFiServer server(80);  	// инициализируем сервер на 80 порте
WiFiServer server1(8080);

//-- Вывод версии,времени -----------------------------------------------------------	
char Ver[]={"12.05.2023 ESPectro32 Esp32.ino Light (webpageLi.h)"};	
//-- Глобальные переменные ------------------------
long lastUpdateTime = 0;        	// Переменная для хранения времени последнего считывания с датчика
int TEMP_UPDATE_TIME = 4000;   	// Определяем периодичность проверок больше 30 000 не даёт = int =знаковый от -32000 до + 32000
uint8_t i1 = 0;
uint8_t 	k1=0,		// счётчик циклов баланса  
			k2=0,		// циклы команды chh.d16 == 13092
			sek1=0, min1=0, hour1=0;	// внутренне время		

//-- DS1820 ----------------------------------------------
#include <DallasTemperature.h>
#include <OneWire.h>
	// Питание ds1820 от 3.3в, при питании от +5в , wi-fi не конектится=не работает
	//-- OneWire: настройка объекта oneWire для связи с любым устройством OneWire,  поиск всех датчиков
	//-- OneWire oneWire(4);  pin 4 - 1 wire port ==> Arduino
OneWire oneWire(4); 	// Создаем объект OneWire для шины 1-Wire, с помощью которого будет осуществляться работа с датчиком 
						//,ещё раз для библиотеки Даллас линия данных подключена к цифровому выводу 4 Arduino
DallasTemperature sensors(&oneWire);	//-- передать ссылку на oneWire библиотеке DallasTemperature
int deviceCount = 0;
float tempC;
DeviceAddress t;
float temC[720][5];

//-- pin out ---------------------------------
#define LED_GREEN 16
#define LED_RED   17
//-- Timer ------------------------------------
hw_timer_t *My_timer = NULL;

//-- NTP ---- Библиотеки C:\GlobalCentr\Arduino\program\libraries\NTPClient\NTPClient.h
#include <NTPClient.h>
#include <WiFiUdp.h>
//-- NTP Define NTP Client to get time  172.16.250.250 --------------------------- 
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
	//-- Variables to save date and time --------------------------------
String formattedDate;
String dayStamp;
String timeStamp;		

///////////-- 12.05.2023 Тестирование SPI проводится при подключении SD карты !!!!!!!!!!!!!!!!!!!!
//--FAT,SPIFFS + SD card --------------------------------
#include "FS.h"		//C:\Users\IgorM\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\FS
#include "SPIFFS.h"	//C:\Users\IgorM\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\SPIFFS\src
#define FORMAT_SPIFFS_IF_FAILED true
	/* You only need to format SPIFFS the first time you run a test or else use the SPIFFS plugin to create a partition
   	https://github.com/me-no-dev/arduino-esp32fs-plugin */

	//-- SPI  13.03.2023 --  C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\SPI\src ----
	//C:\Users\IgorM\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\SPI\src
	//#include <SPI.h>
//------//-- SD.begin() ----------------
#include <SD.h>		
File myFile;	
	//-- SD Задание пина SS для SD карты , иначе пин CS садить на землю - разрешение каристала, 
	//#define SDSS1pin  53                      // on the UNO the Sparkfun SDSS pin is 8 !!!
	//MOSI: 23 MISO: 19 SCK: 18 SS: 5
	/*	#define SCK 25
	#define MISO 32
	#define MOSI 26
	#define CS 33  */

//-- FTP 11.05.2023- https://github.com/ldab/ESP32_FTPClient-------------
#if (defined ESP32)
#include <WiFi.h>
#include <SPIFFS.h>
#define BAUDRATE 115200
#elif (defined ESP8266)
#include <ESP8266WiFi.h>
#include <FS.h>
#define BAUDRATE 74880
#endif

#include <FTPServer.h>
	// Since SPIFFS is becoming deprecated but might still be in use in your Projects, tell the FtpServer to use SPIFFS
FTPServer ftpSrv(SPIFFS);

//-- LTC68041 ----------------------------------------------------------------------
#include <Arduino.h>
#include <stdint.h>
#include "LTC68041.h"
//-- LTC68041 ----------------------------------------------------------------------
const uint8_t TOTAL_IC = 2;		//!<number of ICs (ltc6804) in the daisy chain ячейки "гирлянды"
/*** Global Battery Variables received from 6804 commands  These variables store the results from the LTC6804
 register reads and the array lengths must be based  on the number of ICs on the stack  */
/*!<  The cell codes will be stored in the cell_codes[][12] array in the following format:  
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
*/
uint16_t cell_codes[TOTAL_IC][12];	//-- Cell Voltage Register Group 12 каналов ch1-ch12 16 bit = 12 строк таблицы
/*!< The GPIO codes will be stored in the aux_codes[][6] array in the following format: 
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/
uint16_t aux_codes[TOTAL_IC][6];		// Auxiliary Register Group = 6 строк таблицы
//-----------------------------------------------------------------------------------
/*!<  The tx_cfg[][6] stores the LTC6804 configuration data that is going to be written   to the LTC6804 ICs on the daisy chain. The LTC6804 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:  
 |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    | 
*/
uint8_t tx_cfg[TOTAL_IC][6];		// init_cfg() Configuration Register Group для записи(установки)
//--------------------------------------
/*!<  the rx_cfg[][8] array stores the data that is read back from a LTC6804-1 daisy chain. 
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:
|rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
|---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
|IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/
uint8_t rx_cfg[TOTAL_IC][8];		// 6 reg + 2 рес // прочитанное значение  Configuration Register Group , pec = CRC = контрольная сумма

//-- Status Register Group 20.12.22 -- --------------------------------
uint16_t sta_codes[TOTAL_IC][6];		//uint16_t sta_codes[TOTAL_IC][5];
//-- 28/12/2022  Err Data------
uint8_t errNodeIsoSpi[TOTAL_IC];
//-- 30/01/2023  Flag to balanse------
uint16_t Node_6804[TOTAL_IC];

//-- Covanf from html -------------------------- 
union ddd chh;
//-- clsScreen ---------------------------
uint8_t clsScreen=1; 
//-- Режим баланса ручной/автомат , 0 = ручной ------------ 
uint8_t balMode=0; 
//-------------------------------------------------------------------
float MIN_CELL_V = 3.0;       	// Minimum allowable cell voltage. Depends on battery chemistry.
float MAX_CELL_V = 3.6;       	// Maximum allowable cell voltage. Depends on battery chemistry.

//--- Запрограммируем функцию обработки прерывания от таймера (ISR) ------------------
//В ней мы будем инвертировать состояние контакта GPIO21, к которому подключен светодиод. 
//Данная функция ISR будет вызываться каждый раз при срабатывании прерывания от таймера.
//------------------------- 
void IRAM_ATTR onTimer(){  
	sek1++;	
}
//-- FTP 11.05.2023---------------------
enum consoleaction
{
  show,
  wait,
  format,
  list
};
consoleaction action = show;

/*1. Сделать ftp server = НЕ нужно , работает web копирование =OK
1.1 Сделать FTP server = надо загружать  много файлов WEB 

2. FAT SPIFFS.begin  OK 
3. Запись log в браузере = ?????  OK = запись любых файлов с браузера
4. Схему спаять ADS1115   OK
	4.1 i2C схема подключения   OK
5.Reset плохой , надо запаять 1171СП42 - кнопка "EN"	*/

//-------------------------------------------------------------------
//-------------------------------------------------------------------
void setup() {  
  	Serial.begin(115200);  Serial.println(); Serial.println(Ver); 	Serial.println(__FILE__);	delay(2000);	//-- path to file progect -----         
  
	//-- Инициализируем контакт LED_BUILTIN (GPIO16) в режим вывода
  	pinMode(LED_GREEN, OUTPUT);
  	digitalWrite(LED_GREEN, LOW); // on

	//-- Температура:Start up the library -----------------------
  	sensors.begin();  // !!!!!!!!!!!!!!!!!!!!!!!!
  	deviceCount = sensors.getDeviceCount(); //сколько датчиков  
  	Serial.print("L200 Start ds1820 dev: ");  Serial.println(deviceCount, DEC);
	//-- Sens Temp -----------------------------
  	for (uint8_t index = 0; index < deviceCount; index++){
		sensors.getAddress(t, index);
    	printAddress(t);

    	int id = sensors.getUserData(t);
    	Serial.print("\tID: ");    Serial.println(id);
  	}

	//-- Подключаем к Wi-Fi ---------------------------------------
  	Serial.print("L211 Wi-Fi Connecting to ");	Serial.print(ssid);
  	WiFi.begin(ssid, password);  

  	while(WiFi.status() != WL_CONNECTED) {
    	delay(500);
    	Serial.print(".");
  	}  
  	Serial.println();  Serial.print("L225 IP-address: ");  Serial.println(WiFi.localIP());
  	Serial.print("Hostname:");  Serial.println(WiFi.getHostname());  WiFi.printDiag(Serial);  Serial.println();  

	//-----------------------------------------------------------
  	server.begin();	//-- Запускаем сервер 80 -------------------- 
  	//server1.begin();	//-- Запускаем сервер 8080 ------------------ 

	//-- NTPClient to get time Initialize ---------------------------
  	timeClient.begin();
  	// Set offset time in seconds to adjust for your timezone, for example:GMT +1 = 3600	// GMT -1 = -3600	// GMT 0 = 0
  	timeClient.setTimeOffset((3600*5));	//+5 часов
      
	//-- Timer -------------------------------
  	My_timer = timerBegin(0, 80, true);
  	timerAttachInterrupt(My_timer, &onTimer, true); // ссылка на исп прогу &onTimer
  	timerAlarmWrite(My_timer, 5000000, true);      	// 10 sek, микросекундный таймер   
  	timerAlarmEnable(My_timer);  

	//-- ADC -------------------------------
  	Serial.print("ADS1X15_LIB_VERSION: ");	Serial.println(ADS1X15_LIB_VERSION); 
  	ADS.begin();
 	if (!ADS.isConnected()) {    	
		Serial.print("L240 ADC NOT run ");
		flADC=0;		// error:ADS1115 not connected
  	}
	//ADS.setGain(0);	// установите значение усиления, указывающее максимальное напряжение, которое может быть измерено, 0 	±6.144V 	default

	//--- FAT ---------------------------------------------------------------------
    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        Serial.println("L247 SPIFFS Mount Failed");
        //return;
    }
	//#ifdef ESP32       //esp32 we send true to format spiffs if cannot mount
  	////////if (SPIFFS.begin(true)) {
    
	delay(3000);

///////////-- 12.05.2023 Тестирование SPI проводится при подключении SD карты !!!!!!!!!!!!!!!!!!!!
	//-- SD =>test SPI interface ----------------------------------------------
  	Serial.print("L246  Init SD card..");
	//-- Отдельная инициализация SD карты!!!!!, в 2560 проходит с 1 раза-------------------------
	SD.begin(5);	// номер порта  //SS:5
	//---------------------------
  	if (!SD.begin(5)) {		//SS:5
    	Serial.println("BAD ");  	
    	//while (1);	// STOP!
  	}
  	else Serial.println("OK ");

	delay(1000);
	
  	if (SD.exists("/test.htm")) Serial.print(" L290 /test.htm  OK "); 	//-- file, не читает расширения > 3 символов  
	else Serial.print(" L291 /test.htm  BAD ");	
	Serial.println();	delay(1000);

	//-- Open a new file and close it -------------------
  	Serial.print(" L294 try open /test.htm ");  
  	myFile = SD.open("/test.htm", FILE_WRITE);		// myFile.println("testing 1, 2, 3");
  	myFile.close();
	//-- Check to see if the file exists --------------
  	if (SD.exists("/test.htm")) Serial.print(" L299 /test.htm  OK ");
	else     Serial.print(" L300 /test.htm  BAD ");	
	Serial.println(); Serial.println();	delay(2000);

	//-- SPI pin --------------------------
  	Serial.print("L300  MOSI:");  Serial.print(MOSI);  Serial.print(" MISO:");  Serial.print(MISO);
  	Serial.print(" SCK:");  Serial.print(SCK);  Serial.print(" SS:");  Serial.println(SS); 
	Serial.println();
	delay(5000);

	//-- LTC6804 ----------------------------------------------  
  	// LTC6804_initialize()->SPI.setFrequency(125000);
	// LTC68041->spi_write_array->spi_write->{LT_SPI.cpp} SPI.transfer(data)///////////////////////////
	// LT_SPI.cpp -> spi_write(int8_t  data) == SPI.transfer(data);
	// LT_SPI.cpp -> spi_read(int8_t  data) ==> int8_t ccc = SPI.transfer(data); return ccc;
	LTC6804_initialize(); 	// 	SPI.setFrequency(125000);	 
  	init_cfg0();        		// initialize configuration array(memory) to be written  //coment: tx_cfg[i][0] = 0xFE; ......	  	
	Serial.println("L284 print_config");	//print_config();

   	wakeup_sleep();
   	LTC6804_wrcfg(TOTAL_IC,tx_cfg);    
	int8_t error = LTC6804_rdcfg(TOTAL_IC,rx_cfg, errNodeIsoSpi);

	if (error == -1)      Serial.println(" L291 Read Conf CRC error == -1");  
	Serial.println("L292 print_rxconfig");	 
	print_rxconfig();	delay(2000);		//print_menu();	*/

	clsScreen=1;			// Enable:load html    	
	//-- Состояние системы => принимать команды из веб ------------------
	chh.d16 = 12580;		// команда =1	//chh.d16 = 12836;// команда =2  
	
	//-- FTP 11.05.2023---------------------
  	bool fsok = SPIFFS.begin();
  	Serial.printf_P(PSTR("FS init: %s\n"), fsok ? PSTR("ok") : PSTR("fail!"));
	ftpSrv.begin(F("ftp"), F("ftp"));
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
void loop() {

  	byte data[2];   // хранение значения температуры
  	ADS.setGain(0);	// set the gain value, indicating the maxVoltage that can be measured Adjusting
							// установите значение усиления, указывающее максимальное напряжение, которое может быть измерено, 0 	±6.144V 	default 
	//-- TEMP_UPDATE_TIME ------------------------------------------------------
  	if ( (millis() - lastUpdateTime > TEMP_UPDATE_TIME) ){  //  sec
    	lastUpdateTime = millis();      

		Serial.println();		Serial.print("L310 ");
		//-- LTC6804 ----------------------------------------------  
		if (chh.d16 == 12580 ) {		// -- Измерения ---------------------
			measurementsCell();			
			k1=0;		//-- счётчик циклов баланса 
		}	
		else if (chh.d16 == 12836 ) {	//-- Балансировка напряжение сравнение с MAX_CELL_V,  CELL_BALANCE_THRESHOLD_V
			Set_BalanceVoltCel();			
		}	
		else if (chh.d16 == 13092 ) {	// -- Коммутация ключей ---------------------
			keySwCell();	
		}	
		//-- Flag overVoltage --------------------------------------------------
		//Serial.print(" ");	Serial.print(MIN_CELL_V);	Serial.print(" ");	Serial.print(MAX_CELL_V );	//Serial.print(" ");	Serial.print(k1);
		//-- Поиск макс значения ----------------------------------------------
  		for (int current_ic = 0 ; current_ic < TOTAL_IC ; current_ic++ ){

			if( errNodeIsoSpi[current_ic] ) { 
				continue;		// ошибка связи с нодой=пропускаем
			}
			else	MAX_CELL_V = (cell_codes[current_ic][0]*0.0001);

			for(int i=0; i<12 ; i++) {
				//-- баланс вычисление MAX_CELL_V -----------
				if ( (cell_codes[current_ic][i]*0.0001) > MAX_CELL_V ) {
					MAX_CELL_V = (cell_codes[current_ic][i]*0.0001);
				}		
			}
		}

		//MAX_CELL_V -=0.1;
  		Serial.print(" Max_Cell_V=");	Serial.println(MAX_CELL_V);	 

  		for (int current_ic = 0 ; current_ic < TOTAL_IC ; current_ic++ ){
  			Serial.print(" #");	Serial.print(current_ic+1,DEC);	 
			//Serial.print(errNodeIsoSpi[current_ic]); Serial.print(" ");		// номер контролера 6804 , контроллер на 12 ячееек - 3 группы по 4 ячейки

   		if( errNodeIsoSpi[current_ic] ) Serial.print(":ErLnk_6804");			
			/*                    12 11 10 9                       8 7 6 5 4 3 2 1
			tx_cfg[current_ic][5]:3  2  1  0 tx_cfg[current_ic][4]:8 7 6 5 4 3 2 1	*/	

			//-- Принятие решения о балансировке = наличие флага Flag overVoltage--------------------------------------------------
			int16_t ss1=0;	
			for(int i=0; i<12 ; i++) {
				ss1 = (0x01) << i; 					
				//-- Флаг на включение -- (---------------
				if ( (cell_codes[current_ic][i]*0.0001) >= MAX_CELL_V ) {
					Node_6804[current_ic] |= ss1; 
					//print_bit16(ss1); Serial.print(" ");	
					//Serial.print( cell_codes[current_ic][i]*0.0001 , 2); Serial.print(">"); Serial.print(MAX_CELL_V, 2); Serial.println();	
				}
				else	Node_6804[current_ic] &= ~(ss1);		//инверсия числа 	
				//-- Cells:Напряжение на элементах ------
				Serial.print(" ");  Serial.print(cell_codes[current_ic][i]*0.0001, 2);
			}			
			Serial.print("  Cell_Sum:");	Serial.print(sta_codes[current_ic][0]*0.002, 1); Serial.print("v ");
			tx_cfg[current_ic][4] = Node_6804[current_ic];	tx_cfg[current_ic][5] = ( (Node_6804[current_ic]) >> 8 );
			Serial.print(" tx_cfg4:"); 	Serial.print(tx_cfg[current_ic][4]);	Serial.print(" tx_cfg5:");	Serial.println(tx_cfg[current_ic][5]);
		}
		Serial.print("L369  ");

		i1++;	 	//Serial.print("L349 tic "); Serial.print(i1); Serial.print(" ");	Serial.print(i2);	Serial.print(" ");	 Serial.println(k1);
    
		//--Время  NTP ------------------------------------------- 
    	while(!timeClient.update()) {
      		timeClient.forceUpdate();
      		if ( (millis() - lastUpdateTime > TEMP_UPDATE_TIME) ) break ; // 10 sec
    	}
    	// The formattedDate comes with the following format:2018-05-28T16:00:13Z   We need to extract date and time    
    	formattedDate = timeClient.getFormattedDate();    
    	//Serial.print("L166   formattedDate ");    Serial.println(formattedDate);    
		//-- Extract date  Печатаем дату ---------------------
    	int splitT = formattedDate.indexOf("T");
    	dayStamp = formattedDate.substring(0, splitT);         
    	Serial.print(dayStamp);  Serial.print(" ");  //Serial.print("DATE:");     

		//-- Extract time -----------------------------
    	timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);        
    	Serial.print(timeStamp);    Serial.print(" "); 

		//-- LED ------------------ 
		if(i1 & 0x01) 	digitalWrite(LED_GREEN, HIGH);
		else         	digitalWrite(LED_GREEN, LOW);  	// Зажигаем светодиод низкоуровневым сигналом LOW    
    	Serial.println("    L393 ");
/*
	//-- ADC ------------------ 
  		if(flADC){		// ADS1115 RUN,OK
			val_0 = ADS.readADC(0);  
  			val_1 = ADS.readADC(1);  
  			val_2 = ADS.readADC(2);  
  			val_3 = ADS.readADC(3);   
  			f = ADS.toVoltage(1);  // voltage factor
			a0=val_0 * f * 1.1;	a1=val_1 * f * 1.96;	a2=val_2 * f * 2.95;	a3=val_3 * f * 4.88;
 		
  			Serial.print(" A0:"); Serial.print(val_0); Serial.print(" "); Serial.print(a0, 2);
  			Serial.print(" A1:"); Serial.print(val_1); Serial.print(" "); Serial.print(a1-a0, 2);
  			Serial.print(" A2:"); Serial.print(val_2); Serial.print(" ");	Serial.print(a2-a1, 2);
  			Serial.print(" A3:"); Serial.print(val_3); Serial.print(" ");   Serial.println(a3-a2, 2);
		}
*/
 	}	

	//-- Часы, таймер=k = 1 sek ----------------------------
	if( sek1 >= 59 ) {
		sek1=0; min1++;  	//Serial.println(sek);	// пишем  в COM
		if ( chh.d16 == 12836 )	k1++;
  		
		//---- Определяем температуру от датчика DS18b20, Команда всем датчикам для преобразования температуры ---------------
    	sensors.requestTemperatures();   
    	//-- отобразить температуру с каждого датчика
    	for (int i = 0;  i < deviceCount;  i++)    {
      	sensors.getAddress(t, i);   //получаем t =  HEX адрес 
      	//int id = sensors.getUserData(t);	//Serial.print(id); Serial.print(" ");      
      	tempC = sensors.getTempCByIndex(i);
      	Serial.print("L439");	printAddress(t); Serial.print(" ");   //печатаем t = HEX адрес
  			Serial.println(tempC);           
		}		
	}
	if( min1 >=59 ){
		min1=0;	hour1++;	
  		//Serial.begin(115200); Serial2.begin(19200);	Serial.println(Ver); 
	}
	if( hour1 >=23 ){
		hour1=0;	
	}
   
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// this is all you need make sure to call handleFTP() frequently
	ftpSrv.handleFTP();		//ftpSrv.begin(F("ftp"), F("ftp")); // логин пароль?	
	//
	// Code below just for debugging in Serial Monitor
	//
	if (action == show){
		Serial.printf_P(PSTR("Enter 'F' to format, 'L' to list the contents of the FS\n"));
		action = wait;
	}
	else if (action == wait){
		if (Serial.available()){
			char c = Serial.read();
			if (c == 'F')
			action = format;
			else if (c == 'L')
			action = list;
			else if (!(c == '\n' || c == '\r'))
			action = show;
		}
	}
	else if (action == format){
		uint32_t startTime = millis();
		SPIFFS.format();
		Serial.printf_P(PSTR("FS format done, took %lu ms!\n"), millis() - startTime);
		action = show;
	}
	else if (action == list){
		Serial.printf_P(PSTR("Listing contents...\n"));
		uint16_t dirCount = ListDir("/");
		Serial.printf_P(PSTR("%d files total\n"), dirCount);
		action = show;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//-- Server HTTP port 8080 --------------------------------------------------------
/*  	WiFiClient client1 = server1.available();	//EthernetClient client1 = server1.available();
  	if (client1) {		
		Serial.println();	Serial.print("L454:8080 ");
    	// an HTTP request ends with a blank line
    	bool currentLineIsBlank = true;
    	while (client1.connected()) {
      	if (client1.available()) {
	///////////////////////////////////////////////
        	char c = client1.read();
        	//Serial.write(c); // to serial

        	// if you've gotten to the end of the line (received a newline
        	// character) and the line is blank, the HTTP request has ended,so you can send a reply
        	if (c == '\n' && currentLineIsBlank) {
	///////////////////////////////////////////////////////////////////////////
          	//---- send a standard HTTP response header
          	client1.println("HTTP/1.1 200 OK");
          	client1.println("Content-Type: application/json");
          	client1.println("Connection: close");  // the connection will be closed after completion of the response
         	//client1.println("Refresh: 7");  		// refresh the page automatically every 5 sec          	
				client1.println();
          	client1.print("[");     
  				client1.print( a0, 2); client1.print(", "); 	client1.print( a1-a0, 2);	client1.print(", ");
  				client1.print( a2-a1, 2); client1.print(", ");	client1.print( a3-a2, 2);
				client1.print("]");		
				Serial.println(" L477:8080");	//client1.print("L307 ");	
	////////////////////////////////////////////////////////////////////////////////////////////////////
          	break;		//	while (client.connected())  !!!!!!!!!!!!!!!!!!
        }

        if (c == '\n') {
          	// you're starting a new line
          	currentLineIsBlank = true;
        } else if (c != '\r') {
          	// you've gotten a character on the current line
          	currentLineIsBlank = false;
        }
	////////////////////////////////////
      }
    }
    delay(1);
    client1.stop();     //Serial.println("client disconnected");
  }		*/

	//-- Server HTTP port 80 --------------------------------------------------------------
  	// listen for incoming clients 
  	WiFiClient client = server.available();	//EthernetClient client = server.available();
  	if (client) {	

		Serial.println();	Serial.print("L501  ");
		String header = "";
    	String currentLine = "";

    	while (client.connected()) {
      	if (client.available()) {	//Serial.println("L512");	

        	char c = client.read();		// печатает каждый знак из ввода 	
				//-- отладка OK ----------------------------
				//Serial.write(c);	// to serial  http запросы    
  
			header += c;

			if (header.indexOf("ico") >= 0)	{	//					if (header.indexOf("GET /favicon.ico/") >= 0)	{
				clsScreen=2;		// refresh web
				Serial.print("L516_RELOAD_html  ");
				// не убирать!
				break;	// не убирать!	while (client.connected())					
	    	}

			if(clsScreen >= 1) {	// -- одно кратная загрузка при включении -------------------
            	client.println(webpageCode);	// String webpageCode = R"***( //#include "webpageLi.h"
				clsScreen -- ;	Serial.print("L523  ");		//Serial.println(str_len);
          		break;		   						
			}
			//-- Основной цикл веб сервера ---------------------------------------------------------------------------------------
        	if (c == '\n')
        	{	

			//Serial.println("L569");	// печатаем до превода строки
			//Serial.print(header.c_str());
			//Serial.println("L569!");	// печатаем до превода строки

          	if (currentLine.length() == 0) 
          	{
				//Serial.println();	//Serial.print("L561 ");
				// -- START - GET / ----------------------------
				if (header.indexOf("GET /switch_mode/1") >= 0)
	            {
					chh.d16 = 12580; Serial.print("L537/switch_mode/1");
	            }
	            else if (header.indexOf("GET /switch_mode/2") >= 0)
	            {
					chh.d16 = 12836;	Serial.print("L541/switch_mode/2");
	            }
	            else if (header.indexOf("GET /switch_mode/3") >= 0)
	            {
					chh.d16 = 13092;	Serial.print("L545/switch_mode/3 ");
	            }
				else if (header.indexOf("GET /status/") >= 0)	{
					Serial.print("L548/status/  ");
	            }
				else if (header.indexOf("GET /sw_md/0") >= 0)	{	
					Serial.print(" L551/sw_md/0 "); 
					balMode =0;
				}				
				else if (header.indexOf("GET /sw_md/1") >= 0)	{	
					Serial.print(" L555/sw_md/1 "); 
					balMode =1;
				}		
				else if (header.indexOf("GET /file") >= 0)	{	//<a id='download' href='/file' download>Your Download</a>	
				//--- загрузка на браузер файла по запросу GET /file из клиента ----------------
					Serial.print(" L560/file   "); 
						
					/*	myFile = SPIFFS.open("/temp1820_com.bat");
						
					// -- Your Download  Запрос на файл -----------------------------------------------
					client.println("HTTP/1.1 200 OK");
          			client.println("Content-Disposition: attachment; filename='temp1820_com.bat'; filename*=UTF-8''temp1820_com.bat");
					client.println("Content-Type: text/plain");
					client.println("Accept-Ranges: bytes");
					client.println();	////////!!!!!!!!!!!!!!!!!!!!!!!!

  					char ddd;
					if (myFile && myFile.available()) {		

  						while ( (myFile.position() <= myFile.size() - 1) ) {		//while (myFile.available()) {	
  							//while ( (myFile.position() <= myFile.size() - 1) && ( tik1 <=1023 )) {		//while (myFile.available()) {									 
							ddd = myFile.read();
							client.print(ddd);	//Serial.print(ddd);		// send htm to browse								
							//client.print(myFile.read());		//Serial.print(myFile.read()); // send htm to browse String format = BAD!								
    					}
    					myFile.close();	
						Serial.println("   L581"); 	
 					} 
					else Serial.println("   L584 Err open File");	*/						
					break;
				}			
				// -- EnD - GET / ----------------------------
					
				Serial.print("--L620--");
          		client.println("HTTP/1.1 200 OK");
          		client.println("Content-Type: text/html");
          		client.println("Connection: close");  		// the connection will be closed after completion of the response
          		client.println();
          		client.println("<!DOCTYPE HTML>");  
	///////////////////////////////////////////////////////////////////////////////////////  	
				client.println("<html>");
				//-- Время  NTP ------------------------------------------------------
     			client.print("NTP time:"); client.print(dayStamp);  client.print(" ");	client.print(timeStamp);  
				//-- Время Up Time ------------------------------------------------------
				client.print("   Up Time ");client.print(hour1); client.print(": "); client.print(min1); client.print("."); client.print(sek1);	client.print(" Time is not run => reload page"); 
				client.println("<br />");

				if(chh.d16 == 12580) client.print(" CHECK     ");
				if(chh.d16 == 12836) {
					client.print(" BALANSE   ");	client.print("  BalansRunTime:");	client.print(k1); 
				}
				if(chh.d16 == 13092) client.print(" KEYSwitch ");
				if(chh.d16 == 12836 || chh.d16 == 13092 )client.println("  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
					
  				for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
    				client.println("<br />");	client.print(" ");	
					client.print("#");     client.print(current_ic+1,DEC);	// номер контролера 6804 , контроллер на 12 ячееек - 3 группы по 4 ячейки
					if(errNodeIsoSpi[current_ic]) client.print(" ErLnk6804"); //Serial.print(errNodeIsoSpi[current_ic]);	Serial.print(" ");
					//client.print(" erLnk6804:"); client.print(errNodeIsoSpi[current_ic]);	//client.print(" ");
    				
    				for(int i=0; i<12; i++) {
      					client.print(" ");  
						client.print(cell_codes[current_ic][i]*0.0001, 2);
    				}

   		 			client.print(" Сell_sum:");  	client.print(sta_codes[current_ic][0]*0.002, 1); client.print("v");
      				client.print(" TmC_6804:");  client.print((sta_codes[current_ic][1]/75.0)-273.0, 1 );
      				client.print(" an_v:");  	client.print(sta_codes[current_ic][2]*0.0001 , 1);
      				client.print(" dig_v:");  	client.println(sta_codes[current_ic][3]*0.0001 , 1); client.print(" "); 
  				}
    			client.println("<br />");

				client.print(" Min_cell:");  	client.print(MIN_CELL_V, 2); client.print("v");		// Minimum allowable cell voltage. Depends on battery chemistry.
				client.print(" Max_cell:");  	client.print(MAX_CELL_V, 2); client.print("v");		// Maximum allowable cell voltage. Depends on battery chemistry    				
				if(sek1 & 0x01) client.print(" ^^^^^");   
    			else         	client.print(" .....");    
				client.print("<br />");	client.print(" ");		

				client.print("  Balanse_Code:");
  				for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
					client.print(" (");	client.print(current_ic+1,DEC);	client.print(")=");	client.print(Node_6804[current_ic],HEX);
				}
				//-- Show Balans Cells  Serial.print(k1); balMode
				if(balMode) client.print("  Mode:Auto   ");		// автоматический режим  
				else 			client.print("  Mode:Manual ");	
				client.print("<br />");	client.print(" ");	

				//----- ADS1115 ---------------------------------------
  				client.print(" ADS1115: A0:");	client.print(a0, 2);
  				client.print(" A1:");	client.print(a1-a0, 2);
  				client.print(" A2:");	client.print(a2-a1, 2);
  				client.print(" A3:");	client.println(a3-a2, 2);
				//-- DS1820 -------------
				client.print("   DS1820:");
    			for (int i = 0;  i < deviceCount;  i++)    {
      				//sensors.getAddress(t, i);   //получаем t =  HEX адрес       
      				tempC = sensors.getTempCByIndex(i);
  					client.print(" "); client.println(tempC);           
				}
				client.print("<hr>"); 

				//--// Версия --------------------------------------------------
				//client.print(hour); client.print(": "); client.print(min); client.print("."); client.print(sek);	client.print(" Time is not run => reload page."); 
				client.print(Ver);		client.print("<hr>"); 
      			client.println("</html>");
	////////////////////////////////////////////////////////////////////////////////////////////////////
         		break;		//	while (client.connected())  !!!!!!!!!!!!!!!!!!
          	}	//if (currentLine.length() == 0) 
          	else		currentLine = "";
        }	//if (c == '\n')
        else if (c != '\r'){	currentLine += c;
       	}
		//---------------------------------------------------------------------------------------------
		}	//if (client.available())
    }		//while (client.connected())
    	
    	delay(1);			// give the web browser time to receive the data    		
    	client.stop();   	// close the connection://Serial.println("client disconnected");
		Serial.print("L700    ");	//	Serial.println();
  	}		//if (client)
}	//void loop() 
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//-- температура опрос датчиков ds1820 --------------------------------------------------------------
void printAddress(DeviceAddress deviceAddress){
  	for (uint8_t i = 0; i < 8; i++){
    	// zero pad the address if necessary
    	if (deviceAddress[i] < 16) Serial.print("0");
    	Serial.print(deviceAddress[i], HEX);	//Serial.print(".");
  	}
}
//----------------------------------------------------------------------------------------------

/*!***********************************
 \brief Initializes the configuration array
 **************************************/
void init_cfg0()
{
  for(int i = 0; i<TOTAL_IC;i++)
  {
    tx_cfg[i][0] = 0xFE;	// CFGR0
    tx_cfg[i][1] = 0x00; 	// CFGR1
    tx_cfg[i][2] = 0x00;	// CFGR2
    tx_cfg[i][3] = 0x00; 	// CFGR3
    tx_cfg[i][4] = 0x00;	// CFGR4
    tx_cfg[i][5] = 0x00;	// CFGR5
  } 
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the  LTC6804 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  //Serial.println("Read LTC6804 reg CFGR0-5 8bit L732");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print("ic");
    Serial.print(current_ic+1,DEC);
    Serial.print(" 0x");    serial_print_hex(rx_cfg[current_ic][0]);
    Serial.print(" 0x");    serial_print_hex(rx_cfg[current_ic][1]);
    Serial.print(" 0x");    serial_print_hex(rx_cfg[current_ic][2]);
    Serial.print(" 0x");    serial_print_hex(rx_cfg[current_ic][3]);
    Serial.print(" 0x");    serial_print_hex(rx_cfg[current_ic][4]);
    Serial.print(" 0x");    serial_print_hex(rx_cfg[current_ic][5]);

    Serial.print(" pec 0x");    serial_print_hex(rx_cfg[current_ic][6]);
    Serial.print(" 0x");    serial_print_hex(rx_cfg[current_ic][7]);
    Serial.println(); 
  }
}
//-- 27.12.2022 Состояние ключей = сам конфиг ----------------------------------------------
void print_rxconfig1()
{
	union ddd ddd1;

  	for (int current_ic=0; current_ic < TOTAL_IC; current_ic++){
   		//Serial.print(rx_cfg[current_ic][0],HEX); Serial.print(" ");	
		Serial.print("R0 ");	
  		for (int ii=7 ; ii > 2; ii--){	// R0/7-3 bit= Gpio5-1  		 
			if( (rx_cfg[current_ic][0] >> ii) & 0x01) Serial.print("1 ");
			else Serial.print("0 ");
		}	

		Serial.print(".");
  		for (int ii=2 ; ii >= 0; ii--){  	// R0/2-0 refon,swtrd,adcort 	 
			if( (rx_cfg[current_ic][0] >> ii) & 0x01) Serial.print("1 ");
			else Serial.print("0 ");
		}
		//-- Match VOV = Volt OverVoltage
		ddd1.d16 = rx_cfg[current_ic][3]; 
		ddd1.d16 = ( (ddd1.d16 << 4) & 0x0FF0);
		ddd1.d16 |=( (rx_cfg[current_ic][2] >> 4) & 0x0F);
   		Serial.print(ddd1.d16,DEC); Serial.print(" ");	
		//Serial.print("VUV=");	
		Serial.print(" VOV=");	Serial.print( (ddd1.d16)*16.0*10e-5, 2);

		ddd1.d16 = rx_cfg[current_ic][5] & 0xF000; 
		ddd1.d16 = ( (ddd1.d16 << 8) & 0x0F00);

		ddd1.d16 |= rx_cfg[current_ic][4];
   		Serial.print(ddd1.d16,DEC); Serial.print(" ");	Serial.print("R4,R5 ");
  		for (int ii=12 ; ii >= 0; ii--){  		 
			if( (ddd1.d16 >> ii) & 0x01) Serial.print("1 ");
			else Serial.print("0 ");
		}	
		Serial.println(); 
	}
}
//------------------------------------------------
void serial_print_hex(uint8_t data)
{
    if (data< 16)
    {
      Serial.print("0");
      Serial.print((byte)data,HEX);
    }
    else	Serial.print((byte)data,HEX);
}
//-- 20.12.2022----------------------------------------------
// 7 6 5 4 3 2 1 0 
void print_bit8(uint8_t data)
{
  	for (int ii=7 ; ii >= 0; ii--){  		//Serial.print(" "); 
		Serial.print( (data >> ii) & 0x01); 
	}	
}
//-- 30.01.2023---------------------------------------------- 
void print_bit16(uint16_t data)
{
  	for (int ii=15 ; ii >= 0; ii--){  		//Serial.print(" "); 
		Serial.print( (data >> ii) & 0x01); 
	}	
}
//---------------------------------------------------
void measurementsCell() {

	int8_t err = 0;	

	Serial.print(" L999_M1 ");	//Serial.println();	

	LTC6804_initialize();	
	init_cfg0();		// initialize configuration array(memory) to be written  //coment: tx_cfg[i][0] = 0xFE; ......	
  	
	//-- Измерять необходимо на холостом ходу когда конфиг=0 --------------------
	wakeup_sleep();	// Start wr CFG					
	LTC6804_wrcfg(TOTAL_IC,tx_cfg);	delay(5);    
	wakeup_sleep();	// Start ADC					
	LTC6804_adcv();	delay(5);   LTC6804_ADSTAT();	delay(5);  	LTC6804_adax();	delay(5);

	//-- Read the raw data from the LTC6804 Cell voltage register ------------	
	// if (Reg != 0) Read single cell voltage register for all ICs in daisy chain reg == 0 => Читать все регистры = 12 ячеек 
	wakeup_sleep();	// Start Read Cell								 
	err += LTC6804_rdcv(0, TOTAL_IC,cell_codes, errNodeIsoSpi);	//0=все регистры		

	//-- Reads and parses the LTC6804 Auxiliary registers			
	// reg == 0 => Читать все регистры = Auxiliary registers	аналоговые входы GPIO 1-5
	wakeup_sleep();					
	LTC6804_rdaux(0,TOTAL_IC,aux_codes); 	//0=все регистры // Set to read back all Aux registers

	//--
	wakeup_sleep();	// Start Read STAT								 
	LTC6804_rdstat(0,TOTAL_IC,sta_codes); 	//0=все регистры	// Set to read back all Sta registers    

	//--
	wakeup_sleep();	// Start Read Config					
	err += LTC6804_rdcfg(TOTAL_IC,rx_cfg, errNodeIsoSpi);
	//Serial.print(err); Serial.print(" ");
}
//--26.12.22 -------------------------------------
void Set_BalanceVoltCel() {

	if(balMode == 0) {
		if(k1 < 4) {
  			LTC6804_initialize(); 	// SPI  f=125 кгц, Главная инициализация SPI 
			wakeup_sleep();			// wr CFG					
			LTC6804_wrcfg(TOTAL_IC,tx_cfg);	//delay(5);  
			Serial.print(" L1044_M2-0 ");  
		}
		else if(k1 >= 4) {
			chh.d16 = 12580;
		}
	}

	if(balMode == 1) {
		Serial.print(" L1052_M2-1 "); 
	} 
}

//--- 10.01.2023 Бегущий св 27.03.2023. Ручное управление ключами ---------------------
void keySwCell() {
	if( sek1 & 0x01 ) return;	
	
	Serial.println();	Serial.print(" L1055_M3  ");
	if(k2 >= 12) k2=0;	k2++;				

	for (int current_ic = 0 ; current_ic < TOTAL_IC ; current_ic++ ){
		Serial.print(" #");	Serial.print(current_ic+1,DEC);	 
		 
		int16_t ss1=0;	
		//-- Флаг на вЫключение -----------------
		Node_6804[current_ic] = 0;			 								
		//-- Флаг на вКлючение -----------------
		ss1 = (0x01) << k2;	Node_6804[current_ic] |= ss1; 
		/*                    12 11 10 9                        8 7 6 5 4 3 2 1
		tx_cfg[current_ic][5]:3  2  1  0  tx_cfg[current_ic][4]:8 7 6 5 4 3 2 1	*/	
		//-- Формирование кода для передачи в 68041 -----
		tx_cfg[current_ic][4] = Node_6804[current_ic];	tx_cfg[current_ic][5] = ( (Node_6804[current_ic]) >> 8 );
		Serial.print(" tx_cfg4:"); 	Serial.print(tx_cfg[current_ic][4]);	Serial.print(" tx_cfg5:");	Serial.print(tx_cfg[current_ic][5]);
	}

	LTC6804_initialize(); 	//SPI.setFrequency(125000); 
	wakeup_sleep();								
	LTC6804_wrcfg(TOTAL_IC,tx_cfg);	// wr CFG	//delay(5);  
	//if( errNodeIsoSpi[current_ic] ) Serial.print(":ErLnk_6804");			

	Serial.println("L1079  ");
}
//-- FTP 11.05.2023---------------------
uint16_t ListDir(const char *path)
{
  uint16_t dirCount = 0;
  File root = SPIFFS.open(path);
  if (!root)
  {
    Serial.println(F("failed to open root"));
    return 0;
  }
  if (!root.isDirectory())
  {
    Serial.println(F("/ not a directory"));
    return 0;
  }

  File file = root.openNextFile();
  while (file)
  {
    ++dirCount;
    if (file.isDirectory())
    {
      Serial.print(F("  DIR : "));
      Serial.println(file.name());
      dirCount += ListDir(file.name());
    }
    else
    {
      Serial.print(F("  FILE: "));
      Serial.print(file.name());
      Serial.print(F("\tSIZE: "));
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  return dirCount;
}
//-- END -----------------------------------------------------------------------------
