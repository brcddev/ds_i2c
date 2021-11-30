#include <Arduino.h>

#include <Wire.h>
#include "ds_def.h"
typedef uint8_t DeviceAddress[8];
typedef uint8_t ScratchPad[9];
#include <OneWire.h>

#define START_DATA 10

#define ONEWIREPIN   7		/* OneWire bus on digital pin 7 */
#define MAXSENSORS   10		/* Maximum number of sensors on OneWire bus */

// Model IDs
#define DS18S20      0x10
#define DS18B20      0x28
#define DS1822       0x22

// OneWire commands
#define CONVERT_T       0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

typedef struct {
  DeviceAddress addr;
  uint8_t	ds_type;  
	uint8_t	id;
}ds_type;
uint8_t flagDallRead;

int HighByte, LowByte, TReading, SignBit, Tc_100;
byte i, j, sensors;
byte present = 0;
boolean ready;
int dt;
byte data[12];
byte addr[8];
OneWire ds(ONEWIREPIN);		// DS18S20 Temperature chip i/o
ds_type DS[MAXSENSORS];		/* array of digital sensors */
float DStemp[MAXSENSORS];


void find_ds(){
  uint8_t count_valid_ds=0;
  ds_type ee_ds[MAXSENSORS];
  uint8_t valid_ds[MAXSENSORS];
  Serial.println("Load sensors...");
  eeprom_read_block((void*)&ee_ds, (const void*)START_DATA, sizeof(ee_ds));
  for (uint8_t i=0;i<MAXSENSORS;i++){
    if ( OneWire::crc8( ee_ds[i].addr, 7) == ee_ds[i].addr[7]) {
      valid_ds[i]=1;
      count_valid_ds++;
   }
   else valid_ds[i]=0;
  } 
  ds_type tmp;
  for (int i = MAXSENSORS - 1; i >= 0; i--)
  {
    for (int j = 0; j < i; j++)
    {
      if (valid_ds[j] < valid_ds[j + 1])
      {
        tmp = ee_ds[j];
        ee_ds[j] = ee_ds[j + 1];
        ee_ds[j + 1] = tmp;
      }
    }
  }
  
  Serial.println("Searching for sensors...");
  sensors = 0;
  while ( ds.search(addr)) {
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      break;
    }
    /*
    for( i = 0; i < 8; i++) {
      DS[sensors].addr[i] = addr[i];
      Serial.print(addr[i], HEX);
      Serial.print(" ");
    }
    */
    memcpy(DS[sensors].addr,addr,sizeof(DS[sensors].addr));
    for(uint8_t i=0;i<count_valid_ds;i++){
      //
    }
    sensors++;
  }  
}

void dallRead(unsigned long interval){
  static unsigned long prevTime = 0;
  if (millis() - prevTime > interval) { //Проверка заданного интервала
  static boolean flagDall = 0; //Признак операции
  prevTime = millis();
  flagDall =! flagDall; //Инверсия признака
  if (flagDall) {
    ds.reset();
    ds.write(SKIPROM); //Обращение ко всем датчикам
    ds.write(STARTCONVO); //Команда на конвертацию
    flagDallRead = 1; //Время возврата в секундах
  }
  else {
    byte i;
     int temp;
    for (i = 0; i < sensors; i++){ //Перебор количества датчиков
     ds.reset();
     ds.select(DS[i].addr);
     ds.write(0xBE); //Считывание значения с датчика
     temp = (ds.read() | ds.read()<<8); //Принимаем два байта температуры
     DStemp[i] = (float)temp / 16.0; 
     flagDallRead = 2; //Время возврата в секундах
     }
   }
  }
}



void setup(void) {
  // initialize inputs/outputs
  // start serial port
  Serial.begin(115200);
  find_ds();
  

  Serial.print(sensors,DEC);
  Serial.print(" sensors found");
  Serial.println();
}

void get_ds(int sensors) {		/* read sensor data */

  for (i=0; i<sensors; i++) {
    ds.reset();
    ds.select(DS[i].addr);
    ds.write(CONVERT_T);	// start conversion, with parasite power off at the end

    if (0) {
      dt = 75;
      delay(750);      
    } else {
      ready = false;
      dt = 0;
      delay(10);
      while (!ready && dt<75) {	
	delay(10);
	ready = ds.read_bit();
	dt++;
      }

    present = ds.reset();
    ds.select(DS[i].addr);    
    ds.write(READSCRATCH);         // Read Scratchpad
  
    for ( j = 0; j < 9; j++) {           // we need 9 bytes
      data[j] = ds.read();
    }

    /* check for valid data */
    if ( (data[7] == 0x10) || (OneWire::crc8( addr, 8) != addr[8]) ) {
      LowByte = data[0];
      HighByte = data[1];
      TReading = (HighByte << 8) + LowByte;
      SignBit = TReading & 0x8000;  // test most sig bit
      if (SignBit) // negative
	{
	  TReading = (TReading ^ 0xffff) + 1; // 2's comp
	}
      if (DS[i].addr[0] == DS18B20) { /* DS18B20 0.0625 deg resolution */
	Tc_100 = (6 * TReading) + TReading / 4; // multiply by (100 * 0.0625) or 6.25
      }
      else if ( DS[i].addr[0] == DS18S20) { /* DS18S20 0.5 deg resolution */
	Tc_100 = (TReading*100/2);
      }


      if (SignBit) {
	DStemp[i] = - (float) Tc_100 / 100;
      } else {
	DStemp[i] = (float) Tc_100 / 100;
      }
    } else {		 /* invalid data (e.g. disconnected sensor) */
      DStemp[i] = NAN;
    }
  }
}

void loop(void) {
  dallRead(flagDallRead*1000);
  get_ds(sensors);
  for (i=0; i<sensors; i++) {
    if (isnan(DS[i].temp)) {
      Serial.print("NaN");
    } else {
      Serial.print(DS[i].temp,2);
    }
    if (i<sensors-1) {
      Serial.print(",");
    }
  }
  Serial.println();
  delay(5000);
}


//***Функция считывания температуры c Далласов*****


