/*
 * DS18B20.h
 *
 *  Created on: 14.06.2019
 *      Author: i5
 */

#ifndef DS18B20_H_
#define DS18B20_H_

#include "../inc/stm32f4xx.h"

#define TEMP_CONV 0x44 // Zleca DS18B20 konwersje temperatury i zapisanie jej do scratchpada
#define SCRATCHPAD_READ 0xBE // Odczytaj temperature z rejestru, dla 12 bitow nalezy czekac 750 ms
#define SCRATCHPAD_WRITE 0x4E // Zapis do rejestru
#define SCRATCHPAD_COPY 0x48 // Pozwala na skopiowanie obecnych ustawien scratpadu (Temperatury alarmu i rozdzielczosc) do pamieci EEPROM
#define TEMP_9BIT 0x1F // Wartosc rejestru ustawiajacego rozdzielczosc na 9 bitow
#define TEMP_10BIT 0x3F
#define TEMP_11BIT 0x5F
#define TEMP_12BIT 0x7F
#define T_CONV_12BIT 600 // ms, normalnie powinno byc 750 ms
#define T_CONV_11BIT 375 // ms
#define T_CONV_10BIT 150 // ms
#define T_CONV_9BIT 50 // ms, MAX 95 ms

typedef struct {
	uint8_t DSAddress[8]; // Zamiast 8 bajtow mozna dac byc 6 bajtow, 0 bajt to zawsze bajt rodzinny (0x28), a bajt 7 to CRC bajtu rodzinny i kodu ROM
	float DSTemperature;

	uint8_t DSResponding;
} DS18B20;

typedef enum {
	DSIdle = 0,
	DSConvertTemp = 1,
	DSReadTemp = 2,
	DSWaitAndConvert = 3,
	DSWaitAndRead = 4,
	DSSetResolution = 5,
	DSWriteEEPROM = 6
} DS18B20Routine;

void DSInitSensors(DS18B20 * TempSensors, uint8_t SensorsCnt);
void DSGetROMCode(void (*ReadCallback)(const uint8_t * ROMCode));
void DSStartMeasurement(void);
void DSSetSensorsResolution(uint8_t resolution);
void DSUpdateResolution(void);

#endif /* DS18B20_H_ */
