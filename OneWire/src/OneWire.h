/*
 * OneWire.h
 *
 *  Created on: 12.06.2019
 *      Author: i5
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#include "../inc/stm32f4xx.h"

#define CPU_FREQ 84000000;
#define OWPort GPIOD;
#define OWPin (1 << 7);
#define OWLow
#define OWHigh

/*
 * Timingi dla 1-Wire https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
 */
#define TIM_A 6
#define TIM_B 64
#define TIM_C 60
#define TIM_D 10
#define TIM_E 9
#define TIM_F 55
#define TIM_G 0
#define TIM_H 480
#define TIM_I 70
#define TIM_J 410

#define READ_ROM 0x33 // Jezeli na linii jest jeden slave to READ_ROM pozwala na poznanie ID jego rodzinny, kodu ROM oraz kodu CRC
#define SELECT_SLAVE 0x55 // Pozwala wybrac slave'a na linii podajac najpierw ID rodzinny (DS18B20 0x28), kod ROM i CRC
#define TEMP_CONV 0x44 // Konwertuj temperature
#define SCRATCHPAD_READ 0xBE // Odczytaj temperature z rejestru, dla 12 bitow trzeba czekac 750 ms
#define SCRATCHPAD_WRITE 0x4E // Zapis do rejestru

typedef enum {
	OWIdle = 0,
	OWStart = 1,
	OWPresencePulse = 2,
	OWMasterSample = 3,
	OWMasterOperation = 4,
	OWWriteBit0 = 5,
	OWWriteBit0_H = 6,
	OWWriteBit1 = 7,
	OWWriteBit1_H = 8,
	OWReadBitStart = 9,
	OWReadBitEnd = 10,
} OneWireState;

typedef enum {
	OWHold = 0,
	OWRead = 1,
	OWWrite = 2,
} OneWireOperation;

typedef struct {
	void (*OWReadEnd)(const uint8_t * readResults, uint8_t len); // Wskaznik do funkcji wywolywanej, gdy wszystkie dane zostaly odczytane
	void (*OWNoSlaves)(); // Wskaznik do funkcji wywolywanej, gdy zaden uklad podrzedny nie zostaly znaleziony
	uint8_t OWState; // Obecny stan w jakim znajduje sie automat
	uint8_t OWCommand; // Obecny tryb pracy zapis / odczyt / czekanie
	uint8_t OWSlaveID; // Obecnie obslugiwany slave
	uint8_t OWWriteID;
	uint8_t OWReadID;
	uint8_t OWShiftedBit;
	uint8_t OWWriteBuffer[16];
	uint8_t OWReadBuffer[16];
	uint8_t OWWriteDataLen;
	uint8_t OWReadDataLen;
} OneWire;

void OWInit(OneWire * OWire, void (*OWReadCallback)(const uint8_t *, uint8_t), void (*OWNoSlavesCallback)());

void OWResetCom(OneWire * OWire);

void OWStartCom(OneWire * OWire);

void OWStopCom(OneWire * OWire);

uint8_t CRC_8(const uint8_t * data, uint8_t len, uint8_t _CRC);


#endif /* ONEWIRE_H_ */
