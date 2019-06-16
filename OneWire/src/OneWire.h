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
#define OWPin (1 << 7)
#define OWLow GPIOD->BSRRH = OWPin
#define OWHigh GPIOD->BSRRL = OWPin

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
#define SKIP_ROM 0xCC // Jezeli na linii znajduje sie tylko jeden slave mozna pominac wybieranie jego adresu

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
	void (*OWReadEnd)(const uint8_t * readResults, const uint8_t len, const uint8_t ROMCmd, const uint8_t DeviceCMD); // Wskaznik do funkcji wywolywanej, gdy wszystkie dane zostaly odczytane
	void (*OWWriteEnd)(const uint8_t ROMCmd, const uint8_t DeviceCMD);
	void (*OWNoSlaves)(); // Wskaznik do funkcji wywolywanej, gdy zaden uklad podrzedny nie zostaly znaleziony
	uint8_t OWState; // Obecny stan w jakim znajduje sie automat
	uint8_t OWCommand; // Obecny tryb pracy zapis / odczyt / czekanie
	uint8_t OWROMCmd, OWDeviceCmd;
	uint8_t OWWriteID;
	uint8_t OWReadID;
	uint8_t OWShiftedBit;
	uint8_t OWWriteBuffer[16];
	uint8_t OWReadBuffer[16];
	uint8_t OWWriteDataLen;
	uint8_t OWReadDataLen;
} OneWire;

void OWInit(OneWire * OWire, void (*OWReadCallback)(const uint8_t *,  const uint8_t, const uint8_t, const uint8_t), void (*OWNoSlavesCallback)(), void (*OWWriteEnd)(const uint8_t ROMCmd, const uint8_t DeviceCMD));

void OWResetCom(OneWire * OWire);

void OWStartCom(OneWire * OWire);

void OWStopCom(OneWire * OWire);

uint8_t CRC_8(const uint8_t * data, uint8_t len, uint8_t _CRC);


#endif /* ONEWIRE_H_ */
