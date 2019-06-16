/*
 * OneWire.c
 *
 *  Created on: 13.06.2019
 *      Author: i5
 */

#include "OneWire.h"

uint16_t OWCycleCnt = 0x00;

OneWire * OWirePt = 0;

void OWInit(OneWire * OWire, void (*OWReadCallback)(const uint8_t *, const uint8_t, const uint8_t, const uint8_t), void (*OWNoSlavesCallback)(), void (*OWWriteEndCallback)(const uint8_t, const uint8_t)) {
	OWirePt = OWire;
	OWirePt->OWReadEnd = OWReadCallback;
	OWirePt->OWNoSlaves = OWNoSlavesCallback;
	OWirePt->OWWriteEnd = OWWriteEndCallback;

	GPIOD->BSRRL = OWPin;
	GPIOD->MODER |= GPIO_MODER_MODER7_0; // Wyjscie
	GPIOD->OTYPER |= GPIO_OTYPER_ODR_7; // Otwarty dren
	GPIOD->OSPEEDR = GPIO_OSPEEDER_OSPEEDR7; // Ustaw najwyzsza "czestotliwosc" taktowania pinu 7 (100 MHz)

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Uruchom zegar dla portu GPIOD
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Podepnij zegar pod TIM1

	TIM1->PSC = 21; // Czestotliwosc bazowa 84 MHz, APB2 taktowane jest polowa czestotliwosci HSE PLL
	TIM1->ARR = 3;
	TIM1->DIER = TIM_DIER_UIE;

	OWResetCom(OWire);
}

void OWResetCom(OneWire * OWire) {
	OWire->OWState = OWIdle;
	OWire->OWCommand = OWHold;
	OWire->OWWriteID = 0;
	OWire->OWReadID = 0;
	OWire->OWShiftedBit = 0;
	OWire->OWReadDataLen = 0;
	OWire->OWWriteDataLen = 0;
	OWire->OWDeviceCmd = 0;
}

void OWStartCom(OneWire * OWire) {
	OWire->OWState = OWIdle;
	OWire->OWCommand = OWWrite;
	OWire->OWROMCmd = OWire->OWWriteBuffer[0];

	TIM1->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

void OWStopCom(OneWire * OWire) {
	OWResetCom(OWire);
	TIM1->CR1 &= ~TIM_CR1_CEN;

	NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
}

uint8_t dataCRC = 0;

uint8_t CRC_8(const uint8_t * data, uint8_t len, uint8_t _CRC) {
	uint8_t dataCRC = 0, nextByte, mixByte;

	while(len--) {
		nextByte = *data++;

		for (uint8_t i = 8; i; i--) {
			mixByte = (dataCRC ^ nextByte) & 0x01;
			dataCRC >>= 1;

			if (mixByte) dataCRC ^= 0x8C; // PDF 8

			nextByte >>= 1;
		}
	}

	return _CRC == dataCRC;
}

void TIM1_UP_TIM10_IRQHandler(void){
	if (TIM1->SR & TIM_SR_UIF) { // Flaga przerwania jest ustawiona
	    switch(OWirePt->OWState) {
			case OWIdle: {
				OWirePt->OWState = OWStart;

				OWLow;
			} break;
			case OWStart: {
				if (OWCycleCnt >= TIM_H) {
					OWirePt->OWState = OWPresencePulse;
					OWCycleCnt = 0;

					OWHigh; // Wyzeruj stan wejscia do stanu poczatkowego
				} else OWCycleCnt++;
			} break;
			case OWPresencePulse: { // Czekamy na odpowiedz od przynajmniej jednego ukladu podrzednego
				if (OWCycleCnt >= TIM_I) {
					if (!(GPIOD->IDR & GPIO_IDR_IDR_7)) {
						OWirePt->OWState = OWMasterSample; // Przynajmniej jeden uklad podrzedny znajduje sie na linii, przechodzimy dalej
					} else {
						OWStopCom(OWirePt);

						OWirePt->OWNoSlaves();
					}
					OWCycleCnt = 0;
				} else OWCycleCnt++;
			} break;
			case OWMasterSample: {
				if (OWCycleCnt >= TIM_J) {
					OWCycleCnt = 0;

					OWirePt->OWState = OWMasterOperation;
				} else OWCycleCnt++;
			} break;
			case OWMasterOperation: {
				if (OWirePt->OWCommand == OWWrite){
					if (OWirePt->OWWriteDataLen > 0) {
						if (OWirePt->OWShiftedBit == 8) {
							if (OWirePt->OWWriteID < OWirePt->OWWriteDataLen - 1) {
								OWirePt->OWWriteID++; // Kolejna komenda z tablicy
								OWirePt->OWShiftedBit = 0; // Kolejny bajt do wyslania, zerujemy licznik przesuwajacy bity do wyslania
							}
							else { // Wszystkie bajty z tablicy zostaly wyslane
								if (OWirePt->OWReadDataLen > 0){
									OWirePt->OWCommand = OWRead;
									OWirePt->OWState = OWReadBitStart;
									OWirePt->OWShiftedBit = 0;

									OWirePt->OWWriteEnd(OWirePt->OWROMCmd, OWirePt->OWDeviceCmd);

									break;
								} else {
									OWirePt->OWWriteEnd(OWirePt->OWROMCmd, OWirePt->OWDeviceCmd);

									OWStopCom(OWirePt);
									break;
								}
							}
						}

						uint8_t bitValue = OWirePt->OWWriteBuffer[OWirePt->OWWriteID] >> OWirePt->OWShiftedBit++;
						if (bitValue & 0x01) OWirePt->OWState = OWWriteBit1;
						else OWirePt->OWState = OWWriteBit0;
					}
				} else if (OWirePt->OWCommand == OWRead) {
					if (OWCycleCnt >= TIM_F) {
						OWCycleCnt = 0;
						if (OWirePt->OWShiftedBit == 8) {
							if (OWirePt->OWReadID < OWirePt->OWReadDataLen - 1) {
								OWirePt->OWReadID++;
								OWirePt->OWShiftedBit = 0;
							} else {
								OWHigh;
								OWirePt->OWReadEnd(OWirePt->OWReadBuffer, OWirePt->OWReadDataLen, OWirePt->OWROMCmd, OWirePt->OWDeviceCmd);

								OWStopCom(OWirePt);

								break;
							}
						}

						OWirePt->OWState = OWReadBitStart;
					} else OWCycleCnt++;
				}
			} break;
			case OWWriteBit0: {
				OWLow; // Wystaw stan niski na pin 1Wire
				if (OWCycleCnt >= TIM_C) {
					OWCycleCnt = 0;

					OWHigh; // Powrot do stanu wysokiego

					OWirePt->OWState = OWWriteBit0_H;
				} else OWCycleCnt++;
			} break;
			case OWWriteBit0_H: {
				if (OWCycleCnt >= TIM_D) {
					OWCycleCnt = 0;

					OWirePt->OWState = OWMasterOperation;
				} else OWCycleCnt++;
			} break;
			case OWWriteBit1: {
				OWLow; // Wystaw stan niski na pin 1Wire
				if (OWCycleCnt >= TIM_A) {
					OWCycleCnt = 0;

					OWirePt->OWState = OWWriteBit1_H;

					OWHigh; // Powrot do stanu wysokiego
				} else OWCycleCnt++;
			} break;
			case OWWriteBit1_H: {
				if (OWCycleCnt >= TIM_B) { //Nalezy odczekac, az uklad podrzedny odczyta podany stan jako wysoki
					OWCycleCnt = 0;

					OWirePt->OWState = OWMasterOperation;
				} else OWCycleCnt++;
			} break;
			case OWReadBitStart: {
				OWLow;

				if (OWCycleCnt >= TIM_E) {
					OWHigh;
					OWirePt->OWState = OWReadBitEnd;

					OWCycleCnt = 0;
				} else OWCycleCnt++;
			} break;
			case OWReadBitEnd: {
				if (OWCycleCnt >= TIM_D) {

					if (GPIOD->IDR & GPIO_IDR_IDR_7) {
						OWirePt->OWReadBuffer[OWirePt->OWReadID] |= 0x01 << OWirePt->OWShiftedBit++; // Odczytana zostala jedynka
					} else OWirePt->OWReadBuffer[OWirePt->OWReadID] &= ~(0x01 << OWirePt->OWShiftedBit++); // Zero, mozna by pominac zapisywanie 0, czyszczac bufor odczytanym danych za kazdym uruchomieniem komunikacji

					OWCycleCnt = 0;

					OWirePt->OWState = OWMasterOperation;
				} else OWCycleCnt++;
			} break;
			default: {
				OWStopCom(OWirePt);

				OWHigh;
			} break;
	    }

	    TIM1->SR &= ~(TIM_SR_UIF); // Przerwanie zostalo obsluzone, wyczysc flage oczekujacego przerwania
	}
}
