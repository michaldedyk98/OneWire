/*
 * DS18B20.c
 *
 *  Created on: 14.06.2019
 *      Author: i5
 */

#include "DS18B20.h"
#include "OneWire.h"

void OWireReadEnd(const uint8_t * data, const uint8_t len, const uint8_t ROMCmd, const uint8_t DeviceCMD);
void OWireWriteEnd(const uint8_t ROMCmd, const uint8_t DeviceCMD);
void OWireNoSlaves();

void (*ROMCodeCallback)(const uint8_t *) = 0;

OneWire OWire = { .OWReadEnd = 0, .OWNoSlaves = 0, .OWWriteEnd = 0 };
DS18B20 * DSTempSensors = 0;

uint16_t DSCycleCnt = 0;
uint8_t DSResolution = TEMP_9BIT;
uint8_t DSConversionTime = 0;
uint8_t DSSensorsCnt = 0;
uint8_t DSSkip = 0;
uint8_t DSActiveSensor = 0;
uint8_t DSState = DSIdle;

void DSInitSensors(DS18B20 * TempSensors, uint8_t SensorsCnt) {
	OWInit(&OWire, OWireReadEnd, OWireNoSlaves, OWireWriteEnd);

	SysTick_Config(84000); // Przerwanie co 1 ms

	DSSensorsCnt = SensorsCnt;
	DSTempSensors = TempSensors;
	DSConversionTime = DSSensorsCnt * 6;
}

void DSStartMeasurement(void) {
	DSState = DSConvertTemp;

	DSCycleCnt = 0;
	DSActiveSensor = 0;
	DSSkip = 0;
}

void DSSetSensorsResolution(uint8_t resolution) {
	DSResolution = resolution;

	DSUpdateResolution();
}

void DSUpdateResolution(void) {
	OWire.OWWriteBuffer[0] = SELECT_SLAVE;
	for (uint8_t i = 0; i < 8; i++)
		OWire.OWWriteBuffer[i+1] = DSTempSensors[DSActiveSensor].DSAddress[i];
	OWire.OWWriteBuffer[9] = SCRATCHPAD_WRITE;
	OWire.OWWriteBuffer[10] = 0x00;
	OWire.OWWriteBuffer[11] = 0x00;
	OWire.OWWriteBuffer[12] = DSResolution;

	OWire.OWDeviceCmd = SCRATCHPAD_WRITE;
	OWire.OWWriteDataLen = 13;
	OWire.OWReadDataLen = 0;

	OWStartCom(&OWire);
}

void DSGetROMCode(void (*ReadCallback)(const uint8_t * ROMCode)) {
	ROMCodeCallback = ReadCallback;

	OWire.OWWriteBuffer[0] = READ_ROM;
	OWire.OWWriteDataLen = 1;
	OWire.OWReadDataLen = 8;

	OWStartCom(&OWire);
}

void SysTick_Handler(void) {
	switch (DSState) {
		case DSConvertTemp: {
			OWire.OWWriteBuffer[0] = SELECT_SLAVE;
			for (uint8_t i = 0; i < 8; i++)
				OWire.OWWriteBuffer[i+1] = DSTempSensors[DSActiveSensor].DSAddress[i];
			OWire.OWWriteBuffer[9] = TEMP_CONV;

			OWire.OWDeviceCmd = TEMP_CONV;
			OWire.OWWriteDataLen = 10;
			OWire.OWReadDataLen = 0;

			OWStartCom(&OWire);

			DSCycleCnt = 0;
			DSState = DSIdle;
		} break;
		case DSReadTemp: {
			if (DSSkip || DSCycleCnt >= T_CONV_9BIT - DSConversionTime) {
				OWire.OWWriteBuffer[0] = SELECT_SLAVE;
				for (uint8_t i = 0; i < 8; i++)
					OWire.OWWriteBuffer[i+1] = DSTempSensors[DSActiveSensor].DSAddress[i];
				OWire.OWWriteBuffer[9] = SCRATCHPAD_READ; // Odczytanie temperatury z rejestru

				OWire.OWDeviceCmd = SCRATCHPAD_READ;
				OWire.OWWriteDataLen = 10; // Wysylamy 10 bajtow, 1 bajt to komenda 1Wire, 8 bajtow to kod ROM, ostatnia komenda to odczyt temperatury.
				OWire.OWReadDataLen = 9; // Odbiermy 9 bajtow, zamiast 9 mozna odebrac tylko 2 z temperatura, ale nie bedzie wtedy mozliwosci sprawdzenia CRC

				OWStartCom(&OWire);

				DSSkip = 1;
				DSCycleCnt = 0;
				DSState = DSWaitAndRead;
			}
		} break;
		case DSWaitAndConvert: {
			if (DSCycleCnt > TIM_A) {
				DSState = DSConvertTemp;

				DSCycleCnt = 0;
			}
		} break;
		case DSWaitAndRead: {
			if (DSCycleCnt > 20) {
				DSState = DSReadTemp;

				DSCycleCnt = 0;
			}
		} break;
		case DSSetResolution: {
			if (DSCycleCnt > TIM_A) {
				DSUpdateResolution();

				DSState = DSIdle;
			}
		} break;
		case DSWriteEEPROM: {
			if (DSCycleCnt > TIM_A) {
				OWire.OWWriteBuffer[0] = SELECT_SLAVE;
				for (uint8_t i = 0; i < 8; i++)
					OWire.OWWriteBuffer[i+1] = DSTempSensors[DSActiveSensor].DSAddress[i];
				OWire.OWWriteBuffer[9] = SCRATCHPAD_COPY;

				OWire.OWDeviceCmd = SCRATCHPAD_COPY;
				OWire.OWWriteDataLen = 10;
				OWire.OWReadDataLen = 0;

				OWStartCom(&OWire);

				DSState = DSIdle;
			}
		} break;
		case DSIdle: break;
	}
	DSCycleCnt++;
}

void OWireReadEnd(const uint8_t * data, const uint8_t len, const uint8_t ROMCmd, const uint8_t DeviceCMD) {
	/*
	 * data[0] - LSB temperatury;
	 * data[1] - MSB temperatury;
	 */
	if (DeviceCMD == SCRATCHPAD_READ) {
		if (CRC_8(data, 8, data[8])) { // 8 bajt to CRC
			int16_t temperatureData = (data[1] << 8) + data[0];
			DSTempSensors[DSActiveSensor].DSTemperature = temperatureData * 0.0625;

			GPIOD->ODR ^= (1 << 13);
		} else GPIOD->ODR ^= (1 << 14);

		DSActiveSensor++;

		if (DSActiveSensor == DSSensorsCnt) {
			DSActiveSensor = 0;
			DSSkip = 0;
			DSState = DSWaitAndConvert;

			GPIOD->ODR ^= (1 << 12);
		}
	} else if (ROMCmd == READ_ROM) {
		ROMCodeCallback(data);
	}
}

void OWireWriteEnd(const uint8_t ROMCmd, const uint8_t DeviceCmd) {
	switch(DeviceCmd) {
	case TEMP_CONV: {
			GPIOD->ODR ^= (1 << 15);

			if (DSActiveSensor == DSSensorsCnt - 1) {
				DSState = DSWaitAndRead;
				DSActiveSensor = 0;
			} else {
				DSState = DSWaitAndConvert;
				DSActiveSensor++;
			}
		} break;
		case SCRATCHPAD_WRITE: {
			DSCycleCnt = 0;

			DSState = DSWriteEEPROM;
		} break;
		case SCRATCHPAD_COPY: {
			if (DSActiveSensor == DSSensorsCnt - 1) {
				DSStartMeasurement();
			} else {
				DSCycleCnt = 0;
				DSActiveSensor++;

				DSState = DSSetResolution;
			}
		}
	}

}

void OWireNoSlaves() { // Na linii 1-Wire nie ma zadnego ukladu podrzednego
	GPIOD->BSRRL = (1 << 14);
}


