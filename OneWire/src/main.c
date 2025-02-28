/*
 * main.cpp

 *
 *  Created on: 12.06.2019
 *      Author: i5
 */
#include "../inc/stm32f4xx.h"
#include "../inc/hdr_rcc.h"
#include "OneWire.h"
#include "DS18B20.h"

void Init_PLL();
void ROMCodeReceived(const uint8_t * ROMCode);

#define SENSORS_COUNT 2

DS18B20 TempSensors[SENSORS_COUNT] = {
		{ .DSAddress = { 0x28, 0xFB, 0xC0, 0xFF, 0x08, 0x00, 0x00, 0x78 }, .DSTemperature = 0.0f}, // Termometr 1
		{ .DSAddress = { 0x28, 0xBA, 0x25, 0x00, 0x09, 0x00, 0x00, 0x72 }, .DSTemperature = 0.0f}, // Termometr 2
};

int main(void){
	Init_PLL();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Uruchom zegar dla portu GPIOD

	GPIOD->BSRRH = GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15; // Wyzeruj PIN12 PIN13 PIN14 PIN15
	GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0; // Ustaw wyjscie na PIN12 PIN13 PIN14 PIN15

	DSInitSensors(TempSensors, SENSORS_COUNT);

	DSStartMeasurement();

	//DSSetSensorsResolution(TEMP_9BIT);

	//DSGetROMCode(ROMCodeReceived);

	while(1) {}
}

void ROMCodeReceived(const uint8_t * ROMCode) {
	// Kod ma 8 bajtow
}

static void flash_latency(uint32_t frequency)
{
	uint32_t wait_states;

	wait_states = frequency / 30000000ul;	// calculate wait_states (30M is valid for 2.7V to 3.6V voltage range, use 24M for 2.4V to 2.7V, 18M for 2.1V to 2.4V or 16M for  1.8V to 2.1V)
	wait_states &= 7;						// trim to max allowed value - 7

	FLASH->ACR = wait_states;				// set wait_states, disable all caches and prefetch
	FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST | wait_states;	// reset caches
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | wait_states;	// enable caches and prefetch
}

void Init_PLL() {
	uint32_t frequency = 84000000;
	uint32_t crystal = 8000000;
	uint32_t div, mul, div_core, vco_input_frequency, vco_output_frequency, frequency_core;
	uint32_t best_div = 0, best_mul = 0, best_div_core = 0, best_frequency_core = 0;

	RCC_CR_HSEON_bb = 1;					// enable HSE clock
	flash_latency(frequency);				// configure Flash latency for desired frequency

	for (div = 2; div <= 63; div++)			// PLLM in [2; 63]
	{
		vco_input_frequency = crystal / div;

		if ((vco_input_frequency < 1000000ul) || (vco_input_frequency > 2000000))	// skip invalid settings
			continue;

		for (mul = 64; mul <= 432; mul++)	// PLLN in [64; 432]
		{
			vco_output_frequency = vco_input_frequency * mul;

			if ((vco_output_frequency < 64000000ul) || (vco_output_frequency > 432000000ul))	// skip invalid settings
				continue;

			for (div_core = 2; div_core <= 8; div_core += 2)	// PLLP in {2, 4, 6, 8}
			{
				frequency_core = vco_output_frequency / div_core;

				if (frequency_core > frequency)	// skip values over desired frequency
					continue;

				if (frequency_core > best_frequency_core)	// is this configuration better than previous one?
				{
					best_frequency_core = frequency_core;	// yes - save values
					best_div = div;
					best_mul = mul;
					best_div_core = div_core;
				}
			}
		}
	}

	RCC->PLLCFGR = (best_div << RCC_PLLCFGR_PLLM_bit) | (best_mul << RCC_PLLCFGR_PLLN_bit) | ((best_div_core / 2 - 1) << RCC_PLLCFGR_PLLP_bit) | RCC_PLLCFGR_PLLQ_DIV9 | RCC_PLLCFGR_PLLSRC_HSE;	// configure PLL factors, always divide USB clock by 9

	RCC->CFGR = RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_HPRE_DIV1;	// AHB - no prescaler, APB1 - divide by 4, APB2 - divide by 2

	while (!RCC_CR_HSERDY_bb);				// wait for stable clock

	RCC_CR_PLLON_bb = 1;					// enable PLL
	while (!RCC_CR_PLLRDY_bb);				// wait for PLL lock

	RCC->CFGR |= RCC_CFGR_SW_PLL;			// change SYSCLK to PLL
	while (((RCC->CFGR) & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	// wait for switch
}
