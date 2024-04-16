/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include "anain.h"
#include "my_math.h"

#if ADC_COUNT == 1
   #define TRANSFER_PSIZE DMA_CCR_PSIZE_16BIT
   #define TRANSFER_MSIZE DMA_CCR_MSIZE_16BIT
#elif ADC_COUNT == 2
   #define TRANSFER_PSIZE DMA_CCR_PSIZE_32BIT
   #define TRANSFER_MSIZE DMA_CCR_MSIZE_32BIT
   #undef ANA_IN_ENTRY
   #define ANA_IN_ENTRY(name, port, pin) +1
   #if ((ANA_IN_LIST) & 1) == 1
      #error In dual ADC mode you must define an even number of inputs
   #endif
   #undef ANA_IN_ENTRY
#else
   #error ADC_COUNT must be 1 or 2
#endif // ADC_COUNT

#define ADC_DMA_CHAN 1
#define MEDIAN3_FROM_ADC_ARRAY(a) median3(*a, *(a + ANA_IN_COUNT), *(a + 2*ANA_IN_COUNT))

uint8_t AnaIn::channel_array[ADC_COUNT][ANA_IN_COUNT / ADC_COUNT];
uint16_t AnaIn::values[NUM_SAMPLES*ANA_IN_COUNT];

#undef ANA_IN_ENTRY
#define ANA_IN_ENTRY(name, port, pin) AnaIn AnaIn::name(__COUNTER__);
ANA_IN_LIST
#undef ANA_IN_ENTRY

/**
* Initialize ADC hardware and start DMA based conversion process
*/
void AnaIn::Start()
{
   uint32_t adc[] = { ADC1, ADC2 };

   for (int i = 0; i < ADC_COUNT; i++)
   {
      adc_power_off(adc[i]);
      adc_enable_scan_mode(adc[i]);
      adc_set_continuous_conversion_mode(adc[i]);
      adc_set_right_aligned(adc[i]);
      adc_set_sample_time_on_all_channels(adc[i], SAMPLE_TIME);
      adc_power_on(adc[i]);
      adc_reset_calibration(adc[i]);
      adc_calibrate(adc[i]);
      adc_set_regular_sequence(adc[i], ANA_IN_COUNT / ADC_COUNT, channel_array[i]);
      adc_enable_dma(adc[i]);
      adc_enable_external_trigger_regular(adc[i], ADC_CR2_EXTSEL_SWSTART);
   }

   dma_set_peripheral_address(DMA1, ADC_DMA_CHAN, (uint32_t)&ADC_DR(ADC1));
   dma_set_memory_address(DMA1, ADC_DMA_CHAN, (uint32_t)values);
   dma_set_peripheral_size(DMA1, ADC_DMA_CHAN, TRANSFER_PSIZE);
   dma_set_memory_size(DMA1, ADC_DMA_CHAN, TRANSFER_MSIZE);
   dma_set_number_of_data(DMA1, ADC_DMA_CHAN, NUM_SAMPLES * ANA_IN_COUNT / ADC_COUNT);
   dma_enable_memory_increment_mode(DMA1, ADC_DMA_CHAN);
   dma_enable_circular_mode(DMA1, ADC_DMA_CHAN);
   dma_enable_channel(DMA1, ADC_DMA_CHAN);

   #if ADC_COUNT == 2
   adc_set_dual_mode(ADC_CR1_DUALMOD_CRSISM);
   #endif
   adc_start_conversion_regular(ADC1);
}

void AnaIn::Configure(uint32_t port, uint8_t pin)
{
   gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, 1 << pin);

   #if ADC_COUNT == 1
   channel_array[0][GetIndex()] = AdcChFromPort(port, pin);
   #elif ADC_COUNT == 2
   channel_array[GetIndex() & 1][GetIndex() / 2] = AdcChFromPort(port, pin);
   #endif // ADC_COUNT
}

/**
* Get filtered value of given channel
*
*  - NUM_SAMPLES = 1: Most recent sample is returned
*  - NUM_SAMPLES = 3: Median of last 3 samples is returned
*  - NUM_SAMPLES = 9: Median of last 3 medians is returned
*  - NUM_SAMPLES = 12: Average of last 4 medians is returned
*  - NUM_SAMPLES = 64: Average of last 64 samples is returned
*
* @return Filtered value
*/
uint16_t AnaIn::Get()
{
   #if NUM_SAMPLES == 1
   return *firstValue;
   #elif NUM_SAMPLES == 3
   return MEDIAN3_FROM_ADC_ARRAY(firstValue);
   #elif NUM_SAMPLES == 4
   return (*firstValue + *(firstValue + ANA_IN_COUNT) + *(firstValue + 2*ANA_IN_COUNT) + *(firstValue + 3*ANA_IN_COUNT)) / 4;
   #elif NUM_SAMPLES == 9
   uint16_t *curVal = firstValue;
   uint16_t med[3];

   for (int i = 0; i < 3; i++, curVal += 3*ANA_IN_COUNT)
   {
      med[i] = MEDIAN3_FROM_ADC_ARRAY(curVal);
   }

   return MEDIAN3(med[0], med[1], med[2]);
   #elif NUM_SAMPLES == 12
   uint16_t *curVal = firstValue;
   uint16_t med[4];

   for (int i = 0; i < 4; i++, curVal += 3*ANA_IN_COUNT)
   {
      med[i] = MEDIAN3_FROM_ADC_ARRAY(curVal);
   }

   return (med[0] + med[1] + med[2] + med[3]) >> 2;
   #elif NUM_SAMPLES == 64
   uint16_t *curVal = firstValue;
   uint32_t sum = 0;

   for (int i = 0; i < NUM_SAMPLES; i++, curVal += ANA_IN_COUNT)
   {
      sum += *curVal;
   }

   return sum >> 6;
   #else
   #error NUM_SAMPLES must be 1, 3, 9, 12 or 64
   #endif
}

int AnaIn::median3(int a, int b, int c)
{
   return MEDIAN3(a,b,c);
}

uint8_t AnaIn::AdcChFromPort(uint32_t command_port, int command_bit)
{
    /*
     PA0 ADC12_IN0
     PA1 ADC12_IN1
     PA2 ADC12_IN2
     PA3 ADC12_IN3
     PA4 ADC12_IN4
     PA5 ADC12_IN5
     PA6 ADC12_IN6
     PA7 ADC12_IN7
     PB0 ADC12_IN8
     PB1 ADC12_IN9
     PC0 ADC12_IN10
     PC1 ADC12_IN11
     PC2 ADC12_IN12
     PC3 ADC12_IN13
     PC4 ADC12_IN14
     PC5 ADC12_IN15
     temp ADC12_IN16
     */
    switch (command_port)
    {
    case GPIOA: /* port A */
        if (command_bit<8) return command_bit;
        break;
    case GPIOB: /* port B */
        if (command_bit<2) return command_bit+8;
        break;
    case GPIOC: /* port C */
        if (command_bit<6) return command_bit+10;
        break;
    }
    adc_enable_temperature_sensor();
    return 16;
}
