/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"

// Variables
static volatile bool i2c_running = false;
static volatile float current_sensor_gain = 0.0;
static volatile float input_current_sensor_gain = 0.0;
static volatile float input_current_sensor_offset = 1.65;
static volatile uint16_t input_current_sensor_offset_samples = 0;
static volatile uint32_t input_current_sensor_offset_sum = 0;
static volatile bool current_input_sensor_offset_start_measurement = false;
// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

//Functions
static void terminal_cmd_gt_input_offset(int argc, const char **argv);

static void terminal_cmd_gt_get_humidity(int argc, const char **argv);

void buzzer_beep(void) {
    // External Buzzer (using servo pin!)
    palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN,
                  PAL_MODE_OUTPUT_PUSHPULL |
                  PAL_STM32_OSPEED_HIGHEST);
	palSetPad(HW_ICU_GPIO, HW_ICU_PIN);
    chThdSleepMilliseconds(150);
    palClearPad(HW_ICU_GPIO, HW_ICU_PIN);
}


void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);


	// GPIOE Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOE, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | //1L
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOE, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | //1H
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOE, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | //2L
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOE, 11, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | //2H
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOE, 12, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | //3L
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOE, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | //3H
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);


	// ADC Pins
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); //vin channel 1
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);// fet temp channel 4
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); //ext1 channel 11 
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); //ext2 channel 10
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); //input current channel 12
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG); //motor temp channel 13
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); //current phase 1 channel 14
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG); //current phase 2 channel 15
	//palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);//current phase 1 vref - not really need. offset calibration inherently handles these. 
	//palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);//current phase 2 vref 

	buzzer_beep();
	
	//Terminal stuff
	terminal_register_command_callback(
		"gt_input_offset",
		"Print GT input current sensor offset",
		0,
		terminal_cmd_gt_input_offset);

	terminal_register_command_callback(
		"gt_get_humidity",
		"Print GT humidity sensor information",
		0,
		terminal_cmd_gt_get_humidity);

}

void hw_setup_adc_channels(void) {
	uint8_t t_samp = ADC_SampleTime_15Cycles;

	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, t_samp); // 0 - Current 1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, t_samp); // 3 - adc1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 3, t_samp); // 6 - vrefint
	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, t_samp); // 1 - Current 2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 2, t_samp); // 4 - adc2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 3, t_samp); // 7 - temp fet
	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);// 2 - Input Current
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 2, t_samp); // 5 - Temp motor
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 3, t_samp); // 8 - Vin


	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_14, 1, t_samp); // Current 1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_15, 1, t_samp); // Current 2 
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp); // Current 3
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_14, 2, t_samp); // Current 1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_15, 2, t_samp); // Current 2
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, t_samp); // Current 3
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_14, 3, t_samp); // Current 1 
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_15, 3, t_samp); // Current 2 
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, t_samp); // Current 3
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

float hw_gt_get_humidity(void) {
	float ret_value = 0.0;
	//..... get humidity sensor information
	uint8_t rxbuf[10];
	uint8_t txbuf[10];
	uint8_t ctxbuf[10];
	systime_t tmo = MS2ST(5);
	systime_t tmo2 = MS2ST(15);
	i2caddr_t humidity_addr = 0x40;
	ctxbuf[0] = 0x02;
	ctxbuf[1] = 0;
	ctxbuf[1] &= ~(1 << 4); //individual sensor
	ctxbuf[1] &= ~( (1 << 1) | (1 << 0) ); //14 bit humidity
	txbuf[0] = 0x01; //humidity register

	i2cAcquireBus(&HW_I2C_DEV);
	i2cMasterTransmitTimeout(&HW_I2C_DEV, humidity_addr, ctxbuf, 1, rxbuf, 0, tmo); //configure
	chThdSleep(tmo2);
	i2cMasterTransmitTimeout(&HW_I2C_DEV, humidity_addr, txbuf, 1, rxbuf, 0, tmo); //measure
	chThdSleep(tmo2); //wait for measurement
	i2cMasterTransmitTimeout(&HW_I2C_DEV, humidity_addr, txbuf, 1, rxbuf, 2, tmo); //read
	i2cReleaseBus(&HW_I2C_DEV);
	uint16_t raw_humidity = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
	ret_value = ((float)raw_humidity / 65536.0f) * 100.0f;

	return ret_value;
}

float hw_gt_get_temperature(void) {
	float ret_value = 0.0;
	//.... get humidity sensor temp information
	uint8_t rxbuf[10];
	uint8_t txbuf[10];
	uint8_t ctxbuf[10];
	systime_t tmo = MS2ST(5);
	systime_t tmo2 = MS2ST(15);
	i2caddr_t humidity_addr = 0x40;
	ctxbuf[0] = 0x02;
	ctxbuf[1] = 0;
	ctxbuf[1] &= ~(1 << 4); //individual sensor
	ctxbuf[1] &= ~(1 << 2); //14 bit temperature
	txbuf[0] = 0x00; //temperature register

	i2cAcquireBus(&HW_I2C_DEV);
	i2cMasterTransmitTimeout(&HW_I2C_DEV, humidity_addr, ctxbuf, 1, rxbuf, 0, tmo); //configure
	chThdSleep(tmo2);
	i2cMasterTransmitTimeout(&HW_I2C_DEV, humidity_addr, txbuf, 1, rxbuf, 0, tmo); //measure
	chThdSleep(tmo2); //wait for measurement
	i2cMasterTransmitTimeout(&HW_I2C_DEV, humidity_addr, txbuf, 1, rxbuf, 2, tmo); //read
	i2cReleaseBus(&HW_I2C_DEV);
	uint16_t raw_temperature = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
	ret_value = ((float)raw_temperature / 65536.0f) * 165.0f - 40.0f;
	return ret_value;
}

float hw_gt_read_input_current(void) {
	float ret_value = 0.0;
	ret_value = ( (V_REG / 4095.0) * (float)ADC_Value[ADC_IND_INCURR] - input_current_sensor_offset ) / IN_CURRENT_GAIN;
	return ret_value;
}

void hw_gt_get_input_current_offset(void){

	if(current_input_sensor_offset_start_measurement){

		if( input_current_sensor_offset_samples == 100 ){
			current_input_sensor_offset_start_measurement = false;
			input_current_sensor_offset = ((float)input_current_sensor_offset_sum) / 100.0;
			input_current_sensor_offset *= (V_REG / 4095.0);
		}
		else{
			input_current_sensor_offset_sum += 	ADC_Value[ADC_IND_INCURR];
			input_current_sensor_offset_samples++;
		}
	}else{
		input_current_sensor_offset_samples++;
	}
}

void hw_gt_start_input_current_sensor_offset_measurement(void){
	current_input_sensor_offset_start_measurement = true;
	input_current_sensor_offset_samples = 0;
	input_current_sensor_offset_sum = 0;
}

static void terminal_cmd_gt_input_offset(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("GT input current offset is set as %.8f", (double)input_current_sensor_offset);

	commands_printf(" ");
	return;
}

static void terminal_cmd_gt_get_humidity(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	systime_t tmo = MS2ST(20);
	float temp = 0;
	float humidity = 0;
	temp = hw_gt_get_temperature();
	chThdSleep(tmo);
	humidity = hw_gt_get_humidity();

	commands_printf("Humidity is at %.1f%%", (double)humidity);
	commands_printf(" ");
	commands_printf("Temperature is at %.1f C", (double)temp);

	return;
}