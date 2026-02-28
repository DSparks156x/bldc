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

#ifndef HW_GT_CORE_H_
#define HW_GT_CORE_H_

#define HW_MAJOR				4
#define HW_MINOR				20

#define QMLUI_SOURCE_HW		"hwconf/fm/gt/qmlui_gt.c"
#define QMLUI_HEADER_HW		"hwconf/fm/gt/qmlui_gt.h"

#include "gtbms.h"

// HW properties
#define HW_HAS_PHASE_SHUNTS //It has phase shunts alright, only 2 of em.
#define HW_SHUNT_1_2 //phase shunts are on 1 and 2. Vesc otherwise orders things differently. 
#define HW_NO_PHASE_VSENSE //It has no phase voltage sensing. why. 
#define HW_USE_INTERNAL_RC  
#define HW_HAS_INPUT_CURRENT_SENSOR //gt basically an axiom. 
#define HW_HAS_NO_CAN //why would gt have this.
// Macros
#define LED_GREEN_ON() //it gets angry without these. obviously the gt controller doesnt have basic bitch status leds. 
#define LED_GREEN_OFF()
#define LED_RED_ON()
#define LED_RED_OFF()

/*
 * ADC Vector
 *
 * 0:	IN14	CURR1 PC4 adc12
 * 1:	IN15	CURR2 PC5 adc12
 * 2:	IN12	INPUTCURR PC2 adc123
 * 3:	IN11	ADC_EXT1 PC0 adc123
 * 4:	IN10	ADC_EXT2 PC1 adc123
 * 5:	IN13	TEMP_MOTOR PC3 adc123
 * 6:	Vrefint
 * 7:	IN4 	TEMP_PCB PA4 adc12
 * 8:	IN1 	VIN_IN PA1 adc123
 */

#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			3
#define HW_ADC_CHANNELS			(HW_ADC_NBR_CONV * 3)

// ADC Indexes
#define ADC_IND_SENS1			8 //GT doesnt have these silly things but it doesnt like building without them. 
#define ADC_IND_SENS2			8 //
#define ADC_IND_SENS3			8 //
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_INCURR			2 //HW gt will go brrr. 
#define ADC_IND_VIN_SENS		8
#define ADC_IND_EXT				3
#define ADC_IND_EXT2			4
#define ADC_IND_TEMP_MOS		7
#define ADC_IND_TEMP_MOTOR		5
#define ADC_IND_VREFINT			6
 
// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R2
#define VIN_R2					1.0 //easier to calculate. 
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		1 //hall effect shunt thingies. 
#endif


#ifdef GTS
    #ifdef GTS150V //for extra spicy 150v capable GTS controllers.
        #ifndef VIN_R1
            #define VIN_R1					48 // placeholder
        #endif
    #else
        #ifndef VIN_R1
            #define VIN_R1					34 // placeholder. if reg GT cant measure more than 2v above full charge this thing probably cant either. 
        #endif
    #endif
    #ifdef  GTS150A //for extra spicy 150A measurement GTS controllers. 
        #ifndef CURRENT_AMP_GAIN
            #define CURRENT_AMP_GAIN		(0.01333*(2.0/3.0))  //0.01333v/a * 2/3 divider
        #endif
    #else
        #ifndef CURRENT_AMP_GAIN
            #define CURRENT_AMP_GAIN		(0.02*(2.0/3.0))  //0.02v/a * 2/3 divider
        #endif     
    #endif
    #ifndef IN_CURRENT_GAIN
        #define IN_CURRENT_GAIN         (-0.04*(2.0/3.0)) //0.04v/a * 2/3 divider * inverted
    #endif
#endif
#ifdef GT
#ifndef VIN_R1
#define VIN_R1					22.9045 // 20.05/0.861 = 23.2868 = (R1+R2) / R2 )
#endif

#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		(0.02*(2.0/3.0))  //0.02v/a * 2/3 divider
#endif

#ifndef IN_CURRENT_GAIN
#define IN_CURRENT_GAIN         (-0.04*(2.0/3.0)) //0.04v/a * 2/3 divider * inverted
#endif
#endif



// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

//Input current GT cool like this.
#define GET_INPUT_CURRENT()				hw_gt_read_input_current()
#define GET_INPUT_CURRENT_OFFSET()		hw_gt_get_input_current_offset()
#define MEASURE_INPUT_CURRENT_OFFSET()	hw_gt_start_input_current_sensor_offset_measurement()


// NTC Termistors

#define NTC_RES(adc_val)		(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) 
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side. 
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Footpad ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOC
#define HW_ADC_EXT_PIN			1
#define HW_ADC_EXT2_GPIO		GPIOC
#define HW_ADC_EXT2_PIN			0

// UART Peripheral - BMS RS485
#define HW_UART_DEV				SD2
#define HW_UART_GPIO_AF			GPIO_AF_USART2 //USART 2
#define HW_UART_TX_PORT			GPIOA //PA2 
#define HW_UART_TX_PIN			2
#define HW_UART_RX_PORT			GPIOA //PA3
#define HW_UART_RX_PIN			3

// RS485 Direction Control (DERE)
#define HW_BMS_DERE_PORT		GPIOD
#define HW_BMS_DERE_PIN			4

// Permanent UART Peripheral - Bluetooth Module 
#define HW_UART_P_BAUD		115200
#define HW_UART_P_DEV			SD3
#define HW_UART_P_GPIO_AF		GPIO_AF_USART3
#define HW_UART_P_TX_PORT		GPIOD
#define HW_UART_P_TX_PIN		8 
#define HW_UART_P_RX_PORT		GPIOD
#define HW_UART_P_RX_PIN		9

// ICU Peripheral - status bar
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB //Status bar
#define HW_ICU_PIN				10

// I2C Peripheral - Humidity/Temp Sensor
#define HW_USE_I2CD1
#define HW_I2C_DEV				I2CD1
#define HW_I2C_GPIO_AF			GPIO_AF_I2C1
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			6
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			7

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS		    GPIOA //unused pin on footpad. vesc gets angry if this isnt defined. 
#define HW_SPI_PIN_NSS		    11 
#define HW_SPI_PORT_SCK		    GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI		    7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO		    6

// IMU - LSM6DSL - SPI

#define LSM6DS3_NSS_GPIO		GPIOD
#define LSM6DS3_NSS_PIN			1
#define LSM6DS3_SCK_GPIO		GPIOA
#define LSM6DS3_SCK_PIN			5
#define LSM6DS3_MOSI_GPIO		GPIOA
#define LSM6DS3_MOSI_PIN		7
#define LSM6DS3_MISO_GPIO		GPIOA
#define LSM6DS3_MISO_PIN		6

// Measurement macros
#define ADC_V_L1				0   //placeholders. 
#define ADC_V_L2				0
#define ADC_V_L3				0

#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					26000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150.0	//Cant measure high enough for an OC fault. so. oh well. 
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX         30
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN         -30
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// May be worth trying. 
#endif

//Dead time override
#define HW_DEAD_TIME_NSEC		360.0 //I should probably calculate this correctly. but this seems fine. i think.

//Default motor confs
#ifdef GT
#include "hw_gt_conf.h"
#endif
#ifdef GTS
#include "hw_gts_conf.h"
#endif

// Setting limits
//GT
#ifdef GT
    #define HW_LIM_CURRENT			-95.0, 95.0
    #define HW_LIM_CURRENT_IN		-32, 32.0
    #define HW_LIM_CURRENT_ABS		0.0, 150.0 
    #define HW_LIM_VIN				14.0, 92.0
    #define HW_LIM_ERPM				-200e3, 200e3
    #define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 85.0 //setting this low. fet kaboom bad. 
#endif
//GTS
#ifdef GTS
    #ifdef GTS150A
        #define HW_LIM_CURRENT          -135.0, 135.0
    #else
        #define HW_LIM_CURRENT			-95.0, 95.0
    #endif
    #define HW_LIM_CURRENT_IN		-45, 45.0
    #define HW_LIM_CURRENT_ABS		0.0, 150.0
    #define HW_LIM_VIN				14.0, 119.0 //120v fets on a 27s controller is dumb to put it lightly. i dont know how to set this. We can risk going higher and fet go kaboom. We can keep it as it is and the rider goes down. Lose-lose. 
    #define HW_LIM_ERPM				-200e3, 200e3
    #define HW_LIM_DUTY_MIN			0.0, 0.1
    #define HW_LIM_DUTY_MAX			0.0, 0.99
    #define HW_LIM_TEMP_FET			-40.0, 85.0 //setting this lowish. fet kaboom bad.
#endif

//HW Functions
void hw_gt_get_hdc(float* temp, float* humidity);
float hw_gt_read_input_current(void);
void hw_gt_get_input_current_offset(void);
void hw_gt_start_input_current_sensor_offset_measurement(void);
#endif /* HW_GT_CORE_H_ */

