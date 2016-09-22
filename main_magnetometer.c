/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*=========================================================*/
/* Implementation of ECompass  */
/* Author: Rohan Sarkar                                    */
/* ======================================================== */
#include "MKL46Z4.h"
#include "slcd.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
// -----------------------------------------------------------------
/*========================================================*/
/* Define Macros */
/* =======================================================*/
/*========================================================*/
/* Define PIN Function */
/* =======================================================*/
#define PIN(x)                 (1 << x)
/*========================================================*/
/* Macros for  Green LED*/
/* =======================================================*/
// Green LED is connected to PTD5 and define functions for Green LED
#define GREEN_LED             (5)
#define GREEN_LED_ON()       PTD->PCOR |= PIN(GREEN_LED) ;// Define function to turn Green LED ON
#define GREEN_LED_OFF()      PTD->PSOR |= PIN(GREEN_LED) ;// Define function to turn Green LED OFF
#define GREEN_LED_TOGGLE()   PTD->PTOR |= PIN(GREEN_LED) ;// Define function to TOGGLE Green LED
/*========================================================*/
/* Macros for  Red LED*/
/* =======================================================*/
// Red LED is connected to PTE29 and define functions for Red LED
#define RED_LED             (29)
#define RED_LED_ON()       PTE->PCOR |= PIN(RED_LED) ; //Define function to turn Red LED ON
#define RED_LED_OFF()      PTE->PSOR |= PIN(RED_LED) ; //Define function to turn Red LED OFF
#define RED_LED_TOGGLE()   PTE->PTOR |= PIN(RED_LED) ; //Define function to TOGGLE Red LED
/*========================================================*/
/* Define SWitch 1(SW1) and Switch 2(SW3) */
/* =======================================================*/
// SW1 is connected to PTC3
#define SW1             (3)
// SW2 is connected to PTC12
#define SW2             (12)
// This is used to set the internal pull up resistor of the corresponding pin.
// Setting PE = 1 and PS = 1 for the corresponding pin to which the switches are connected.
#define ENABLE_PULLUP_RESISTOR  (3)
/*========================================================*/
/* Macros for I2C Bus*/
/* =======================================================*/
#define READ_MASK                          0x01
#define WRITE_MASK                         0xFE
#define DATA_SHIFT                         1
#define ACC_DEVICE_ADDRESS                 0x1D
#define RESET_MASK                         0x00
#define RIGHT_SHIFT(x,y)                   (x >> y)
#define LEFT_SHIFT(x,y)                    (x << y)
#define I2C0_SCL                           (24)
#define I2C0_SDA                           (25)
#define READ(x)                            ((x<<1)|(0x01))
#define WRITE(x)                           ((x<<1)&(0xFE))
/*========================================================*/
/* Macros for Magnetometer Device*/
/* =======================================================*/
#define MAG_DEVICE_ADDRESS                 0x0E
#define MAG_CTRL_REG1                      0x10
#define MAG_CTRL_REG2                      0x11
#define MAG_DR_STATUS                      0x00
#define MAG_OUT_X_MSB                      0x01
#define MAG_OUT_X_LSB                      0x02
#define MAG_OUT_Y_MSB                      0x03
#define MAG_OUT_Y_LSB                      0x04
#define MAG_OUT_Z_MSB                      0x05
#define MAG_OUT_Z_LSB                      0x06
#define MAG_DEVICE_ID_REGISTER_ADDRESS     0x07
#define MAG_SYSMOD                         0x08
#define MAG_OFF_X_MSB                      0x09
#define MAG_OFF_X_LSB                      0x0A
#define MAG_OFF_Y_MSB                      0x0B
#define MAG_OFF_Y_LSB                      0x0C
#define MAG_OFF_Z_MSB                      0x0D
#define MAG_OFF_Z_LSB                      0x0E
/*========================================================*/
/* Macros for Accelerometer Device*/
/* =======================================================*/
#define ACC_DEVICE_ADDRESS                 0x1D
#define ACC_DEVICE_ID_REGISTER_ADDRESS     0x0D
#define XYZ_CFG 						   0x0E
#define ACC_CTRL_REG1                      0x2A
#define ACC_OUT_X_MSB                      0x01
// -----------------------------------------------------------------
/*========================================================*/
/* Define States*/
/* =======================================================*/
typedef enum {
	STOP,
	RUN,
	MAG_ACQ,
	MAG_CAL,
	ACC_CAL
} enumECompassOperationState;

enumECompassOperationState enumECompassState = STOP;

bool bSW3Pressed = false;
bool bSW1Pressed = false;

bool bIsTimerExpired = false;

unsigned char    ucSecond = 0;
unsigned char    ucHundredsMilliSecond = 0;
unsigned char    ucMinute = 0;
unsigned short   usTimeElapsed = 0;

unsigned char    ucaryLCDMsg[5] = "";
/*========================================================*/
/* Define and initialise variables*/
/* =======================================================*/
// --------------------------------------------------------------------
// Defining variables to be used for calculation of TPM0 MOD Count for Timer Overflow every 0.1 second
int prescalar;
int prescalar_factor;//to be
int resolution;
int original_clock_frequency; //in kHz
int new_clock_frequency ;
int timer_overflow_count;
int reset_count;
// Defining variables to be used for acquiring and processing Magnetometer Data
unsigned char DATA_READ[6];
short int MAG_DATA_READ_AXIS[3];
short int MAG_DATA_MAX_AXIS[3];
short int MAG_DATA_MIN_AXIS[3];
short int MAG_DATA_AVERAGE_AXIS[3];
short int MAG_DATA_HI_CALIBRATED[3];
unsigned char DR_STATUS_DATA;
short int ANGLE;
// Defining variables to be used for acquiring and processing Accelerometer Data
short int ACC_DATA_READ_AXIS[3];
short int ACC_DATA_OFFSET_AXIS[3];
short int ACC_DATA_CALIBRATED[3];
unsigned char DR_STATUS_DATA_ACC;
// Defining variables to be used as flags
int flag_first_data = 0;
int flag_initialisation = 0;
int flag_first_data_acquire =0;
int flag_first_data_run =0;
/*========================================================*/
/* Delay Function to allow processes to complete on I2C Bus*/
/* =======================================================*/
void delay()
	{for(int i =1;i<100;i++)
	{}
	}
/*========================================================*/
/* Function to initialise LEDs*/
/* =======================================================*/
void LED_Init(void)
{
// --------------------------------------------------------------------
	/*========================================================*/
	/*   Turn on clock to Port D and E module respectively*/
	/* ======================================================== */
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	/*========================================================*/
	/*Set configuration for Green LED (Output Mode)*/
	/* ======================================================== */
	/*Set the PTD5 pin multiplexer to GPIO mode*/
	PORTD->PCR[GREEN_LED] = PORT_PCR_MUX(1) ;
	/*Set the initial output state to low*/
	PTD->PCOR |= PIN(GREEN_LED) ;
	/*Set the pins direction to output*/
	PTD->PDDR |= PIN(GREEN_LED);
	/*========================================================*/
	/*Set configuration for Red LED (Output Mode)*/
	/* ======================================================== */
	/*Set the PTE29 pin multiplexer to GPIO mode*/
	PORTE->PCR[RED_LED] = PORT_PCR_MUX(1) ;
	/*Set the initial output state to low*/
	PTE->PCOR |= PIN(RED_LED) ;
	/*Set the pins direction to output*/
	PTE->PDDR |= PIN(RED_LED);
// --------------------------------------------------------------------
}
/*========================================================*/
/* Function to initialise Switches*/
/* =======================================================*/
void SWITCH_Init(void)
{
// --------------------------------------------------------------------
	/*========================================================*/
	/*   Turn on clock to Port C module respectively*/
	/* ======================================================== */
		SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	/*========================================================*/
	/*Set configuration for Switch 1 (Input Mode)*/
	/* ======================================================== */
	/*PTC3 pin set to GPIO mode, PE(bit0) and PS(bit1) set to 1 by ENABLE_PULLUP_RESISTOR,
	 * PTC_IRQC(Interrupt Configuration) set to 1010 to detect interrupt on falling edge  */
		PORTC->PCR[SW1] |= PORT_PCR_MUX(1) | ENABLE_PULLUP_RESISTOR | PIN(17) | PIN(19);
	/*========================================================*/
	/*Set configuration for Switch 2 (Input Mode)*/
	/* ======================================================== */
	/*PTC3 pin set to GPIO mode, PE(bit0) and PS(bit1) set to 1 by ENABLE_PULLUP_RESISTOR,
	 * PTC_IRQC(Interrupt Configuration) set to 1010 to detect interrupt on falling edge  */
		PORTC->PCR[SW2] |= PORT_PCR_MUX(1) | ENABLE_PULLUP_RESISTOR | PIN(17) | PIN(19);
	/*========================================================*/
	/*Set the pins direction to input (Set the corresponding bit to 0) */
	/*========================================================*/
	PTC->PDDR &= (~PIN(SW1))&(~PIN(SW2));
// --------------------------------------------------------------------
}
/*========================================================*/
/* Function to initialise Timer*/
/* =======================================================*/
void TIMER_Init(void)
{
// --------------------------------------------------------------------
	/*===========================================================================================================================*/
	/*Calculation of the MOD Value or upper limit for the TPM0 Counter to trigger a Timer Overflow Interrupt:*/
	/*User can change the prescalar factor and resolution if needed and the upper count limit will be calculated automatically*/
	//TPM counter increments on every TPM counter clock
	//Prescale Factor set to 6, It divides clock frequency by 2^6 = 64.New Frequency = 8MHz/64 = 0.125 MHz = 125kHz
    //Time period = 1/New Frequency.
	//timer_overflow_count = resolution/time period = resolution * original_clock_frequency / prescalar = 12500
	/*===========================================================================================================================*/
	prescalar_factor = 6;//user specified prescalar factor
	prescalar = 1<<(prescalar_factor);//Left shift to find the prescalar
	resolution = 100; //The lowest count in milliseconds (Can be modified by user)
	original_clock_frequency = 8000; //The Frequency of the onboard crystal oscillator in kHz
	timer_overflow_count = resolution * original_clock_frequency / prescalar;//Calculating the MOD value after which the Timer Overflow occurs and an interrupt is triggered
	reset_count = 0;//Reset value for TPM0
	/*===========================================================================================================================*/
	/*Connect Clock Source to the TPM0 */
	/*===========================================================================================================================*/
	OSC0->CR |= PIN(7); //Enabling the external reference clock through ERCLKEN bit in OSC0 Control Register
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; //Enable TPM0 clock using System Clock Gating Control Register 6
	SIM->SOPT2 |= PIN(25); //Selecting the OSCERCLK as source of TPM0 by setting TPMSRC to 10 in System Options Register 2
	/*===========================================================================================================================*/
	/*Resetting TPM0 Count Values and Specifying the upper limit(MOD value) for the TPM0 Counter */
	/*===========================================================================================================================*/
	TPM0->CNT = reset_count; //The CNT register of TPM0 is reset to 0
	TPM0->MOD = timer_overflow_count; //Setting the maximum count value. TPM0 counter reaches this modulo value and increments, the overflow flag (TOF) is set.
	/*===========================================================================================================================*/
	/*Status and Control register configuration for the TPM0 */
	/*===========================================================================================================================*/
	TPM0->SC &=(~(TPM_SC_CPWMS_MASK));//CPWMS is set to 0 so that TPM0 Counter operates in upcounting mode
	TPM0->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(prescalar_factor);//CMOD is set to 01 so that TPM counter increments on every TPM counter clock and PS is set to user defined prescalar value
	TPM0->SC |= TPM_SC_TOIE_MASK; //Enabling the timer to raise an interrupt on overflow every hundred-millisecond
	TPM0->CONF |= PIN(7) | PIN(6); //Setting DBGMODE to be 11  in TPM0_CONF so that TPM counter continues in debug mode
// --------------------------------------------------------------------
}
/*========================================================*/
/* Function to initialise I2C0*/
/* =======================================================*/
void I2C0_Init(void)
{
// --------------------------------------------------------------------
/*===========================================================================================================================*/
/*Initializing I2C0 */
/*===========================================================================================================================*/
		// Enable I2C by providing Gate Clock to I2C0
		   SIM->SCGC4|=SIM_SCGC4_I2C0_MASK;
			/*========================================================*/
			/* Removing Dead Lock*/
			/* =======================================================*/
		   	// Setting Pins corresponding of I2C0 clock and Data to GPIO pins
		   	   PORTE->PCR[I2C0_SCL]|= PORT_PCR_MUX(1);
		   	   PORTE->PCR[I2C0_SDA]|= PORT_PCR_MUX(1);
		   	// Enabling pullup resistor I2C0 Clock
		   	  PORTE->PCR[I2C0_SCL]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
		   	// Enabling pullup resistor I2C0 Data
		   	  PORTE->PCR[I2C0_SDA]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
		   	// Set the direction of the I2C SCL register to output
		   	  PTE->PDDR |= LEFT_SHIFT(1,I2C0_SCL);
		   	// Set the direction of the I2C SDA register to input
		   	  PTE->PDDR &= ~LEFT_SHIFT(1,I2C0_SDA);
		   	while((PTE->PDIR & LEFT_SHIFT(1,I2C0_SDA)) == 0)
		   	{
		   		// Toggling the Serial clock
		   		PTE->PTOR|=LEFT_SHIFT(1,I2C0_SCL);
		   		// Generating a wait state
		   		for(int i=0;i<10; i++)
		   			{}
		   	}
		// Setting Pins corresponding to I2C0 clock and Data
		   PORTE->PCR[I2C0_SCL]|= PORT_PCR_MUX(5);
		   PORTE->PCR[I2C0_SDA]|= PORT_PCR_MUX(5);
		// Enabling pullup resistor I2C0 Clock
		  PORTE->PCR[I2C0_SCL]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
		// Enabling pullup resistor I2C0 Data
		  PORTE->PCR[I2C0_SDA]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
		// Setting MULT to 2h
		I2C0->F|=I2C_F_MULT(02);
		// Setting ICR to 00 so that the combination of MULT and ICR gives required baud rate of 100kbps
		I2C0->F|=I2C_F_ICR(0x00);
		// Enable the Module operation
		I2C0->C1|=I2C_C1_IICEN_MASK;
		
}
/*========================================================*/
/* Function for Timer Interrupt Handler*/
/* =======================================================*/
void TPM0_IRQHandler(void)
{
// --------------------------------------------------------------------
/*===========================================================================================================================*/
/*Update the Timer Expired state variable and Clearing the Timer Overflow Flag for the TPM0 */
/*===========================================================================================================================*/
	bIsTimerExpired = true;//This indicates that 1 unit lowest count/resolution(hundred-millisecond) has reached
	TPM0->SC |= TPM_SC_TOF_MASK; //Clears the overflow flag TOF so that clock can start counting the next lowest count or resolution (hundred-millisecond)
// --------------------------------------------------------------------
}
/*========================================================*/
/* Function for PORT C and PORT D Interrupt Handler*/
/* =======================================================*/
void PORTC_PORTD_IRQHandler(void)
{
// --------------------------------------------------------------------
	int SW1_Status, SW2_Status; //Indicates the status of SW1 and SW2 (if they are pressed or not)
	/*===========================================================================================================================*/
	/*Read Interrupt Status Flag for Switch 1 and Switch 2 and storing the status after right shifting to indicate a 0 or 1 value*/
	/*===========================================================================================================================*/
	SW1_Status=(PORTC->PCR[SW1] & PIN(24))>>24;
	SW2_Status=(PORTC->PCR[SW2] & PIN(24))>>24;
	/*===========================================================================================================================*/
	/*Update the corresponding State variables and Clearing Interrupt Status Flags for the corresponding switches for detecting future interrupts */
	/*===========================================================================================================================*/
	if (SW1_Status==1)
	{
		bSW1Pressed=1;      //Update the state variable for Reset Switch (Switch 1)
		PORTC->PCR[SW1] |= PIN(24); //Clearing the Interrupt Status Flag for Switch 1
	}
	if (SW2_Status==1)
	{
		bSW3Pressed=1;  //Update the state variable for Start/Stop Toggle Switch (Switch 2)
		PORTC->PCR[SW2] |= PIN(24); //Clearing the Interrupt Status Flag for Switch 2
	}
// --------------------------------------------------------------------
}
/*========================================================*/
/* Function for I2C Single Byte Read Operation*/
/* Input Arguments:  Device Address, Register Address*/
/* Output Arguments: Register Data*/
/* =======================================================*/
unsigned char I2C_SingleByteRead(unsigned char DEV_ADR, unsigned char REG_ADR)
{   unsigned char DATA_DUMMY =0;unsigned char DATA =0;
	//Set I2C in Transmit mode
	I2C0->C1 |=I2C_C1_TX_MASK;
	// Send Start bit
	I2C0->C1 |=I2C_C1_MST_MASK;
	// Sending Device Address of the Magnetometer and a Write Bit as the last Bit
	I2C0->D = WRITE(DEV_ADR);
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S |= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Sending Register Address of Magnetometer that we want to read
	I2C0->D = REG_ADR;
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S |= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Send the Repeated Start
	I2C0->C1 |= I2C_C1_RSTA_MASK;
	// Sending Device Address of the Magnetometer and a Read Bit as the last Bit
	I2C0->D = READ(DEV_ADR);
	//Sending NAK
	I2C0->C1 |= (I2C_C1_TXAK_MASK);
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
    while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
    //Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	//Set the I2C in Receiver Mode
	I2C0->C1 &= (~I2C_C1_TX_MASK);
	//Read Dummy Magnetometer Data
	DATA_DUMMY = I2C0->D;
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S |= I2C_S_IICIF_MASK;
	//Send Stop Bit
	I2C0->C1 &= (~I2C_C1_MST_MASK);
	// Clear Transmit Nack by setting TXAK to 0
	I2C0->C1 &= ~(I2C_C1_TXAK_MASK);
	//Read Magnetometer Data
	DATA= I2C0->D;
	//delay to ensure all processes are completed before next process starts
	delay();
	//Return Data
	return DATA;
}
/*========================================================*/
/* Function for I2C Multiple Byte Read Operation*/
/* Input Arguments:  Device Address, Register Address of Starting Location ,Number of Bytes to be Read*/
/* Output : Global Variable DATA_READ[] Array Updated with read data*/
/* =======================================================*/
void I2C_MultipleByteRead(unsigned char DEV_ADR,unsigned char REG_ADR, int max_count)
{   unsigned char DATA_DUMMY =0;unsigned char DATA =0;unsigned char data_output[6];
	//Set I2C in Transmit mode
	I2C0->C1|=I2C_C1_TX_MASK;
	// Send Start bit
	I2C0->C1|=I2C_C1_MST_MASK;
	// Sending Device Address of the Magnetometer and a Write Bit as the last Bit
	I2C0->D= WRITE(DEV_ADR);
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Sending Register Address of Magnetometer that we want to read
	I2C0->D=REG_ADR;
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Send the Repeated Start
	I2C0->C1|=I2C_C1_RSTA_MASK;
	// Sending Device Address of the Magnetometer and a Read Bit as the last Bit
	I2C0->D=READ(DEV_ADR);

	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
    while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
    //Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
			{}
	//Set the I2C in Receiver Mode
	I2C0->C1&=(~I2C_C1_TX_MASK);
	//Read Dummy Magnetometer Data
	DATA_DUMMY= I2C0->D;
	for(int count = 0;count < max_count;count++)
			{
		if(count<(max_count-2))
				{//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
				 while((I2C0->S & I2C_S_IICIF_MASK) == 0)
						{}
				 //Clear IICIF Flag
				 I2C0->S|= I2C_S_IICIF_MASK;
				DATA_READ[count]=I2C0->D;
				}
		else
				{
					// Set transfer acknowledgment to NACK to stop the slave transmission
					I2C0->C1|=(I2C_C1_TXAK_MASK);
					//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
					while((I2C0->S & I2C_S_IICIF_MASK) == 0)
							{}
					DATA_READ[count]=I2C0->D;
					count = count+1;
					//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
					while((I2C0->S & I2C_S_IICIF_MASK) == 0)
						{}
					//Clear IICIF Flag
					I2C0->S|= I2C_S_IICIF_MASK;
					// Send the stop signal
					I2C0->C1&=(~I2C_C1_MST_MASK);
					// Clear Transmit Nack by setting TXAK to 0
					I2C0->C1&=(~I2C_C1_TXAK_MASK);
					// Finally read the last byte in the I2C data register
					DATA_READ[count]=I2C0->D;

				}
			}
	//delay to ensure all processes are completed before next process starts
	delay();
			

}
/*========================================================*/
/* Function for I2C Single Byte Write Operation*/
/* Input Arguments:  Device Address, Register Address, Data*/
/* =======================================================*/
void I2C_SingleByteWrite(unsigned char DEV_ADR, unsigned char REG_ADR, unsigned char DATA)
{
	//Set I2C in Transmit mode
	I2C0->C1|=I2C_C1_TX_MASK;
	// Send Start bit
	I2C0->C1|=I2C_C1_MST_MASK;
	// Sending Device Address of the Magnetometer and a Write Bit as the last Bit
	I2C0->D= WRITE(DEV_ADR);
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Sending Register Address of Magnetometer that we want to read
	I2C0->D=REG_ADR;
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Send the single byte of data
	I2C0->D=DATA;
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
    while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
    //Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
			{}
	//Send Stop Bit
	I2C0->C1&=(~I2C_C1_MST_MASK);
	//delay to ensure all processes are completed before next process starts
	delay();
}
/*========================================================*/
/* Function for I2C Multiple Byte Write Operation*/
/* Input Arguments:  Device Address, Register Address of Starting Location ,*/
/* Number of Bytes to be Written,Data Array containing values to be written*/
/* =======================================================*/
void I2C_MultipleByteWrite(unsigned char DEV_ADR, unsigned char REG_ADR, int max_count, unsigned char data_wr[])
{

	//Set I2C in Transmit mode
	I2C0->C1|=I2C_C1_TX_MASK;
	// Send Start bit
	I2C0->C1|=I2C_C1_MST_MASK;
	// Sending Device Address of the Magnetometer and a Write Bit as the last Bit
	I2C0->D= WRITE(DEV_ADR);
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Sending Register Address of Magnetometer that we want to read
	I2C0->D=REG_ADR;
	//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
		{}
	//Clear IICIF Flag
	I2C0->S|= I2C_S_IICIF_MASK;
	//Waiting for Acknowledgement from slave
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
		{}
	// Send the multiple bytes of data
	for(int count = 0;count < max_count;count++)
		{
		I2C0->D=data_wr[count];
		//Wait for Transfer to complete i.e. until IICIF Flag is set to 0
    	while((I2C0->S & I2C_S_IICIF_MASK) == 0)
			{}
    	//Clear IICIF Flag
    	I2C0->S|= I2C_S_IICIF_MASK;
    	//Waiting for Acknowledgement from slave
    	while ((I2C0->S & I2C_S_RXAK_MASK) != 0)
			{}
		}
	//Send Stop Bit
	I2C0->C1&=(~I2C_C1_MST_MASK);
	//delay to ensure all processes are completed before next process starts
	delay();
}
/*========================================================*/
/* Interfacing the Magnetometer*/
/* =======================================================*/

/*========================================================*/
/* Function to initialise the Magnetometer*/
/* =======================================================*/
void Magnetometer_Init(void)
// Initialize local variables here
{	unsigned char MAG_DEVICE_ID;int flag_device_id_match = 0;unsigned char control_reg1;unsigned char control_reg2;
    /*========================================================*/
	/* Configuring the settings for the Magnetometer*/
	/*
	Use 16-bit full-resolution mode for output data (X, Y and Zaxis)
    Set to ‘continuous measurements mode’ with ‘ODR=80Hz’ and ‘OSR=1’
	Set to ACTIVE Mode in ’CTRL_REG1’ register
	*/
	/* =======================================================*/
	unsigned char DATA_WRITE[] = {0x01,0xA0};
	MAG_DEVICE_ID = I2C_SingleByteRead(MAG_DEVICE_ADDRESS, MAG_DEVICE_ID_REGISTER_ADDRESS);
	/*checking the device ID (‘0xC4’)of magnetometer */
	while(MAG_DEVICE_ID != 0xC4)
		/*If the device ID is inaccessible or incorrect, the function should return an error.*/
    	{SLCD_WriteMsg((unsigned char *)"Err");
    	}
		flag_device_id_match = 1;
		control_reg1=I2C_SingleByteRead(MAG_DEVICE_ADDRESS,MAG_CTRL_REG1);

		if(control_reg1 != 0x01)
		/*Writing corresponding data to control registers to configure settings(enable the active mode with ODR=80Hz, OSR=1)*/
		{
			I2C_MultipleByteWrite(MAG_DEVICE_ADDRESS,MAG_CTRL_REG1,2,DATA_WRITE);

		}


}
/*========================================================*/
/* Function to acquire the Magnetometer data and find maximum and minimum values*/
/* =======================================================*/
void Magnetometer_Acq(void)
{	// Initialize local variables here
	int loop_count = 0;
	// Reading data from magnetometer along X, Y, Z axes
	I2C_MultipleByteRead(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, 6);
	// Combining upper and lower bytes of data for each axis to get complete data along each axis
	for(loop_count=0;loop_count<3;loop_count++)
	{
		MAG_DATA_READ_AXIS[loop_count]=((short int)((DATA_READ[2*loop_count]<<8)|DATA_READ[2*loop_count+1]));
	}
	// Find the maximum and minimum values along X, Y, Z axes for calibration purposes
	for(loop_count=0;loop_count<3;loop_count++)
	{
		if(flag_first_data == 0)
		{   //Set the first reading to be the maximum and minimum value along each axis
			MAG_DATA_MAX_AXIS[loop_count]=MAG_DATA_READ_AXIS[loop_count];
			MAG_DATA_MIN_AXIS[loop_count]=MAG_DATA_READ_AXIS[loop_count];
			flag_first_data = 1;
			//Finding the Maximum value along each axis
		}else if (MAG_DATA_READ_AXIS[loop_count] > MAG_DATA_MAX_AXIS[loop_count])
		{
			MAG_DATA_MAX_AXIS[loop_count] = MAG_DATA_READ_AXIS[loop_count];
			//Finding the Minimum value along each axis
		}else if (MAG_DATA_READ_AXIS[loop_count] < MAG_DATA_MIN_AXIS[loop_count])
		{
			MAG_DATA_MIN_AXIS[loop_count]=MAG_DATA_READ_AXIS[loop_count];
		}
	}
}
/*========================================================*/
/* Function to calibrate the Magnetometer for Hard Iron losses*/
/* =======================================================*/
void Magnetometer_Cal(void)
{/*Calibrate the magnetometer for hard-iron effects based on the maximum and minimum X, Y, and Z-axis data
	found during ‘MAG_ACQ’ mode */
for(int loop_count=0;loop_count<3;loop_count++)
	{   MAG_DATA_AVERAGE_AXIS[loop_count]=(MAG_DATA_MAX_AXIS[loop_count]+MAG_DATA_MIN_AXIS[loop_count])/2;
	}
}
/*========================================================*/
/* Function to acquire the correct Magnetometer data after calibration and also calculate the angle for e-compass 
i.e the angle subtended wrt magnetic north pole*/
/* =======================================================*/
void Magnetometer_Run(void)
{
	// Initialize local variables here
	int loop_count = 0;float TEMP_ANGLE1;short int TEMP_ANGLE;
	// Reading data from magnetometer along X, Y, Z axes
	I2C_MultipleByteRead(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, 6);
	// Combining upper and lower bytes of data for each axis to get complete data along each axis
	for(loop_count=0;loop_count<3;loop_count++)
		{
			MAG_DATA_READ_AXIS[loop_count]=((short int)((DATA_READ[2*loop_count]<<8)|DATA_READ[2*loop_count+1]));
		}
	/*Calculating the calibrated magnetometer data by subtracting the data offset from the raw uncalibrated data*/
	for(loop_count=0;loop_count<3;loop_count++)
	{
		MAG_DATA_HI_CALIBRATED[loop_count]=MAG_DATA_READ_AXIS[loop_count]-MAG_DATA_AVERAGE_AXIS[loop_count];
	}
	/*calculate the angle for e-compass i.e the angle subtended wrt magnetic north pole*/
	if((MAG_DATA_HI_CALIBRATED[1]==0)&&(MAG_DATA_HI_CALIBRATED[0]<0))
		{
		ANGLE = 180;
		}
	else if((MAG_DATA_HI_CALIBRATED[1]==0)&&(MAG_DATA_HI_CALIBRATED[0]>0))
		{
		ANGLE = 0;
		}
	else if(MAG_DATA_HI_CALIBRATED[1]>0)
		{
		ANGLE = 90-(atan(((double)MAG_DATA_HI_CALIBRATED[0]/(double)MAG_DATA_HI_CALIBRATED[1]))*57.29);
		}
	else if(MAG_DATA_HI_CALIBRATED[1]<0)
		{
		ANGLE = 270-(atan(((double)MAG_DATA_HI_CALIBRATED[0]/(double)MAG_DATA_HI_CALIBRATED[1]))*57.29);
		}
}

void main(void)
{
// --------------------------------------------------------------------
// Place your local variable(s) here - Start

// Place your local variable(s) here - End
// --------------------------------------------------------------------

    /*========================================================*/
    /*========================================================*/
    /*   Initialization                                       */
    /*========================================================*/
    /* Disable global interrupt */
    __disable_irq();

    /* Peripheral initialization */
    SLCD_Init();
    /*========================================================*/
    while(1){
      /* State transition upon a switch-press */
      // Check if SW3 is pressed
        if(bSW3Pressed == true){
          // Clear the flag
            bSW3Pressed = false;
            if(enumECompassState == STOP){
                RED_LED_OFF();
                GREEN_LED_OFF();
                enumECompassState = MAG_ACQ;
            }else if(enumECompassState == MAG_ACQ){
                enumECompassState = MAG_CAL;
            }else if(enumECompassState == MAG_CAL){
                RED_LED_OFF();
                GREEN_LED_OFF();
                enumECompassState = RUN;
            }
        // Check if SW1 is pressed
        }else if(bSW1Pressed == true){
          // Clear the flag
            bSW1Pressed = false;
            if(enumECompassState == STOP){
                // Nothing to be done
            }else if(enumECompassState == MAG_ACQ){
                // Nothing to be done
            }else if(enumECompassState == MAG_CAL){
                // Nothing to be done
            }else if(enumECompassState == RUN){
				//Stop if SW1 is pressed
                enumECompassState = STOP;
            }
        }
        /* Carry out the given tasks defined in the current state */
        if(enumECompassState == STOP){
            // (Re)initialize variables
        	/* Peripheral initialization */
        	    LED_Init();
        	    SWITCH_Init();
        	    TIMER_Init();
				I2C0_Init();
        	    /* Enable individual interrupt */
        	    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
        	    NVIC_EnableIRQ(TPM0_IRQn);

        	    /* Enable global interrupt */
        	    __enable_irq();
        	    //Display STOP Message
        	    SLCD_WriteMsg((unsigned char *)"STOP");

        }else if(enumECompassState == RUN){
			//The e-compass heading is updated every 100ms using the timer IRQ
        	if(bIsTimerExpired == true)
            	{float Z_ACC;
			     short int z_acc_sensitivity = 4096;
			     short int g_lower_limit = 0.9*z_acc_sensitivity;
				 short int g_upper_limit = 1.1*z_acc_sensitivity;
			    //clear bIsTimerExpired Flag 
				bIsTimerExpired = false;
				//Polling the DR_Status register data to read Magnetometer whenever new data is available 
            	DR_STATUS_DATA_ACC = I2C_SingleByteRead(MAG_DEVICE_ADDRESS,MAG_DR_STATUS);
            	if((DR_STATUS_DATA_ACC!=0)||((flag_first_data_run ==0)))
					//Update Magnetometer Data
            		{Magnetometer_Run();flag_first_data_run =1;
            		// Displaying current magnetometer heading angle wrt magnetic north on LCD in degrees
            		snprintf(ucaryLCDMsg,5,"%4d",ANGLE);
            		SLCD_WriteMsg(ucaryLCDMsg);
					/* The green LED should be turned on between 345° to 15° (± 15° tolerance) as the e-compass is heading towards
					north magnetic pole (Otherwise, the green LED should be turned off).*/
            		if(((ANGLE >= 0)&&(ANGLE <= 15))||((ANGLE >= 345)&&(ANGLE <= 360)))
            			{
            			GREEN_LED_ON();
            			}
            		else
            			{
            			GREEN_LED_OFF();
            			}
            		}
            	}
        }else if(enumECompassState == MAG_ACQ){
            if(flag_initialisation == 0)
            {   //initialise the Magnetometer once
            	Magnetometer_Init();
            	flag_initialisation =1;
            }
			//Polling the DR_Status register data to read Magnetometer whenever new data is available 
            DR_STATUS_DATA = I2C_SingleByteRead(MAG_DEVICE_ADDRESS,MAG_DR_STATUS);
            if((DR_STATUS_DATA!=0)||((flag_first_data_acquire ==0)))
               	{//Acquire Magnetometer data for calibration purpose
            	 Magnetometer_Acq();
            	 flag_first_data_acquire =1;
            	 }
			//Display MACQ Message
        	SLCD_WriteMsg((unsigned char *)"MACQ");

        }else if(enumECompassState == MAG_CAL){
			//Calibrate Magnetometer
        	Magnetometer_Cal();
			//Display MCAL Message
        	SLCD_WriteMsg((unsigned char *)"MCAL");


        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
