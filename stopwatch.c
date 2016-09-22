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
/* Implementation of Stopwatch using Timer and Interrupts  */
/* Output in the format of M:SS:m*/
/* Author: Rohan Sarkar                                    */
/* ======================================================== */
#include "MKL46Z4.h"
#include "slcd.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// -----------------------------------------------------------------
/*========================================================*/
/* Define Macros */
/* ======================================================== */
/*========================================================*/
/* Define PIN Function */
/* ======================================================== */
#define PIN(x)                 (1 << x)
/*========================================================*/
/* Green LED*/
/* ======================================================== */
// Green LED is connected to PTD5 and define functions for Green LED
#define GREEN_LED             (5)
#define GREEN_LED_ON()       PTD->PCOR |= PIN(GREEN_LED) ;// Define function to turn Green LED ON
#define GREEN_LED_OFF()      PTD->PSOR |= PIN(GREEN_LED) ;// Define function to turn Green LED OFF
#define GREEN_LED_TOGGLE()   PTD->PTOR |= PIN(GREEN_LED) ;// Define function to TOGGLE Green LED
/*========================================================*/
/* Red LED*/
/* ======================================================== */
// Red LED is connected to PTE29 and define functions for Red LED
#define RED_LED             (29)
#define RED_LED_ON()       PTE->PCOR |= PIN(RED_LED) ; //Define function to turn Red LED ON
#define RED_LED_OFF()      PTE->PSOR |= PIN(RED_LED) ; //Define function to turn Red LED OFF
#define RED_LED_TOGGLE()   PTE->PTOR |= PIN(RED_LED) ; //Define function to TOGGLE Red LED
/*========================================================*/
/* Define SWitch 1(SW1) and Switch 2(SW3) */
/* ======================================================== */
// SW1 is connected to PTC3
#define SW1             (3)
// SW2 is connected to PTC12
#define SW2             (12)
// This is used to set the internal pull up resistor of the corresponding pin.
// Setting PE = 1 and PS = 1 for the corresponding pin to which the switches are connected.
#define ENABLE_PULLUP_RESISTOR  (3)
// -----------------------------------------------------------------

typedef enum {
	STOP,
	RUN,
	PAUSED
} enumStopWatchOperationState;

enumStopWatchOperationState enumStopWatchState = STOP;

bool bStartStopSwitchPressed = false;
bool bResetSwitchPressed     = false;

bool bIsTimerExpired         = false;

unsigned char    ucSecond = 0;
unsigned char    ucHundredsMilliSecond = 0;
unsigned char    ucMinute = 0;
unsigned short   usTimeElapsed = 0;

unsigned char    ucaryLCDMsg[5] = "";

// --------------------------------------------------------------------
// Defining variables to be used for calculation of TPM0 MOD Count for Timer Overflow every 0.1 second
int prescalar;
int prescalar_factor;//to be
int resolution;
int original_clock_frequency; //in kHz
int new_clock_frequency ;
int timer_overflow_count;
int reset_count;
// --------------------------------------------------------------------

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
		bResetSwitchPressed=1;      //Update the state variable for Reset Switch (Switch 1)
		PORTC->PCR[SW1] |= PIN(24); //Clearing the Interrupt Status Flag for Switch 1
	}
	if (SW2_Status==1)
	{
		bStartStopSwitchPressed=1;  //Update the state variable for Start/Stop Toggle Switch (Switch 2)
		PORTC->PCR[SW2] |= PIN(24); //Clearing the Interrupt Status Flag for Switch 2
	}
// --------------------------------------------------------------------
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
    LED_Init();
    SWITCH_Init();
    TIMER_Init();

    /* Enable individual interrupt */
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(TPM0_IRQn);

    /* Enable global interrupt */
    __enable_irq();

    /*========================================================*/
    while(1){
      /* State transition upon a switch-press */
      // Check if SW3 is pressed
        if(bStartStopSwitchPressed == true){
          // Clear the flag
            bStartStopSwitchPressed = false;
            if(enumStopWatchState == STOP){
                // Turn off the red LED
                RED_LED_OFF();
                enumStopWatchState = RUN;
            }else if(enumStopWatchState == RUN){
                enumStopWatchState = PAUSED;
            }else if(enumStopWatchState == PAUSED){
                RED_LED_OFF();
                enumStopWatchState = RUN;
            }
        // Check if SW1 is pressed
        }else if(bResetSwitchPressed == true){
          // Clear the flag
            bResetSwitchPressed = false;
            if(enumStopWatchState == STOP){
                // Nothing to be done
            }else if(enumStopWatchState == RUN){
                // Nothing to be done
            }else if(enumStopWatchState == PAUSED){
                enumStopWatchState = STOP;
            }
        }
        /* Carry out the given tasks defined in the current state */
        if(enumStopWatchState == STOP){
            // (Re)initialize variables
            ucSecond = 0;
            ucHundredsMilliSecond = 0;
            ucMinute = 0;
            usTimeElapsed = 0;
            // Write a message on the LCD
            SLCD_WriteMsg((unsigned char *)"STOP");
            // The red LED is turned on while the stopwatch is in standby
            RED_LED_ON();
        }else if(enumStopWatchState == RUN){
            // Check if timer is expired
            if(bIsTimerExpired == true){
                // Clear the flag
                bIsTimerExpired = false;
                // Increment the variable that takes care of hundreds-milliseconds
                ucHundredsMilliSecond++;
            }
            //  10 * 100 ms = 1 s
            if(ucHundredsMilliSecond == 10){
                // The variable for hundreds-milliseconds rolls over back to zero
                ucHundredsMilliSecond = 0;
                // Increment the variable that takes care of seconds
                ucSecond++;
                // Toggling the green LED every second
                GREEN_LED_TOGGLE();
            }
            // 1 min = 60 s
            if(ucSecond == 60){
                // The variable for seconds rolls over back to zero
                ucSecond = 0;
                // Increment the variable that takes care of minutes
                ucMinute++;
            }
            // 10 * 1 min = 10 min
            if(ucMinute == 10){
                // The variable for minutes rolls over back to zero due to limited digits in the LCD
                ucMinute = 0;
            }
            // The red LED is turned off indicating stopwatch is in operation
            RED_LED_OFF();
            // Combine the time-related subcomponents for LCD visualization
            usTimeElapsed = ucMinute * 1000 + ucSecond * 10 + ucHundredsMilliSecond;
            // Convert integer to string
            snprintf(ucaryLCDMsg, 5,"%4d",usTimeElapsed);
            // Write elapsed time on the LCD
            SLCD_WriteMsg(ucaryLCDMsg);
        }else if(enumStopWatchState == PAUSED){
            // Make sure the green LED is turned off
            GREEN_LED_OFF();
            // Check if timer is expired
            if(bIsTimerExpired == true){
                // Clear the flag
                bIsTimerExpired = false;
                // Toggling the red LED to indicate the stopwatch is paused
                RED_LED_TOGGLE();
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
