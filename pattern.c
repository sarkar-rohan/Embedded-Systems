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
 * LOSS OF USE, status, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*=======================================================================================*/
/*Program to Implement a Pattern Function with Switches and LEDs
Input provided through switches SW1(connected to PTC3) and SW3(connected to PTC12)
Output observed through Red LED(connected to PTE29) and Green Led(connected to PTD5)
Author: Rohan Sarkar
*/
/*=======================================================================================*/
#include "MKL46Z4.h"
/*========================================================*/
/* Define PIN Function */
/* ======================================================== */
#define PIN(x)                 (1 << x)
/*========================================================*/
/* Green LED */
/* ======================================================== */
// Green LED is connected to PTD5 and define functions for Green LED
#define GREEN_LED             (5)
#define GREEN_LED_ON()       PTD->PCOR |= PIN(GREEN_LED) ;// Define function to turn Green LED ON
#define GREEN_LED_OFF()      PTD->PSOR |= PIN(GREEN_LED) ;// Define function to turn Green LED OFF
#define GREEN_LED_TOGGLE()   PTD->PTOR |= PIN(GREEN_LED) ;// Define function to TOGGLE Green LED
/*========================================================*/
/* Red LED */
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
//Main Function to call program
void main(void){
//Declare and initialise the status variables for SW1 and SW2.
int sw1_status = 0; int sw2_status =0;
 /*========================================================*/
 /*   Turn on clock to Port C, D and E module respectively*/
 /* ======================================================== */
SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
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
/*========================================================*/
/*Set configuration for Switch 1 and Switch 2 (Input Mode)*/
/* ======================================================== */
/*Set the PTC3 pin multiplexer to GPIO mode and enable pull up resistor for the corresponding pin*/
PORTC->PCR[SW1] = PORT_PCR_MUX(1)|ENABLE_PULLUP_RESISTOR ;
/*Set the PTC12 pin multiplexer to GPIO mode and enable pull up resistor for the corresponding pin*/
PORTC->PCR[SW2] = PORT_PCR_MUX(1)|ENABLE_PULLUP_RESISTOR ;
/*Set the pins direction to input (Set the corresponding bit to 0) */
PTC->PDDR &= (~PIN(SW1))&(~PIN(SW2));
while(1) //using a while loop with argument 1 so that the program runs continuously
{
/*========================================================*/
/* Read Status of Switch 1 and Switch 2 Inputs*/
/* Since I have enabled pull up resistor for the input pins: 1 signifies Switch is OFF(Switch Released) and 0 signifies Switch is ON(Switch Pushed Down)*/
/* ======================================================== */
//Read the SW1 status by masking and then doing right shift to the LSB and thus convert the switch status to a single digit status( 0 or 1).
 sw1_status = ((PTC->PDIR & PIN(SW1))>>SW1);
//Read the SW2 status by masking and then doing right shift to the LSB and thus convert the switch status to a single digit status( 0 or 1).
 sw2_status = ((PTC->PDIR & PIN(SW2))>>SW2);
 /*========================================================*/
 /*Controlling the Pattern Function on LED based on Switch Inputs*/
 /* ======================================================== */
//Switch the Green LED and Red LED OFF as default case.
 GREEN_LED_OFF();
 RED_LED_OFF();
//Switch the Green LED and Red LED OFF if both switches are ON.
 if ((sw1_status == 0) & (sw2_status == 0))
	 {GREEN_LED_OFF();
	 RED_LED_OFF();}
 //Switch the Green LED OFF and Red LED ON if Switch 1 is OFF and Switch 2 is ON.
 if ((sw1_status == 1) & (sw2_status == 0))
 	 {GREEN_LED_OFF();
 	 RED_LED_ON();}
 //Switch the Green LED ON and Red LED OFF if Switch 1 is ON and Switch 2 is OFF.
 if ((sw1_status ==0) & (sw2_status == 1))
 	 {GREEN_LED_ON() ;
 	 RED_LED_OFF();}
 //Switch the Green LED and Red LED OFF if both switches are OFF.
 if ((sw1_status == 1) & (sw2_status == 1))
 	 {GREEN_LED_OFF();
 	 RED_LED_OFF();}
}
}
