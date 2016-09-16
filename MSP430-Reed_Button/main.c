/**********************************************************************************/
/*    Demo program for:								  */
/*	  Board: MSP430-PIR     						  */
/*    Manufacture: OLIMEX                                                   	  */
/*	  COPYRIGHT (C) 2011							  */
/*    Designed by: Engineer Penko T. Bozhkov                                      */
/*    Module Name    :  main module                                               */
/*    File   Name    :  main.c                                                    */
/*    Revision       :  initial                                                   */
/*    Date           :  08.02.2011                                                */
/*    Built with IAR Embedded Workbench Version: 4.21                             */
/**********************************************************************************/
#include  <intrinsics.h>
#include  <msp430x20x3.h>

#define _BIC_SR(reg) __bic_SR_register(reg)
#define _BIS_SR(reg) __bis_SR_register(reg)

#define REED   BIT0
#define BUTTON   BIT4
#define SCLK   BIT5
#define SDO   BIT6
#define SDI   BIT7

#define SCLK_ON                 P1OUT |= SCLK; // Set P1.0 HIGH
#define SCLK_OFF                P1OUT &= ~SCLK; //P1DIR |= 0x01;
#define SCLK_Check              (P1IN & SCLK)

#define SDO_ON                 P1OUT |= SDO; // Set P1.0 HIGH
#define SDO_OFF                P1OUT &= ~SDO; //P1DIR |= 0x01;
#define SDO_Check              (P1IN & SDO)

#define SDI_ON                 P1OUT |= SDI; // Set P1.0 HIGH
#define SDI_OFF                P1OUT &= ~SDI; //P1DIR |= 0x01;
#define SDI_Check              (P1IN & SDI)

#define Button_Check              (P1IN & BUTTON)
#define Reed_Check              (P1IN & REED)

unsigned int Button_Counter;

/**********************************************************************************/
/*  Function name: Delay                                                          */
/*  	Parameters                                                                */
/*          Input   :  No	                                                  */
/*          Output  :  No	                                                  */
/*	Action: Simple delay							  */
/**********************************************************************************/
void Delay(volatile unsigned int delay_counter){
	while(delay_counter){delay_counter--;}
}


/**********************************************************************************/
/*  Function name: Ports_initial_initialization                                   */
/*  	Parameters                                                                */
/*          Input   :  No	                                                  */
/*          Output  :  No	                                                  */
/*	Action: Define initial ports state and directions.			  */
/**********************************************************************************/
void Ports_initial_initialization(void)
{
    P1SEL &= (~SCLK); // Set P1.5 SEL for GPIO
    P1DIR |= SCLK; // Set P1.5 as Output
    P1SEL &= (~SDO); // Set P1.5 SEL for GPIO
    P1DIR |= SDO; // Set P1.5 as Output
    P1SEL &= (~SDI); // Set P1.5 SEL for GPIO
    P1DIR |= SDI; // Set P1.5 as Output
    // Button setup
    P1DIR &= ~BUTTON;                     // button is an input
    P1OUT |= BUTTON;                      // pull-up resistor
    P1REN |= BUTTON;                      // resistor enabled
    P1IES |= BUTTON;                      // interrupt on low-to-high transition
    P1IE |= BUTTON;                       // interrupt enable
    // Reed setup
    P1DIR &= ~REED;                     // button is an input
    P1OUT |= REED;                      // pull-up resistor
    P1REN |= REED;                      // resistor enabled
    P1IES |= REED;                      // interrupt on low-to-high transition
    P1IE |= REED;                       // interrupt enable
    

}


/**********************************************************************************/
/*  Function name: init_devices	                                                  */
/*  	Parameters                                                                */
/*          Input   :  No			     		                  */
/*          Output  :  No                                      	            	  */
/*	Action: Initialize all used MSP430F2274 peripheral devices.	  	  */
/**********************************************************************************/
void init_devices(void){
  _BIC_SR(GIE); // Disable interrupts during initialization process
  
  /********** 1.Ports initialization ***********/
  Ports_initial_initialization();
  /********** 2.Select System Clock  ***********/
  // By default System Clock is 1.1MHz DCO Clock
  
  /********** 3.Peripherals initialization ***********/
  // 3.1. Timer_A
  // =============
  TACTL = 0x0004;   // Timer_A clear.
  TACCTL0 = 0x0010; // Timer_A Capture/compare interrupt enable.
  TACCR0 = 0x007F;  // Set TACCR0 value
  TACTL = 0x02D0;   // Selected: SMCLK, Divider 1:8, Up mode.
  //TAIV = 0x000A;

  _BIS_SR(GIE); // Global Interrupt enabled. Do this at the END of the initialization!!!!!!!!
}


/**********************************************************************************/
/*  Function name: main	                                                  	  */
/*  	Parameters                                                                */
/*          Input   :  No			     		                  */
/*          Output  :  No                                      	            	  */
/*	Action: Call "init_devices".                    	  		  */
/**********************************************************************************/
void  main(void){
  WDTCTL = 0x5A80;              // Stop WDT
  init_devices();
  while(1){
    
  }
}


/**********************************************************************************/
/*  Function name: TIMERA_Capture_Compare_ISR                                  	  */
/*  	Parameters                                                                */
/*          Input   :  No			     		                  */
/*          Output  :  No                                      	            	  */
/*	Action:  Toggle SCLK and LED2 with frequency ~ 1Hz.                       */
/**********************************************************************************/
#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA_Capture_Compare_ISR(void){
  // Scan Buttons
  if(!Button_Check){      // Then BUT is pressed
    SCLK_OFF;
    SDI_OFF;
  }
  else{
    SCLK_ON;
    SDI_ON;
  }
  
  if(!Reed_Check){      // Then BUT is pressed
    SCLK_OFF;
    SDO_OFF;
  }
  else{
    SCLK_ON;
    SDO_ON;
  }
}

