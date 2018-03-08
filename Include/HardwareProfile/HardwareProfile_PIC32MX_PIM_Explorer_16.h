// Copyright (c) 2002-2010,  Microchip Technology Inc.
//
// Microchip licenses this software to you solely for use with Microchip
// products.  The software is owned by Microchip and its licensors, and
// is protected under applicable copyright laws.  All rights reserved.
//
// SOFTWARE IS PROVIDED "AS IS."  MICROCHIP EXPRESSLY DISCLAIMS ANY
// WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL
// MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
// CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR
// EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY
// OR SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED
// TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION,
// OR OTHER SIMILAR COSTS.
//
// To the fullest extent allowed by law, Microchip and its licensors
// liability shall not exceed the amount of fees, if any, that you
// have paid directly to Microchip to use this software.
//
// MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
// OF THESE TERMS.
#ifndef __HARDWAREPROFILE_PIC32MX_PIM_EXPLORER_16_H__
#define __HARDWAREPROFILE_PIC32MX_PIM_EXPLORER_16_H__

//rpr changes made for spacelogger wifi hardware
/*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #define self_power          1

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISBbits.TRISB5    // Input
    #define USB_BUS_SENSE       1 

#if defined(__PIC32MX1XX_2XX__)
		/* The 44 pins of PIC32MX1xx/2xx PIM are not mapped to any of the LEDs/Switches 
		of Explorer-16 board. User has to modify the LED and switch mapping as required.	
		*/
        #error "If you are compiling this project for PIC32MX1xx/2xx devices, please read the supplied document pic321xx_2xxx_support.htm and then proceed"

		#define mLED              
	    // Blinks LED 5 on explorer 16 board
	    #define BlinkLED() 
	    #define InitLED() 
	    // Switch ON all the LEDs to indicate Error.
	    #define Error()   
	
	    // Switch S3 on Explorer 16.
	    #define ReadSwitchStatus() 1
#elif defined(__PIC32MX3XX_7XX__)
//i/o defines from spacelogger WiFi user.h
#define LED1_TRIS			(TRISEbits.TRISE0)	//LED1 left green
#define LED1_IO				(LATEbits.LATE0)
#define LED1_ON                         LED1_IO=1u
#define LED1_OFF                        LED1_IO=0u
#define LED1_TOGGLE                     (PORTEINV=0x00001u)

#define LED2_TRIS			(TRISEbits.TRISE1)	//LED2 left red
#define LED2_IO				(LATEbits.LATE1)
#define LED2_ON                         LED2_IO=1u
#define LED2_OFF                        LED2_IO=0u
#define LED2_TOGGLE                     (PORTEINV=0x00000002u)

#define LED3_TRIS			(TRISEbits.TRISE2)	//LED3 right green
#define LED3_IO				(LATEbits.LATE2)
#define LED3_ON                         LED3_IO=1u
#define LED3_OFF                        LED3_IO=0u
#define LED3_TOGGLE                     (PORTEINV=0x00000004u)


#define LED4_TRIS			(TRISEbits.TRISE3)	//LED4 right red
#define LED4_IO				(LATEbits.LATE3)
#define LED4_ON                         LED4_IO=1u
#define LED4_OFF                        LED4_IO=0u

// Momentary push buttons
//  RE5     1   I/O ST  SW1     push switch input
#define SW1_TRIS                        (TRISEbits.TRISE5)	//SW1
#define SW1_IO                          (PORTEbits.RE5)

//  SD card switches
//  RE4     64  I/O ST  SD_CARD_WP input
#define SD_WP_TRIS                      (TRISEbits.TRISE4)	//SD_CARD_WP
#define SD_WP_IO                        (PORTEbits.RE4)
#define SD_WE_TRIS                      SD_WP_TRIS
#define SD_WE                           SD_WP_IO
//  RE7     3   I/O ST  SD_CARD_CD input
#define SD_CD_TRIS                      (TRISEbits.TRISE7)	//SD_CARD_CD
#define SD_CD_IO                        (PORTEbits.RE7)
#define SD_CD                           SD_CD_IO
//  SD card interface
#define SD_CS_TRIS                      (TRISBbits.TRISB5)
#define SD_CS_IO                        (LATBbits.LATB5)
#define SD_CS                           SD_CS_IO
#define SD_SDI_TRIS                     (TRISGbits.TRISG7)
#define SD_SCK_TRIS                     (TRISGbits.TRISG6)
#define SD_SDO_TRIS                     (TRISGbits.TRISG8)
#define SD_PWR_TRIS			(TRISGbits.TRISG9)	//SD power - low to shut down
#define SD_PWR_IO                       (LATGbits.LATG9)
#define SD_PWR_ON                       (SD_PWR_IO=1u)
#define SD_PWR_OFF                      (SD_PWR_IO=0u)
// RS232
#define RS232_SHDN_TRIS                 (TRISBbits.TRISB15)
#define RS232_EN_TRIS                   (TRISBbits.TRISB13)
#define RS232_SHDN_IO                   (LATBbits.LATB15)   //0 to shut down RS232 chip
#define RS232_EN_IO                     (LATBbits.LATB13)   //0 to enable receive buffers
#define RS232_TX_TRIS                   (TRISFbits.TRISF5)
#define RS232_RX_TRIS                   (TRISFbits.TRISF4)
#define RS232_RTS_TRIS                  (TRISBbits.TRISB14) //RTS output
#define RS232_RTS_IO                    (LATBbits.LATB14)
#define RS232_RTS_MASK                  0x4000u
#define RS232_RTS_SET                   (PORTBSET=RS232_RTS_MASK)
#define RS232_RTS_CLR                   (PORTBCLR=RS232_RTS_MASK)
#define RS232_CTS_TRIS                  (TRISBbits.TRISB8)  //CTS input
#define RS232_CTS_IO                    (PORTBbits.RB8)


//  WiFi module
//  RB4     12      WIFI_PWR output low to shut down
#define WF_PWR_TRIS                     (TRISBbits.TRISB4)
#define WF_PWR_IO                       (LATBbits.LATB4)
#define WF_PWR_ON                       (WF_PWR_IO=1u)
#define WF_PWR_OFF                      (WF_PWR_IO=0u)
//  RB7     18      WIFI_WP should be low to write-protect module flash
#define WF_WP_TRIS                      (TRISBbits.TRISB7)
#define WF_WP_IO                        (LATBbits.LATB7)
#define WF_WP_ON                        (WF_WP_IO=0u)           //write-protect on
#define WF_WP_OFF                       (WF_WP_IO=1u)           //write-protect off
#define WF_CS_TRIS                      (TRISBbits.TRISB11)
#define WF_CS_IO                        (LATBbits.LATB11)

//buzzer RD10 & RD11
#define BUZZER1_TRIS                    (TRISDbits.TRISD10)
#define BUZZER1_IO                      (PORTDbits.RD10)
#define BUZZER2_TRIS                    (TRISDbits.TRISD11)
#define BUZZER2_IO                      (PORTDbits.RD11)
#define BUZZER1_MASK                    0x0400u
#define BUZZER2_MASK                    0x0800u
#define BUZZER_MASK                     0x0C00u
#define BUZZER1_SET                     (PORTDSET=BUZZER1_MASK)
#define BUZZER1_CLR                     (PORTDCLR=BUZZER1_MASK)
#define BUZZER2_SET                     (PORTDSET=BUZZER2_MASK)
#define BUZZER2_CLR                     (PORTDCLR=BUZZER2_MASK)
#define BUZZER_CLR                      (PORTDCLR=BUZZER_MASK)
#define BUZZER_TOGGLE                   (PORTDINV=BUZZER_MASK)

//  EEPROM write-protect
#define EEPROM_WP_TRIS                  (TRISFbits.TRISF1)
#define EEPROM_WP_IO                    (LATFbits.LATF1)

//	#define mLED              LATAbits.LATA2
	#define mLED              LED1_IO
	// Blinks LED 5 on explorer 16 board
	#define BlinkLED() (mLED = ((ReadCoreTimer() & 0x0800000) == 0))
//	#define InitLED() do{DDPCON = 0; TRISA = 0x00; LATA= 0;}while(0)
	#define InitLED() do{DDPCON = 0; LED1_TRIS = 0; LED1_OFF; LED2_TRIS = 0; LED2_OFF; LED3_TRIS = 0; LED3_OFF; LED4_TRIS = 0; LED4_OFF;}while(0)
	// Switch ON red LEDs to indicate Error.
	#define Error()   do{LED2_ON; LED4_ON;}while(0)
	
	// Switch S3 on Explorer 16.
//	#define ReadSwitchStatus() (PORTReadBits(IOPORT_D, BIT_6) & BIT_6)
        //use card detect switch and lock
//	#define ReadSwitchStatus() ((!SD_CD_IO) & SD_WP_IO)
	#define ReadSwitchStatus() (SD_WP_IO)

#endif
	
	
	#ifdef TRANSPORT_LAYER_SD_CARD
		// Define following only for SD card bootloader.
		#define USE_SD_INTERFACE_WITH_SPI
	#endif

#endif
