/********************************************************************
 FileName:     SD Bootloader.c
 Dependencies: See INCLUDES section
 Processor:		PIC32 USB Microcontrollers
 Hardware:		
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Description
  1.0   Initial release
  2.1   Updated for simplicity and to use common
                     coding style
********************************************************************/
#include "FSIO.h"
#include "NVMem.h"
#include "sd_bootloader.h"
#include "Bootloader.h"
#include <plib.h>
#include <string.h>
#include "HardwareProfile.h"

// *****************************************************************************
// *****************************************************************************
// Device Configuration Bits (Runs from Aux Flash)
// *****************************************************************************
// *****************************************************************************
// Configuring the Device Configuration Registers
// 80Mhz Core/Periph, Pri Osc w/PLL, Write protect Boot Flash
//#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
//#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
//#pragma config ICESEL = ICS_PGx2, BWP = OFF


//the following are copied from configuration_bits.c of the spacelogger wifi application
//PLL input divider - 1xDivider

#pragma config FPLLODIV = DIV_1
//PLL Multiplier - 20xMultiplier
#pragma config FPLLMUL = MUL_20
//System PLL Output Clock Divider - PLL divide by 2
#pragma config FPLLIDIV = DIV_2
//Oscillator Selection Bits - Fast RC Osc with PLL
#pragma config FNOSC = FRCDIV16 // FRCPLL
//Secondary Oscillator Enable - Enabled
#pragma config FSOSCEN = ON
//Internal/External Switch Over - Disabled
#pragma config IESO = OFF
//Primary Oscillator Configuration - Primary osc disabled
#pragma config  POSCMOD = OFF
//CLKO Output Signal Active on the OSCO Pin - Disabled
#pragma config OSCIOFNC = OFF
//Peripheral Clock Divisor - Pb_Clk is Sys_Clk/1
#pragma config FPBDIV = DIV_1
//Clock Switching and Monitor Selection - Clock Switch Enable, FSCM Enabled
#pragma config FCKSM = CSECME
//Watchdog Timer Postscaler - 1:1
#pragma config WDTPS = PS32768
//Watchdog Timer Enable - WDT Disabled (SWDTEN Bit Controls)
#pragma config FWDTEN = OFF
//Background Debugger Enable - Debugger is enabled
#pragma config DEBUG = ON
//ICE/ICD Comm Channel Select - ICE EMUC1/EMUD1 pins shared with PGC1/PGD1
#pragma config ICESEL = ICS_PGx1
//Program Flash Write Protect - Disable
#pragma config PWP = OFF
//Boot Flash Write Protect bit - Protection Disabled
#pragma config BWP = OFF
//Code Protect - Protection Disabled
#pragma config CP = OFF






#if defined(TRANSPORT_LAYER_ETH)
	#pragma config FMIIEN = OFF, FETHIO = OFF	// external PHY in RMII/alternate configuration
#endif

//#define SWITCH_PRESSED 0

/*
 * this define controls the inclusion of the comparators in Check_Trigger
 * Comment out to remove the comparators
 */
#define BUILD_WITH_CMP2



/******************************************************************************
Macros used in this file
*******************************************************************************/
#define SWITCH_PRESSED 0u
#define AUX_FLASH_BASE_ADRS				(0x7FC000)
#define AUX_FLASH_END_ADRS				(0x7FFFFF)
#define DEV_CONFIG_REG_BASE_ADDRESS 	(0xF80000)
#define DEV_CONFIG_REG_END_ADDRESS   	(0xF80012)
#define SWUPDATEFLAG                    (0xABCD1234)
#define WDOGCAUSE                       (0x4321DCBA)
/******************************************************************************
Global Variables
*******************************************************************************/
FSFILE * myFile;
BYTE myData[512];
size_t numBytes;
UINT pointer = 0;
UINT readBytes;

UINT8 asciiBuffer[1024];
UINT8 asciiRec[200];
UINT8 hexRec[100];

T_REC record;

/******************************************************************************
 * Global PERSISTENT (and/or SHARED) variables between bootloader and main App
 *****************************************************************************/
BYTE WdogEvent __attribute__((persistent, address(0xA0000000) ));
BYTE WdogEventChk __attribute__((persistent, address(0xA0000004) ));

int SWUpdateFlag __attribute__((persistent, address(0xA0000010) ));
int SWUpdateFlagChk __attribute__((persistent, address(0xA0000014) ));

/* persistent variables for exceptions - define in main app as well */
//DWORD excep_cause __attribute__((persistent, address(0xA0000020) ));
//DWORD excep_addr __attribute__((persistent, address(0xA0000024) ));

/****************************************************************************
Function prototypes
*****************************************************************************/
void ConvertAsciiToHex(UINT8* asciiRec, UINT8* hexRec);
void InitializeBoard(void);
BOOL CheckTrigger(void);
void JumpToApp(void);
BOOL ValidAppPresent(void);
void InitApp(void);
void clock_switch_80M( void);
#ifdef BUILD_WITH_CMP2
void open_cmp2( void );
void open_cmpref( void );
void CMPSettle (void);
void my_delay(void);
#endif

/********************************************************************
* Function: 	main()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview: 	Main entry function. If there is a trigger or 
*				if there is no valid application, the device 
*				stays in firmware upgrade mode.
*
*			
* Note:		 	None.
*              Added 500ms delay after SD card turn on
*              Changed LED's so that:
*                  LED1 indicates file system failure
*                  LED3 indicates file open failure for image.txt
********************************************************************/
int main(void)
{
    volatile UINT i;
    volatile BYTE led = 0;
        
    // Setup configuration
//    (void)SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
//    SYSTEMConfig(SYS_FREQ, SYS_CFG_ALL)        
    
	InitLED();
        
        InitApp();
        
   //SD_PWR_ON;  //switch power on to the card so switches can be tested
        LED1_OFF; // flashes to indicate attention/notification purposes
        LED2_OFF; // indicates FS Init failure when LED1 is flashing
        LED3_OFF; // Indicates that re-programming will be attempted
        LED4_OFF; // indicates image.hex missing when LED1 flashing


//    if(!CheckTrigger() && ValidAppPresent())
    if(!CheckTrigger())
	{
        // This means the switch is not pressed. Jump
        // directly to the application
        JumpToApp();
    }

        /*
         * At this point a SW update is going to occur, so switch upto 80MHz
         */
        clock_switch_80M();

        //Error(); // tuen on LEDs 2 & 4;


#if   (((__PIC32_FEATURE_SET__ >= 100) && (__PIC32_FEATURE_SET__ <= 299)))
		#error("TODO: For PIC32MX1xx/PIC32MX2xx devices, user must map the SPI ports to required I/Os using PPS");
		/* Example Code
		PPSInput(3,SDI2,RPn); // SDI2 mapping, where RPn = RPA2, RPB6....
    	
    	PPSOutput(2,RPn,SDO2);// SDO2 on RPA8
    
    	//Do not forget to switch-off corrresponding "analog selection".
    	ANSELx = 0;
    	*/
#endif

    //Initialize the media
    while (!MDD_MediaDetect())
    {
	    // Waiting for media to be inserted.
	    BlinkLED();

    }

        my_delay(); // just wait for some time before doing FSInit
    // Initialize the File System
   	if(!FSInit())
   	{
	   	//Indicate error and stay in while loop.
         //Error();
         LED2_ON; // LED1 INDICATES FSInit failure
         while(1)
         {
             BlinkLED();
         }
        }    


	myFile = FSfopen("image.hex","r");
			
    if(myFile == NULL)// Make sure the file is present.
    {
	    //Indicate error and stay in while loop.
         //Error();
         LED4_ON; // LED3 indicates file open failure
         while(1)
         {
            BlinkLED();
         }
    }     

    // for backward compatability when programming
    Error(); // RED LEDs on

    // Erase Flash (Block Erase the program Flash)
    EraseFlash();
    // Initialize the state-machine to read the records.
    record.status = REC_NOT_FOUND;
	 
     while(1)
     {
	     
         // For a faster read, read 512 bytes at a time and buffer it.
         readBytes = FSfread((void *)&asciiBuffer[pointer],1,512,myFile);
         
         if(readBytes == 0)
         {
             // Nothing to read. Come out of this loop
             // break;
             FSfclose(myFile);
             // Something fishy. The hex file has ended abruptly, looks like there was no "end of hex record".
             //Indicate error and stay in while loop.
             Error();
             while(1);             
         }

         for(i = 0; i < (readBytes + pointer); i ++)
         {
	         
          // This state machine seperates-out the valid hex records from the read 512 bytes.
             switch(record.status)
             {
                 case REC_FLASHED:
                 case REC_NOT_FOUND:
                     if(asciiBuffer[i] == ':')
                     {
                      // We have a record found in the 512 bytes of data in the buffer.
                         record.start = &asciiBuffer[i];
                         record.len = 0;
                         record.status = REC_FOUND_BUT_NOT_FLASHED;
                     }
                     break;
                 case REC_FOUND_BUT_NOT_FLASHED:
                     if((asciiBuffer[i] == 0x0A) || (asciiBuffer[i] == 0xFF))
                     {
                      // We have got a complete record. (0x0A is new line feed and 0xFF is End of file)
                         // Start the hex conversion from element
                         // 1. This will discard the ':' which is
                         // the start of the hex record.
                         ConvertAsciiToHex(&record.start[1],hexRec);
                         WriteHexRecord2Flash(hexRec);
                         record.status = REC_FLASHED;
                     }
                     break;
             }
             // Move to next byte in the buffer.
             record.len ++;
         }

         if(record.status == REC_FOUND_BUT_NOT_FLASHED)
         {
          // We still have a half read record in the buffer. The next half part of the record is read 
          // when we read 512 bytes of data from the next file read. 
             memcpy(asciiBuffer, record.start, record.len);
             pointer = record.len;
             record.status = REC_NOT_FOUND;
         }
         else
         {
             pointer = 0;
         }
         // Blink LED at Faster rate to indicate programming is in progress.
         led += 3;
	     mLED = ((led & 0x80) == 0);
        	 
     }//while(1)
  
  
    return 0;
}


/********************************************************************
* Function: 	CheckTrigger()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		TRUE: If triggered
				FALSE: No trigger
*
* Side Effects:	None.
*
* Overview: 	Checks if there is a trigger to enter 
				firmware upgrade mode.
 Checks that SD card is present and it is locked
 ie SD_CD_IO = 0 and SD_WP_IO = 1
*
*			
* Note:		 	None.
********************************************************************/
BOOL CheckTrigger(void)
{
	UINT SwitchStatus;
//	SwitchStatus = ReadSwitchStatus();
	SwitchStatus = ((!SD_CD_IO) & SD_WP_IO);
#ifdef BUILD_WITH_CMP2
        /* check main power is applied*/
        open_cmpref();   // set up the comparator reference
        open_cmp2();     // configure the comparator
        CMPSettle();    // wait for the comparator to settle
        if (CMP2Read() != 0 )
        {
            /* main power is NOT applied, so don't attempt to bootload!*/
           CVREFClose();// turn off the reference and comparator
           CMP2Close();
           return FALSE;
        }
#endif
        SD_PWR_ON;  //switch power on to the card so switches can be tested
        /* check for the SW Update flags */

        if ( ((SWUpdateFlag ^ SWUpdateFlagChk) == SWUPDATEFLAG) &&
              ( RCONbits.SWR != 0  )  ) {
            LED3_ON;
            // bootloader has detected that a software update has been 
            // requested by the application
            SWUpdateFlag = SWUpdateFlagChk; //invalidate the flag            
            //while(1) {BlinkLED();}
            return TRUE;
        }
        /* Check to see is the switch is pressed */
        SwitchStatus = ((!SD_CD_IO) & SD_WP_IO);
        if(SwitchStatus == 1u)
        //if( (!SD_CD_IO) & SD_WP_IO )
	{
            LED3_ON;
            // Switch is pressed (card with lock)
            return TRUE;
        }	

	
        // Switch is not pressed. (no card or not locked card)
        // or SWUPDATEFLAG not correct, so don't FLASH the SD Card
        return FALSE;
		
}	


/********************************************************************
* Function: 	JumpToApp()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Jumps to application.
*
*			
* Note:		 	None.
********************************************************************/
void JumpToApp(void)
{	
	void (*fptr)(void);
	fptr = (void (*)(void))USER_APP_RESET_ADDRESS;
	fptr();
}	



/********************************************************************
* Function: 	ConvertAsciiToHex()
*
* Precondition: 
*
* Input: 		Ascii buffer and hex buffer.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Converts ASCII to Hex.
*
*			
* Note:		 	None.
********************************************************************/
void ConvertAsciiToHex(UINT8* asciiRec, UINT8* hexRec)
{
	UINT8 i = 0;
	UINT8 k = 0;
	UINT8 hex;
	
	
	while((asciiRec[i] >= 0x30) && (asciiRec[i] <= 0x66))
	{
		// Check if the ascci values are in alpha numeric range.
		
		if(asciiRec[i] < 0x3A)
		{
			// Numerical reperesentation in ASCII found.
			hex = asciiRec[i] & 0x0F;
		}
		else
		{
			// Alphabetical value.
			hex = 0x09 + (asciiRec[i] & 0x0F);						
		}
	
		// Following logic converts 2 bytes of ASCII to 1 byte of hex.
		k = i%2;
		
		if(k)
		{
			hexRec[i/2] |= hex;
			
		}
		else
		{
			hexRec[i/2] = (hex << 4) & 0xF0;
		}	
		i++;		
	}		
	
}
// Do not change this
#define FLASH_PAGE_SIZE 0x1000
/********************************************************************
* Function: 	EraseFlash()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Erases Flash (Block Erase).
*
*			
* Note:		 	None.
********************************************************************/
void EraseFlash(void)
{
	void * pFlash;
    UINT result;
    INT i;

    pFlash = (void*)APP_FLASH_BASE_ADDRESS;									
    for( i = 0; i < ((APP_FLASH_END_ADDRESS - APP_FLASH_BASE_ADDRESS + 1)/FLASH_PAGE_SIZE); i++ )
    {
	     result = NVMemErasePage( pFlash + (i*FLASH_PAGE_SIZE) );
        // Assert on NV error. This must be caught during debug phase.

        if(result != 0)
        {
           // We have a problem. This must be caught during the debug phase.
            while(1);
        } 
        // Blink LED to indicate erase is in progress.
        mLED = mLED ^ 1;
    }			           	     
}



/********************************************************************
* Function: 	WriteHexRecord2Flash()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Writes Hex Records to Flash.
*
*			
* Note:		 	None.
********************************************************************/
void WriteHexRecord2Flash(UINT8* HexRecord)
{
	static T_HEX_RECORD HexRecordSt;
	UINT8 Checksum = 0;
	UINT8 i;
	UINT WrData;
	UINT RdData;
	void* ProgAddress;
	UINT result;
		
	HexRecordSt.RecDataLen = HexRecord[0];
	HexRecordSt.RecType = HexRecord[3];	
	HexRecordSt.Data = &HexRecord[4];	
	
	// Hex Record checksum check.
	for(i = 0; i < HexRecordSt.RecDataLen + 5; i++)
	{
		Checksum += HexRecord[i];
	}	
	
    if(Checksum != 0)
    {
	    //Error. Hex record Checksum mismatch.
	    //Indicate Error by switching ON all LEDs.
	    Error();
	    // Do not proceed further.
	    while(1);
	} 
	else
	{
		// Hex record checksum OK.
		switch(HexRecordSt.RecType)
		{
			case DATA_RECORD:  //Record Type 00, data record.
				HexRecordSt.Address.byte.MB = 0;
					HexRecordSt.Address.byte.UB = 0;
					HexRecordSt.Address.byte.HB = HexRecord[1];
					HexRecordSt.Address.byte.LB = HexRecord[2];
					
					// Derive the address.
					HexRecordSt.Address.Val = HexRecordSt.Address.Val + HexRecordSt.ExtLinAddress.Val + HexRecordSt.ExtSegAddress.Val;
							
					while(HexRecordSt.RecDataLen) // Loop till all bytes are done.
					{
											
						// Convert the Physical address to Virtual address. 
						ProgAddress = (void *)PA_TO_KVA0(HexRecordSt.Address.Val);
						
						// Make sure we are not writing boot area and device configuration bits.
						if(((ProgAddress >= (void *)APP_FLASH_BASE_ADDRESS) && (ProgAddress <= (void *)APP_FLASH_END_ADDRESS))
						   && ((ProgAddress < (void*)DEV_CONFIG_REG_BASE_ADDRESS) || (ProgAddress > (void*)DEV_CONFIG_REG_END_ADDRESS)))
						{
							if(HexRecordSt.RecDataLen < 4)
							{
								
								// Sometimes record data length will not be in multiples of 4. Appending 0xFF will make sure that..
								// we don't write junk data in such cases.
								WrData = 0xFFFFFFFF;
								memcpy(&WrData, HexRecordSt.Data, HexRecordSt.RecDataLen);	
							}
							else
							{	
								memcpy(&WrData, HexRecordSt.Data, 4);
							}		
							// Write the data into flash.	
							result = NVMemWriteWord(ProgAddress, WrData);	
							// Assert on error. This must be caught during debug phase.		
							if(result != 0)
							{
    							while(1);
    						}									
						}	
						
						// Increment the address.
						HexRecordSt.Address.Val += 4;
						// Increment the data pointer.
						HexRecordSt.Data += 4;
						// Decrement data len.
						if(HexRecordSt.RecDataLen > 3)
						{
							HexRecordSt.RecDataLen -= 4;
						}	
						else
						{
							HexRecordSt.RecDataLen = 0;
						}	
					}
					break;
			
			case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
			    HexRecordSt.ExtSegAddress.byte.MB = 0;
				HexRecordSt.ExtSegAddress.byte.UB = HexRecordSt.Data[0];
				HexRecordSt.ExtSegAddress.byte.HB = HexRecordSt.Data[1];
				HexRecordSt.ExtSegAddress.byte.LB = 0;
				// Reset linear address.
				HexRecordSt.ExtLinAddress.Val = 0;
				break;
				
			case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address. 
				HexRecordSt.ExtLinAddress.byte.MB = HexRecordSt.Data[0];
				HexRecordSt.ExtLinAddress.byte.UB = HexRecordSt.Data[1];
				HexRecordSt.ExtLinAddress.byte.HB = 0;
				HexRecordSt.ExtLinAddress.byte.LB = 0;
				// Reset segment address.
				HexRecordSt.ExtSegAddress.Val = 0;
				break;
				
			case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
				HexRecordSt.ExtSegAddress.Val = 0;
				HexRecordSt.ExtLinAddress.Val = 0;
				// Disable any interrupts here before jumping to the application.
				JumpToApp();
				break;
				
			default: 
				HexRecordSt.ExtSegAddress.Val = 0;
				HexRecordSt.ExtLinAddress.Val = 0;
				break;
		}		
	}	
		
}	

/********************************************************************
* Function: 	ValidAppPresent()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		TRUE: If application is valid.
*
* Side Effects:	None.
*
* Overview: 	Logic: Check application vector has 
				some value other than "0xFFFFFF"
*
*			
* Note:		 	None.
********************************************************************/
BOOL ValidAppPresent(void)
{
	volatile UINT32 *AppPtr;
	
	AppPtr = (UINT32*)USER_APP_RESET_ADDRESS;

	if(*AppPtr == 0xFFFFFFFF)
	{
		return FALSE;
	}
	else
	{
        LED1_ON;
		return TRUE;
	}
}			
/******************************************************************************/
//  Function:
//    static void InitApp(void)
//
//  Description:
//  This routine initializes the hardware.
//  Precondition:
//    None
//
//  Parameters:
//    None - None
//
//  Returns:
//    None
//
//  Remarks:
//    modified form spacelogger WiFi version
//==============================================================================
/* TODO Initialize User Ports/Peripherals/Project here */
//Port usage
//
//Port A
//not used in 64 pin package
//
//Port B
//PORTB is a bidirectional I/O port.
//  RB0     16  I/O ST  PGED1   Data I/O pin for programming/debugging
//  RB1     15  I/O ST  PGEC1   Clock input pin for programming/debugging
//  RB2     14  I/O ST  C2IN    Comparator 2 Negative Input.
//  RB3     13  I/O ST  ?
//  RB4     12  I/O ST  WIFI_PWR    Low to shut down WiFi module
//  RB5     11  I/O ST  SD_CARD_CS  Low to select
//  RB6     17  I/O ST  WIFI_RESET  Low to reset WiFi module
//  RB7     18  I/O ST  WIFI_WP
//  RB8     21  I/O ST  U2CTS       RS232 CTS
//  RB9     22  I/O ST  C2OUT       Comparator 2 Output PWR FAIL connected to INT1 (pin 42)
//  RB10    23  I/O ST  WIFI_HIBERNATE
//  RB11    24  I/O ST  WIFI_CS     Low to select
//  RB12    27  I/O ST  not connected
//  RB13    28  I/O ST  RS232_EN    Low to enable
//  RB14    29  I/O ST  U2RTS       RS232 RTS
//  RB15    30  I/O ST  RS232_SHDN  Low to shut down
//
//PORT C
//PORTC is a bidirectional I/O port.
//  RC12    39  I/O ST  not connected
//  RC13    47  I/O ST  SOSCI
//  RC14    48  I/O ST  SOSCO
//  RC15    40  I/O ST  not connected
//
//PORT D
//PORTD is a bidirectional I/O port.
//  RD0     46  I/O ST  Connected to 0v
//  RD1     49  I/O ST  not connected
//  RD2     50  I/O ST  not connected
//  RD3     51  I/O ST  not connected
//  RD4     52  I/O ST  not connected
//  RD5     53  I/O ST  not connected
//  RD6     54  I/O ST  not connected
//  RD7     55  I/O ST  not connected
//  RD8     42  I/O ST  INT1    POWR_FAIL connected to C2OUT, pin 22
//  RD9     43  I/O ST  INT2    WIFI_INTERRUPT active low
//  RD10    44  I/O ST  BUZZER_1
//  RD11    45  I/O ST  BUZZER_2
//
//PORT E
//PORTE is a bidirectional I/O port.
//  RE0     60  I/O ST  LED1    High to turn led on
//  RE1     61  I/O ST  LED2    High to turn led on
//  RE2     62  I/O ST  LED3    High to turn led on
//  RE3     63  I/O ST  LED4    High to turn led on
//  RE4     64  I/O ST  SD_CARD_WP input
//  RE5     1   I/O ST  SW1     push switch input
//  RE6     2   I/O ST  not connected
//  RE7     3   I/O ST  SD_CARD_CD input
//
//PORT F
//PORTF is a bidirectional I/O port.
//  RF0     58  I/O ST  not connected
//  RF1     59  I/O ST  not connected
//  RF2     34  I/O ST  SDI1    WIFI_SDO
//  RF3     33  I/O ST  SDO1    WIFI_SDI
//  RF4     31  I/O ST  U2RX    RS232_RX
//  RF5     32  I/O ST  U2TX    RS232_TX
//  RF6     35  I/O ST  SCK1    WIFI_SCK
//
//PORT G
//  RG2     37  I ST input pin not connected
//  RG3     36  I ST input pin not connected
//  RG6     4   I/O ST  SCK2    SD_CARD_CLK
//  RG7     5   I/O ST  SDI2    SD_CARD_DO
//  RG8     6   I/O ST  SDO2    SD_CARD_DI
//  RG9     8   I/O ST  SD_CARD_PWR low to shut down


void InitApp(void)
{
    /* Setup analog functionality and port direction */
    //start with LEDs off
    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
    //LED ports as outputs
    LED1_TRIS = 0;
    LED2_TRIS = 0;
    LED3_TRIS = 0;
    LED4_TRIS = 0;
    //write-protect WiFi module flash
    WF_WP_ON;
    WF_WP_TRIS = 0u;    //set as an output
    WF_CS_IO = 1;       //start with WiFi de-selected
    WF_CS_TRIS = 0;

    //switch off power to SD card
    SD_PWR_OFF;
    SD_PWR_TRIS = 0u;   //define as an output

    //write-potect EEPROM
    EEPROM_WP_IO = 1;
    EEPROM_WP_TRIS = 0u;    //set as an output

    CNPUESET = 0x00098000;		// Turn on weak pull ups on CN15, CN16, CN19 (RD5, RD7, RD13), which is connected to buttons on PIC32 Starter Kit boards
    //switch off power to WiFi module
    WF_PWR_OFF;
    WF_PWR_TRIS = 0u;   //make port pin an output

    //set up RS232 ports
    //start RS232 chip in low power mode
    RS232_SHDN_IO = 0;
    RS232_EN_IO = 1;
    //enable outputs
    RS232_SHDN_TRIS = 0;    //make port pin an output
    RS232_EN_TRIS = 0;      //make port pin an output
    RS232_TX_TRIS = 0;      //make tx pin an output
    RS232_RX_TRIS = 1;      //make rx pin an input

    //set the buzzer outputs low
    BUZZER1_IO = 0u;
    BUZZER2_IO = 0u;
    BUZZER1_TRIS = 0u;
    BUZZER2_TRIS = 0u;

    //Configure Multivector Interrupt Mode.  Using Single Vector Mode
    //is expensive from a timing perspective, so most applications
    //should probably not use a Single Vector Mode*/
//    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    // Enable multi-vectored interrupts
//    INTEnableSystemMultiVectoredInt();
#if (0)
        /*
         * don't change the clock now, wait until after check_trigger
         * or after jumping to the main application
         */
    // Enable optimal performance
    if ( ( RCONbits.SWR == 0  ) || //SWRST flag not set, so not entering sleep
         (SWUpdateFlag ^ SWUpdateFlag == SWUPDATEFLAG) ) {  // SW Update requested
        clock_switch_80M(); // step up to 80MHz
        SYSTEMConfigPerformance(GetSystemClock());
        mOSCSetPBDIV(OSC_PB_DIV_1);				// Use 1:1 CPU Core:Peripheral clocks
    }


		// Disable JTAG port so we get our I/O pins back, but first
		// wait 50ms so if you want to reprogram the part with
		// JTAG, you'll still have a tiny window before JTAG goes away.
		// The PIC32 Starter Kit debuggers use JTAG and therefore must not
		// disable JTAG.
//		DelayMs(50);
		#if !defined(__MPLAB_DEBUGGER_PIC32MXSK) && !defined(__MPLAB_DEBUGGER_FS2)
			DDPCONbits.JTAGEN = 0;
		#endif
//rpr		LED_PUT(0x00);				// Turn the LEDs off

		CNPUESET = 0x00098000;		// Turn on weak pull ups on CN15, CN16, CN19 (RD5, RD7, RD13), which is connected to buttons on PIC32 Starter Kit boards
#endif
}
#if 1
void clock_switch_80M( void){
             //DMACONbits.ON = 0;

            // when wake from sleep uses RTC or WDOG, disable cmp2 in sleep as well
            //unlock the OSCCON reg

    int new_osc;
    asm volatile ("di"); // disable ALL IRQs f
    SYSKEY = 0x12345678; //ensure OSCCON is locked
    SYSKEY = 0xAA996655; // Write Key1
    SYSKEY = 0x556699AA; // Write Key2
    OSCCON = 0x10; // set power save mode to sleep

    new_osc = 1; //default value, 80MHz

    OSCCONCLR = (7 << 8); // clear the clock select bits
    OSCCONSET = new_osc << 8; //
    OSCCONSET = (5 << 16); // set PLLMULT to 20
    OSCCONCLR = (3 << 19 ); // clear PBDIV to 0, PBCLK = SYSCLK
    OSCCONSET = (1 << 1); // set SOSCEN for good measure
    OSCCONSET = (1 << 4); // set SLPEN for good measure
    OSCCONSET = 1; // initiate the switch by setting OSWEN

    SYSKEY = 0x12345678; // relock OSCCON
    while (OSCCONbits.OSWEN); //{LED1_ON;} // wait until the switch is confirmed
    LED1_OFF;

     asm volatile ("ei");


        SYSTEMConfigPerformance(GetSystemClock());
        mOSCSetPBDIV(OSC_PB_DIV_1);				// Use 1:1 CPU Core:Peripheral clocks

		// Disable JTAG port so we get our I/O pins back, but first
		// wait 50ms so if you want to reprogram the part with
		// JTAG, you'll still have a tiny window before JTAG goes away.
		// The PIC32 Starter Kit debuggers use JTAG and therefore must not
		// disable JTAG.
//		DelayMs(50);
		#if !defined(__MPLAB_DEBUGGER_PIC32MXSK) && !defined(__MPLAB_DEBUGGER_FS2)
			DDPCONbits.JTAGEN = 0;
		#endif
//rpr		LED_PUT(0x00);				// Turn the LEDs off

		CNPUESET = 0x00098000;		// Turn on weak pull ups on CN15, CN16, CN19 (RD5, RD7, RD13), which is connected to buttons on PIC32 Starter Kit boards

}
#endif
#ifdef BUILD_WITH_CMP2

//=============================================================================
//    Function name:	open_cmpref
//    Return Value:     void
//    Parameters:       none
//    Description:      This sets up the on chip compararot reference
//=============================================================================
void open_cmpref( void )
{
    // configure the comparator voltage reference
#ifdef CMP_REF_SRC_CVREF
    CVREFOpen
    (
      CVREF_ENABLE |
      CVREF_OUTPUT_DISABLE |
      CVREF_RANGE_LOW |
      CVREF_SOURCE_AVDD |
      CVREF_STEP_4
    );
     //printf("Comparator Reference Configured\n");
#endif
};

//=============================================================================
//    Function name:	open_cmp2
//    Return Value:     void
//    Parameters:       none
//    Description:      This sets up the on chip comparator 2
//                      to determine power fail events
//=============================================================================
void open_cmp2( void )
{
    mPORTBSetPinsDigitalOut ( BIT_9 ); // set RB9 (C2OUT) as an output
    mPORTBClearBits( BIT_9 ); // set RB9 low
#ifdef CMP_REF_SRC_CVREF
    CMP2Open
    (
      CMP_ENABLE |
      CMP_OUTPUT_NONINVERT |
      CMP_OUTPUT_DISABLE |  // output not used for external IRQ
      CMP_RUN_IN_IDLE |
      CMP_EVENT_LOW_TO_HIGH | // CMP_EVENT_LOW_TO_HIGH on power fail
      CMP_POS_INPUT_CVREF |
      CMP2_NEG_INPUT_C2IN_POS
    );
#else
    CMP2Open
    (
      CMP_ENABLE |
      CMP_OUTPUT_INVERT |
      CMP_OUTPUT_DISABLE |  // output not used for external IRQ
      CMP_RUN_IN_IDLE |
      CMP_EVENT_LOW_TO_HIGH | // CMP_EVENT_LOW_TO_HIGH on power fail
      CMP_POS_INPUT_C2IN_POS |
      CMP2_NEG_INPUT_IVREF
    );

#endif

   // printf("Comparator 2 Configured %d\n", CMP2Read() );
};

//=============================================================================
//    Function name:	CMPSettle
//    Return Value:    none
//    Parameters:       none
//    Prerequistis:    running from FRCDIV16
//    Description:     takes ~ 1ms to run, assuming ~5 instructions per loop
//                     and 2us per instruction
//=============================================================================
void CMPSettle (void){
    int i;
    for(i=0; i<100; i++) {LED3_TOGGLE;}
    LED3_OFF;
}
#endif

#if (1)
/*
 * Approx delay caclulation 
 */
void my_delay( void)
{

    UINT state_ref;
    UINT state_now = 0;
    UINT count = 0;

    state_ref = ReadCoreTimer() & 0x0200000;
    while (count < 5)
    {
        state_now = ReadCoreTimer() & 0x0200000;
        if (state_now != state_ref)
        {
            state_ref = state_now;
            count++;
        }
    }

}
#endif
/*********************End of File************************************/
