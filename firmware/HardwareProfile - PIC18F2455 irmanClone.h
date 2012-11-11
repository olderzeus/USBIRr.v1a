/********************************************************************
 FileName:     	HardwareProfile - PICDEM FSUSB.h
 Dependencies:  See INCLUDES section
 Processor:     PIC18 USB Microcontrollers
 Hardware:      PICDEM FSUSB
 Compiler:      Microchip C18
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
********************************************************************/

#ifndef HARDWARE_PROFILE_18F2455_IRMANCLONE
#define HARDWARE_PROFILE_18F2455_IRMANCLONE

//HACKADAY


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

    //The PICDEM FS USB Demo Board platform supports the USE_SELF_POWER_SENSE_IO
    //and USE_USB_BUS_SENSE_IO features.  Uncomment the below line(s) if
    //it is desireable to use one or both of the features.
    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif


//HACKADAY
    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISBbits.TRISB0    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTBbits.RB0
    #else
    #define USB_BUS_SENSE       1
    #endif


    //Uncomment the following line to make the output HEX of this  
    //  project work with the MCHPUSB Bootloader    
    //#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER
	
    //Uncomment the following line to make the output HEX of this 
    //  project work with the HID Bootloader
    //#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER		

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    #define DEMO_BOARD PICDEM_FS_USB
    #define PICDEM_FS_USB
    #define CLOCK_FREQ 48000000

//HACKADAY
    /** LED ************************************************************/
    #define mInitAllLEDs()      LATC &= 0xF0; TRISC &= 0xF0;
    
    #define mLED_1              LATCbits.LATC2
    #define mLED_2              mLED_1
    #define mLED_3              mLED_1
    
    #define mGetLED_1()         mLED_1
    #define mGetLED_2()         mLED_2
    #define mGetLED_3()         mLED_3

    #define mLED_1_On()         mLED_1 = 1;
    #define mLED_2_On()         mLED_2 = 1;
    #define mLED_3_On()         mLED_3 = 1;
    
    #define mLED_1_Off()        mLED_1 = 0;
    #define mLED_2_Off()        mLED_2 = 0;
    #define mLED_3_Off()        mLED_3 = 0;
    
    #define mLED_1_Toggle()     mLED_1 = !mLED_1;
    #define mLED_2_Toggle()     mLED_2 = !mLED_2;
    #define mLED_3_Toggle()     mLED_3 = !mLED_3;

	//TMR2 Pre- and post- scalers
	#define PRE_x4 0b01
	#define PRE_x16 0b11
	#define POST_x2 0b1000
	#define POST_x3 0b10000
	#define POST_x4 0b11000
	#define POST_x5 0b100000
	#define POST_x6 0b101000
	#define POST_x7 0b110000
	#define POST_x8 0b111000
	#define POST_x9 0b1000000
	#define POST_x10 0b1001000
	#define POST_x11 0b1010000
	#define POST_x12 0b1011000
	#define POST_x13 0b1100000
	#define POST_x14 0b1101000
	#define POST_x15 0b1110000
	#define POST_x16 0b1111000
	
	/** RS 232 lines ****************************************************/
	#define UART_TRISTx   TRISCbits.TRISC6
	#define UART_TRISRx   TRISCbits.TRISC7
	#define UART_Tx       PORTCbits.RC6
	#define UART_Rx       PORTCbits.RC7
	#define UART_ENABLE 	RCSTAbits.SPEN
	#define mDataRdyUSART() PIR1bits.RCIF
	#define mTxRdyUSART()   TXSTAbits.TRMT

#endif  //HARDWARE_PROFILE_PICDEM_FSUSB_H
