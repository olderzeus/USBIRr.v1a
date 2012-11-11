/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:		PIC18 or PIC24 USB Microcontrollers
 Hardware:		The code is natively intended to be used on the following
 				hardware platforms: PICDEM™ FS USB Demo Board, 
 				PIC18F87J50 FS USB Plug-In Module, or
 				Explorer 16 + PIC24 USB PIM.  The firmware may be
 				modified for use on other USB platforms by editing the
 				HardwareProfile.h file.
 Complier:  	Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:		Microchip Technology, Inc.

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
********************************************************************/

/** CONFIGURATION **************************************************/
        #pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
        #pragma config CPUDIV   = OSC1_PLL2   
        #pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
        #pragma config FOSC     = HSPLL_HS
        #pragma config FCMEN    = OFF
        #pragma config IESO     = OFF
        #pragma config PWRT     = OFF
        #pragma config BOR      = ON
        #pragma config BORV     = 3
        #pragma config VREGEN   = ON      //USB Voltage Regulator
        #pragma config WDT      = OFF
        #pragma config WDTPS    = 32768
        #pragma config MCLRE    = ON
        #pragma config LPT1OSC  = OFF
        #pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
        #pragma config STVREN   = ON
        #pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
        #pragma config XINST    = OFF       // Extended Instruction Set
        #pragma config CP0      = OFF
        #pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
        #pragma config CPB      = OFF
//      #pragma config CPD      = OFF
        #pragma config WRT0     = OFF
        #pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
        #pragma config WRTB     = OFF       // Boot Block Write Protection
        #pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
        #pragma config EBTR0    = OFF
        #pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
        #pragma config EBTRB    = OFF

/** INCLUDES *******************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "./USB/usb_device.h"
#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "HardwareProfile.h"

	#define IDLE 0
	#define HALF_PERIOD 1
	#define BIT_PERIOD_0 2
	#define BIT_PERIOD_1 3
	#define PROCESS 4
	#define SEND 5
	#define SENDING 6


/** V A R I A B L E S ********************************************************/
#pragma udata
char USB_In_Buffer[64];
char USB_Out_Buffer[64];

unsigned char irState=IDLE;//ir input state machine state
unsigned char sample[50];
unsigned char sampleCount,b;
BOOL stringPrinted;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void BlinkUSBStatus(void);
void UserInit(void);
void InterruptHandlerHigh (void);

void InitializeUSART(void);
unsigned char getcUSART (void);
void putcUSART(char c); 


/** DECLARATIONS ***************************************************/
#pragma code

/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{   
    InitializeSystem();

    while(1)
    {
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // frequently (such as once about every 100 microseconds) at any
        				  // time that a SETUP packet might reasonably be expected to
        				  // be sent by the host to your device.  In most cases, the
        				  // USBDeviceTasks() function does not take very long to
        				  // execute (~50 instruction cycles) before it returns.
    				  

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{

    ADCON1 |= 0x0F;                 // Default all pins to digital
   
//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = 1; // See HardwareProfile.h
    #endif
    
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
    UserInit();

}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{

	//disable some default nonsense
    ADCON1 |= 0x0F;                 // Default all pins to digital
	CVRCON=0b00000000;

    //Initialize all of the LED pins
	mInitAllLEDs();

	//setup the timer 
	T2CON=0b00000000;
//	T2CONbits.TMR2ON=1;
//	T2CONbits.T2CKPS1=0;
//	T2CONbits.T2CKPS0=0;
//	T2CONbits.T2OUTPS3=1;
//	T2CONbits.T2OUTPS2=0;
//	T2CONbits.T2OUTPS1=1;
//	T2CONbits.T2OUTPS0=0;
	T2CON=PRE_x4+POST_x6;
	PR2=0xFF;
	IPR1bits.TMR2IP=1;//timer 2 high priority
//	PIR1bits.TMR2IF=0;//clear the interrupt flas
//	PIE1.TMR2IE=1; enable interrupts...

   //setup RB interrupt
   LATB=0;	//ground
   TRISB=0;	//output
   TRISBbits.TRISB4=1; //direction to input
   INTCONbits.RBIF = 0;    //Reset the RB Port Change Interrupt Flag bit   
   INTCONbits.RBIE = 1;  //Enables the RB port change interrupt

   //INTCON=0b11000000; 			//global/peripheral/tmr0ie/_/_/TMR0IF/_/_
   INTCONbits.GIEL = 1;        //enable TMR2 peripheral interrupts
   INTCONbits.GIEH = 1;        //enable interrupts
   
   InitializeUSART();

}//end UserInit

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{   
    BYTE numBytesRead, numBytesWrite=0,i;
	static unsigned char rc5x,addr,cmd,toggle;

    //Blink the LEDs according to the USB device status
    BlinkUSBStatus();

    #define mLED_Both_Off()         {mLED_1_Off();}
    #define mLED_Both_On()          {mLED_1_On();}

	if(irState==PROCESS){
		//0 second start bit
		rc5x=sample[0];
		//1 toggle bit
		toggle=sample[1];
		addr=0;
		for(i=2;i<7;i++){ //56 78 910 1112 1314
			addr<<=1;
			addr+=sample[i];					
		}

		cmd=rc5x;//make rc5x bit 6
		//cmd=0;
		for(i=7;i<13;i++){ //1516 1718 1920 2122 2324 2526
			cmd<<=1;
			cmd+=sample[i];
		}
		irState=SEND;
	}

    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

    if(mUSBUSARTIsTxTrfReady()){
		numBytesWrite=0;
		if(irState==SEND){
			BYTE i;

			USB_Out_Buffer[numBytesWrite]=addr;
			numBytesWrite++;
			USB_Out_Buffer[numBytesWrite]=cmd;
			numBytesWrite++;
			
			for(i=0;i<4;i++){//four buffer bytes for IRMAN/RC5 combo...
				USB_Out_Buffer[numBytesWrite]=0x00;
				numBytesWrite++;			
			}

			irState=SENDING;
		}else if(irState==SENDING){
			irState=IDLE;//wait for more RC commands....
		}

		numBytesRead = getsUSBUSART(USB_In_Buffer,64);
		//a=getcUSART();
		if(numBytesRead != 0){
			BYTE i;        
			for(i=0;i<numBytesRead;i++){			
				switch(USB_In_Buffer[i]){
					case 'I'://reset the system, load defaults from eeprom
						break;
					case 'R'://got R, send "OK" for IRman protocol
						//USB
						USB_Out_Buffer[numBytesWrite]='O';
						numBytesWrite++;
						USB_Out_Buffer[numBytesWrite]='K';
						numBytesWrite++;
						
						//USART
						//while(!mTxRdyUSART());//wait for tx
						//putcUSART('O');
						//while(!mTxRdyUSART());//wait for tx
						//putcUSART('K');
						break;
					case 'C'://rc5 mode	

						//enforce manchester encoding of all 13bits/26periods
						//take exactly 26 samples.
						//terminal rc5 decoding protocol
					//case 0x00-0x29://take 0-x bit samples
					case 'P'://go silent
					//set prescaler, post scaler, PR2?
					case 'S'://save settings to eeprom
						break;
				
				}
			}
		}
		putUSBUSART(USB_Out_Buffer,numBytesWrite);
	}

    CDCTxService();
}		//end ProcessIO


/******************************************************************************
 * Function:        void InterruptVectorHigh(void)
 *
 *
 * Note:            When the PWM timer interrupt fires the PIC checks here for directions
 *					
 *		
 *****************************************************************************/
//----------------------------------------------------------------------------
// High priority interrupt vector
#pragma code InterruptVectorHigh = 0x08
void
InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}

/******************************************************************************
 * Function:        void InterruptHandlerHigh ()
 *
 *
 * Note:            When the timer interrupts this function is executed:
 *					1)increment the master SoftPWM counter (SoftPWM.tick)
 *					2)compare each color's value (SoftPWM.R/G/B) with the counter
 *					3)turn off colors if SoftPWM.R/G/B=SoftPWM.tick
 *					
 *		
 *****************************************************************************/
//----------------------------------------------------------------------------
// High priority interrupt routine
#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh ()
{
	static unsigned char a,pr0=0,pr1=0;

	if(INTCONbits.RBIF == 1){ //if RB Port Change Interrupt	
	  a=PORTBbits.RB4;//must read PORTB to clear RBIF....
	  switch(irState){
		case IDLE:
			if(a==0){//only if a start bit
				T2CON=PRE_x4+POST_x6;
				PR2=221;
				PIR1bits.TMR2IF=0;//clear the interrupt flag
				PIE1bits.TMR2IE=1; //able interrupts...
				T2CONbits.TMR2ON=1;
				irState=HALF_PERIOD;
			}
			break;
		}	
    	INTCONbits.RBIF = 0;    //Reset the RB Port Change Interrupt Flag bit  

  	}else if(PIE1bits.TMR2IE==1 && PIR1bits.TMR2IF==1){
		switch(irState){
			case HALF_PERIOD:
				PIE1bits.TMR2IE=1;
				T2CONbits.TMR2ON=0;
				//full bit values
				T2CON=PRE_x4+POST_x12;
				//PR2=221;

				PIR1bits.TMR2IF=0;//clear the interrupt flag
				PIE1bits.TMR2IE=1; //able interrupts...
				T2CONbits.TMR2ON=1;
				sampleCount=0;
				irState=BIT_PERIOD_0;
				break;
			case BIT_PERIOD_0:
				pr0=PORTBbits.RB4;
				irState=BIT_PERIOD_1;
				break;
			case BIT_PERIOD_1:
				pr1=PORTBbits.RB4;
				if(pr0==1 && pr1==0){
					sample[sampleCount]=1;
				}else if (pr0==0 && pr1==1){
					sample[sampleCount]=0;
				}else{
					//error
					irState=IDLE;
					T2CONbits.TMR2ON=0;
					break;
				}
				sampleCount++;
				irState=BIT_PERIOD_0;
			
				if(sampleCount==13){//done sampling
					irState=PROCESS;
					T2CONbits.TMR2ON=0;
				}
				break;				
		}
		PIR1bits.TMR2IF=0;//clear the interrupt flag
	}  
}


/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
      if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();}
    #define mLED_Both_On()          {mLED_1_On();}
    #define mLED_Only_1_On()        {mLED_1_On();}
    #define mLED_Only_2_On()        {mLED_1_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus



/******************************************************************************
 * Function:        void InitializeUSART(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine initializes the UART to 9600
 *
 * Note:            
 *
 *****************************************************************************/
void InitializeUSART(void)
{
	unsigned char c;

	UART_TRISRx=1;				// RX
	UART_TRISTx=0;				// TX
	TXSTA = 0x24;       	// TX enable BRGH=1
	RCSTA = 0x90;       	// Single Character RX
	SPBRG = 0xe1;			// 0x0271 for 48MHz -> 19200 baud 0x0067->115200 0x04e1->9600
	SPBRGH = 0x04;      	
	BAUDCON = 0x08;     	// BRG16 = 1
	c = RCREG;				// read 
}

/******************************************************************************
 * Function:        void putcUSART(char c)
 *
 * PreCondition:    None
 *
 * Input:           char c - character to print to the UART
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Print the input character to the UART
 *
 * Note:            
 *
 *****************************************************************************/
void putcUSART(char c)  
{
    #if defined(__18CXX)
	    TXREG = c;
    #else
        UART2PutChar(c);
    #endif
}


/******************************************************************************
 * Function:        void putcUSART(char c)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          unsigned char c - character to received on the UART
 *
 * Side Effects:    None
 *
 * Overview:        get character from the UART
 *
 * Note:            
 *
 *****************************************************************************/
unsigned char getcUSART ()
{
	char  c;

	if (RCSTAbits.OERR)  // in case of overrun error
	{                    // we should never see an overrun error, but if we do, 
		RCSTAbits.CREN = 0;  // reset the port
		c = RCREG;
		RCSTAbits.CREN = 1;  // and keep going.
	}
	else
		c = RCREG;
	
	// not necessary.  EUSART auto clears the flag when RCREG is cleared
	//	PIR1bits.RCIF = 0;    // clear Flag
	return c;
}




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        TRISA &= 0xFF3F;
        LATAbits.LATA6 = 1;
        Sleep();
        LATAbits.LATA6 = 0;
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since 
 *                  the USBCBCheckOtherReq() function is the apps
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 *******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif



/** EOF main.c *************************************************/
