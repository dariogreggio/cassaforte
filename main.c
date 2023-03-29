/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Compiler:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  -----------------------------------------
  1.0   Initial release


Dedicato a me! (e in morte di italiani e austriaci e inglesi)

********************************************************************/

#ifndef MAIN_C
#define MAIN_C

/** INCLUDES *******************************************************/

#include <generictypedefs.h>
#include <delays.h>
#include <timers.h>
#include <reset.h>
#include <pwm.h>
#include <spi.h>
#include <portb.h>
#include <stdio.h>
#include <ctype.h>
#include "cassaforte.h"
#include "io_cfg.h"

/** CONFIGURATION **************************************************/

#pragma config WDT = ON, WDTPS = 32768, MCLRE=ON, STVREN=ON, LVP=OFF
#pragma config OSC=IRCIO7, FCMEN=OFF, IESO=OFF, PWRT=ON, BOREN=OFF, BORV=3
#pragma config LPT1OSC=OFF, PBADEN=OFF, XINST=ON, DEBUG=OFF



#pragma romdata
static rom const char CopyrString[]= {'C','y','b','e','r','d','y','n','e',' ','(','A','D','P','M',')',' ','-',' ','C','a','s','s','a','f','o','r','t','e',' ',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0', ' ','2','9','/','0','3','/','2','3', 0 };


#pragma romdata myidlocs=0x200000
const rom char data0=0x04u;
const rom char data1=0x04u;
const rom char data2=0x04u;
const rom char data3=0x07u;
const rom char data4=0x00u;
const rom char data5=0x00u;
const rom char data6=0x00u;
const rom char data7=0x00u;
#pragma romdata

#pragma udata

volatile BYTE tmr_cnt,second_1,second_100;
volatile BYTE Timer10;

enum MODES {
	MODE_BOOT=0,
	MODE_ERASED,
	MODE_LEARNING,
	MODE_IDLE,
	MODE_ENTERING_CODE,
	MODE_CODE_OK,
	MODE_CODE_OPEN,
	MODE_CODE_ERROR,
	MODE_DEFAULT=MODE_IDLE
	};
enum EDITS {
	EDIT_NOEDIT=0,
	EDIT_KEY1,
	EDIT_KEY2,
	EDIT_KEY3,
	EDIT_KEY4,
	EDIT_MAX=EDIT_KEY4
	};

enum MODES Mode=MODE_DEFAULT;					// 
BYTE dividerBeep;

#define MAX_ERRORS 3
BYTE errorCounter=0;
WORD retryTime=10,retryTimeCnt=0;

BYTE dividerUI;
enum EDITS inEdit=EDIT_NOEDIT;

#define MAX_KEY_TIME 100		// mS
#define TIME_LONGCLICK 4
BYTE keyTimeout;
BYTE keyCnt=0;			// in effetti inutile, v.inEdit
BYTE code[4]={0,0,0,0};
BYTE keys[4]={0,0,0,0};
char key;
BYTE longClickCnt;


/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();


/** VECTOR REMAPPING ***********************************************/
	
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR(void)
	{
  _asm 
	  goto YourHighPriorityISRCode 
	_endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR(void)
	{
  _asm 
		goto YourLowPriorityISRCode
	_endasm
	}
	

#pragma code

volatile BYTE divider1s;

//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode
void YourHighPriorityISRCode() {		//
		
//	if(PIE1bits.TMR1IE && PIR1bits.TMR1IF) {

		//WriteTimer1(TMR1BASE);					// inizializzo TMR0
		//WRITETIMER1(TMR1BASE); //dovrebbe essere la macro! solo su Hitech...
		TMR1H=TMR1BASE >> 8; // è SEMPRE a 16bit - non è chiaro a che serva il RW_8BIT
		TMR1L=TMR1BASE & 0xff; //FINIRE sono 8mSec!!

// 100.001 mSec 26/3/23 con ottimizzazioni @4MHz

//	m_LedVBit ^= 1; //check timer	

		INTCONbits.TMR0IF = 0;


		Timer10++;
		second_100=1;					// flag

		divider1s++;
		if(divider1s==10) {		// per RealTimeClock
			divider1s=0;
			second_1=1;					// flag
			}

		PIR1bits.TMR1IF = 0;
//		}
	
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 



#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode()	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.

//	if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {		//c'è solo questo

//		}
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 




/** DECLARATIONS ***************************************************/
#pragma code

/********************************************************************
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
 *******************************************************************/
void main(void) {   
	BYTE i;

//	STKPTR=0;		// risparmia una posizione di Stack, xché main() è stata CALLed!

  InitializeSystem();

	Mode=MODE_BOOT;


  while(1) {

		// Application-specific tasks.

    if(handle_events())
			updateUI();

    }	//end while
	}	//end main


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
static void InitializeSystem(void) {

	StatusReset();

	ClrWdt();

	// imposto oscillatore a 4MHz (per risparmiare corrente...)
	OSCCONbits.IRCF2=1;
	OSCCONbits.IRCF1=1;
	OSCCONbits.IRCF0=0;
	OSCTUNEbits.PLLEN=0;

	OSCCONbits.SCS0 = 0;
	OSCCONbits.SCS1 = 0;

	OSCCONbits.IDLEN=0;

	__delay_ms(100);		// sembra sia meglio aspettare un pizzico prima di leggere la EEPROM.. (v. forum 2006)

  ADCON1 |= 0x0F;                 // Default all pins to digital


  UserInit();
    
	}	//end InitializeSystem



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
 * Overview:        This routine should take care of all of the code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void) {
	BYTE i;

	ClrWdt();

	EnablePullups();

	TRISA=0b00000000;		// 2 led, elettrocalamita
	TRISB=0b11111111;		// 5 pulsanti
	TRISC=0b00000000;		// ev I2C/SPI; buzzer

	LATA=0b00000000;		// tutto spento
	LATB=0b00000000;
	LATC=0b00000000;


	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF & T1_PS_1_8);		// RTC (100mS opp 32768Hz)
//	T1CON=0b10110101;
//	PIE1bits.TMR1IE=1;

	OpenTimer2(TIMER_INT_OFF & T2_PS_1_4 & T2_POST_1_1);		// per Buzzer

	OpenPWM1(BEEP_STD_FREQ); SetDCPWM1(200 /* MSB di 200 << 6 va in CCPR1L..*/);		// SetOutputPWM1(SINGLE_OUT,PWM_MODE_1); // FINIRE! ma SetOutputPWM1 c'è solo sui moduli per motori ossia 4 PWM
	OpenPWM1ConfigIO();			// fa il TRIS...
	dividerBeep=2;		//tanto per... all'accensione!

	RCONbits.IPEN=1;				// interrupt in Modalita' avanzata (18X)
//	INTCON2bits.TMR0IP = 0;			// Timer-0 low Pty
	IPR1bits.TMR1IP = 1;				// Timer-1 high Pty, ev. 
//	IPR1bits.TMR2IP = 0;				// Timer-2 Low Pty NON USATO

	TMR1H=TMR1BASE / 256;					// inizializzo TMR1
	TMR1L=TMR1BASE & 255;					// 
  
	INTCONbits.GIEL = 1;			// attiva interrupt LO-pri
	INTCONbits.GIEH = 1;			// attiva interrupt HI-pri

#ifdef USA_RFID
  MFRC_Init();
  //If you set Antenna Gain to Max it will increase reading distance
  MFRC_SetAntennaGain(RxGain_max);
#endif


	dividerUI=0;
	errorCounter=0;
	retryTime=10;		// 1 sec al boot
	keyCnt=keyTimeout=longClickCnt=0;

	}		//end UserInit


BYTE handle_events(void) {
	static BYTE oldPulsBit=0xff;
	WORD n;
	BYTE i;

	ClrWdt();

	if(second_100) {

		second_100=0;

		switch(Mode) {
			case MODE_BOOT:
				retryTime=10;
				retryTimeCnt=0;
				EEcopiaARAM(&errorCounter);
rileggi_code:
				for(i=0; i<4; i++)
					EEcopiaARAM(&code[i]);
			
				if(code[0]==0xff || code[1]==0xff || code[2]==0xff || code[3]==0xff) {
					StdBeep();
					Delay_S();
					for(i=0; i<4; i++) {
						code[i]=0;
						EEcopiaAEEPROM(&code[i]);
						m_LedRBit^=1;
						T2CONbits.TMR2ON=1;
						__delay_ms(100);
						T2CONbits.TMR2ON=0;
						__delay_ms(100);
						}
					errorCounter=0;
					EEcopiaAEEPROM(&errorCounter);
					goto rileggi_code;
					}

				Mode=MODE_DEFAULT;
				for(i=0; i<4; i++) {
					if(!isdigit(code[i])) {
						Mode=MODE_ERASED;					// 
						break;
						}
					}
				break;
			case MODE_ERASED:
				if((PORTB & 0xf0) != 0xf0) {
					while((PORTB & 0xf0) != 0xf0) 		// debounce
						ClrWdt();
					inEdit=EDIT_NOEDIT;
					keyCnt=0;
					Mode=MODE_LEARNING;
					}
				keyTimeout=longClickCnt=0;
				break;
			case MODE_LEARNING:
				keyTimeout++;
				if(keyTimeout>=MAX_KEY_TIME)
					Mode=MODE_ERASED;
				
#ifdef USA_RFID			// in .h :)
				if(getID()) {     //700mS 8/4/21
			    FLAGK |= (1 << TRIGRF);   // questo prende cmq il sopravvento sui Tasti, qua...
			        
			    kbKeys[0]=uid.uidByte[0];
			    kbKeys[1]=uid.uidByte[1];
					}
#endif
				break;
			case MODE_IDLE:
// mettere!			retryTime=150 << (errorCounter);		// 15 sec, 60sec, 4min
				if((PORTB & 0xf0) != 0xf0) {
					if(retryTimeCnt<retryTime) {
						retryTimeCnt=0;
						m_LedRBit=1;
						ErrorBeep();
						m_LedRBit=0;
						}
					else {
						while((PORTB & 0xf0) != 0xf0) 		// debounce
							ClrWdt();
						inEdit=EDIT_NOEDIT;
						keyCnt=0;
						Mode=MODE_ENTERING_CODE;
						}
					}
				if(retryTimeCnt<retryTime) 			// anche al primo giro... ok
					retryTimeCnt++;
				keyTimeout=longClickCnt=0;
				break;
			case MODE_ENTERING_CODE:
				keyTimeout++;
				if(keyTimeout>=MAX_KEY_TIME) {
					Mode=MODE_IDLE;
					inEdit=EDIT_NOEDIT;
					keyCnt=0;
					}

#ifdef USA_RFID			// in .h :)
				if(getID()) {     //700mS 8/4/21
			    FLAGK |= (1 << TRIGRF);   // questo prende cmq il sopravvento sui Tasti, qua...
			        
			    kbKeys[0]=uid.uidByte[0];
			    kbKeys[1]=uid.uidByte[1];
					}
#endif
				break;
			case MODE_CODE_OK:
				Mode=MODE_CODE_OPEN;
				break;
			case MODE_CODE_OPEN:
				break;
			case MODE_CODE_ERROR:
				break;
			default:
				break;
			}

		
		switch(Mode) {
			case MODE_LEARNING:
			case MODE_ENTERING_CODE:
				if(m_Puls1Bit) {
					if(!(oldPulsBit & 1)) {
						oldPulsBit |= 1;
						key=longClickCnt>TIME_LONGCLICK ? '4' : '0';

updateKeys:
						SetBeep(2);
						keys[inEdit]=key;
						inEdit++;

						if(inEdit>=EDIT_MAX) {
updateDaTimeout:
							inEdit=EDIT_NOEDIT;

							if(Mode==MODE_LEARNING) {
								code[0]=keys[0];
								EEcopiaAEEPROM(&code[0]);
								code[1]=keys[1];
								EEcopiaAEEPROM(&code[1]);
								code[2]=keys[2];
								EEcopiaAEEPROM(&code[2]);
								code[3]=keys[3];
								EEcopiaAEEPROM(&code[3]);

								Mode=MODE_IDLE;
								}
							else {
								if(code[0]==keys[0] && code[1]==keys[1] && code[2]==keys[2] && code[3]==keys[3]) {
									Mode=MODE_CODE_OK;
									}
								else {
									Mode=MODE_CODE_ERROR;
									}
								}
							}
						keyTimeout=0;
						}
					}
				else {
					if(oldPulsBit & 1)
						longClickCnt=0;
					else
						longClickCnt++;
					oldPulsBit &= ~1;
					}
		
				if(m_Puls2Bit) {
					if(!(oldPulsBit & 2)) {

						key=longClickCnt>TIME_LONGCLICK ? '5' : '1';
			
						oldPulsBit |= 2;
						goto updateKeys;
			
						}
					}
				else {
					if(oldPulsBit & 2)
						longClickCnt=0;
					else
						longClickCnt++;
					oldPulsBit &= ~2;
					}
				if(m_Puls3Bit) {
					if(!(oldPulsBit & 4)) {

						key=longClickCnt>TIME_LONGCLICK ? '6' : '2';
			
						oldPulsBit |= 4;
						goto updateKeys;
			
						}
					}
				else {
					if(oldPulsBit & 4)
						longClickCnt=0;
					else
						longClickCnt++;
					oldPulsBit &= ~4;
					}
	
				if(m_Puls4Bit) {
					if(!(oldPulsBit & 8)) {

						key=longClickCnt>TIME_LONGCLICK ? '7' : '3';
			
						oldPulsBit |= 8;
						goto updateKeys;
			
						}
					}
				else {
					if(oldPulsBit & 8)
						longClickCnt=0;
					else
						longClickCnt++;
					oldPulsBit &= ~8;
					}
	
#ifdef USA_RFID			// in .h :)
				if(getID()) {     //700mS 8/4/21
			    FLAGK |= (1 << TRIGRF);   // questo prende cmq il sopravvento sui Tasti, qua...
			        
			    kbKeys[0]=uid.uidByte[0];
			    kbKeys[1]=uid.uidByte[1];
					}
#endif
				break;
			default:
				break;
			}

		if(!m_PulsRBit) {
			m_LedRBit=m_LedVBit=1;
			EEscrivi_(&code[0],0xff);
			EEscrivi_(&errorCounter,0);
			Mode=MODE_BOOT;
			while(!m_PulsRBit) 
				ClrWdt();
			}
	
		return 1;
		}		// second_100

	return 0;
	}


void updateUI(void) {
	BYTE i;
	static BYTE dividerT=0;

	dividerUI++;

	if(dividerBeep) {
		dividerBeep--;
		if(!dividerBeep) {
			T2CONbits.TMR2ON=0;
//			ClosePWM1();		// se no non vanno più gli altri... uniformare :)
			}
		}

	switch(Mode) {
		case MODE_BOOT:
			break;
		case MODE_ERASED:
			m_LedVBit^=1;
			m_LedRBit^=1;
			break;
		case MODE_LEARNING:
			m_LedVBit=0;
			if(dividerUI>=1) {		//.2
				dividerUI=0;
				m_LedRBit^=1;
				}
			break;
		case MODE_IDLE:
			m_CoilBit=0;
			m_LedVBit=0;
			if(dividerUI>=8) {		//.8
				dividerUI=0;
				m_LedRBit=1;
				__delay_ms(10);
				m_LedRBit=0;
				}
			break;
		case MODE_ENTERING_CODE:
			if((inEdit==EDIT_NOEDIT && dividerUI>=2) || (inEdit>EDIT_NOEDIT && dividerUI>=4)) {		//.2 / .4
				dividerUI=0;
				m_LedVBit^=1;
				}
			break;
		case MODE_CODE_OK:
			dividerUI=0;
			errorCounter=0;
			EEcopiaAEEPROM(&errorCounter);
			retryTime=10;
			retryTimeCnt=0;
			m_LedVBit=1;
			SetBeep(10);
			break;
		case MODE_CODE_OPEN:
			m_CoilBit=1;
			if(dividerUI>=30) {		//3S
				dividerUI=0;
				Mode=MODE_IDLE;
				}
			break;
		case MODE_CODE_ERROR:
			m_LedVBit=0;
			m_LedRBit=1;
			SetBeep(20);
			if(dividerUI>=20) {		//2S
				retryTime=75 << (errorCounter+1);		// 15 sec, 60sec, 4min, 16min
				if(errorCounter<MAX_ERRORS)
					errorCounter++;
				EEcopiaAEEPROM(&errorCounter);
				retryTimeCnt=0;
				dividerUI=0;
				Mode=MODE_IDLE;
				}
			break;
		}

	}



void StdBeep(void) {

	CCPR1L=(BEEP_STD_FREQ/2);
	CCP1CONbits.DC1B=0;
	PR2=BEEP_STD_FREQ;
//	OpenPWM1(BEEP_STD_FREQ); SetDCPWM1(BEEP_STD_FREQ/2 );		// SetOutputPWM1(SINGLE_OUT,PWM_MODE_1); // FINIRE! ma SetOutputPWM1 c'è solo sui moduli per motori ossia 4 PWM
//	OpenPWM1ConfigIO();			// fa il TRIS...

	T2CONbits.TMR2ON=1;
	Delay_S();
	T2CONbits.TMR2ON=0;
	}

void SetBeep(BYTE n) {

	CCPR1L=(BEEP_STD_FREQ/2);
	CCP1CONbits.DC1B=0;
	PR2=BEEP_STD_FREQ;

	T2CONbits.TMR2ON=1;
//	OpenPWM1(BEEP_STD_FREQ); SetDCPWM1(BEEP_STD_FREQ/2 );		// SetOutputPWM1(SINGLE_OUT,PWM_MODE_1); // FINIRE! ma SetOutputPWM1 c'è solo sui moduli per motori ossia 4 PWM
// cmq c'è qualcosa di strano RIPROVARE DOPO MODIFICA! diviso invece di per qua sopra
	//OpenPWM1ConfigIO();			// fa il TRIS...

	dividerBeep=n;

	}

void ErrorBeep(void) {

	CCPR1L=(BEEP_ERR_FREQ/2);
	CCP1CONbits.DC1B=0;
	//CCPR1H=BEEP_ERR_FREQ/2;
	PR2=BEEP_ERR_FREQ;
//	OpenPWM1(BEEP_ERR_FREQ); SetDCPWM1(BEEP_ERR_FREQ/2 );		// SetOutputPWM1(SINGLE_OUT,PWM_MODE_1); // FINIRE! ma SetOutputPWM1 c'è solo sui moduli per motori ossia 4 PWM
//	OpenPWM1ConfigIO();			// fa il TRIS...

	T2CONbits.TMR2ON=1;
	Delay_S();
	T2CONbits.TMR2ON=0;
	}



// -------------------------------------------------------------------------------------
void EEscrivi_(SHORTPTR addr,BYTE n) {		// usare void * ?

	EEADR = (BYTE)addr;
	EEDATA=n;

	EECON1bits.EEPGD=0;		// Point to Data Memory
	EECON1bits.CFGS=0;		// Access EEPROM
	EECON1bits.WREN=1;

	INTCONbits.GIE = 0;			// disattiva interrupt globali... e USB?
	EECON2=0x55;		 // Write 55h
	EECON2=0xAA;		 // Write AAh
	EECON1bits.WR=1;									// abilita write.
	INTCONbits.GIE = 1;			// attiva interrupt globali
	do {
		ClrWdt();
		} while(EECON1bits.WR);							// occupato ? 


	EECON1bits.WREN=0;								// disabilita write.
  }

BYTE EEleggi(SHORTPTR addr) {			// usare void * ?

	EEADR=(BYTE)addr;			// Address to read
	EECON1bits.EEPGD=0;		// Point to Data Memory
	EECON1bits.CFGS=0;		// Access EEPROM
	EECON1bits.RD=1;		// EE Read
	return EEDATA;				// W = EEDATA
	}

void EEscriviWord(SHORTPTR addr,WORD n) {			// usare void * ?

	EEscrivi_(addr++,n & 0xff);
	n >>= 8;
	EEscrivi_(addr,n & 0xff);
	}

WORD EEleggiWord(SHORTPTR addr) {			// usare void * ?
	WORD n;

	n=EEleggi(addr++);
	n <<= 8;
	n|=EEleggi(addr);

	return n;
	}


// -------------------------------------------------------------------------------------
//Delays W*4 microseconds (includes movlw, call, and return) @ 4MHz
void __delay_us(BYTE uSec) {

	// 3/4 di prologo...
	do {
//		Delay1TCY();			// 1
		ClrWdt();						// 1; Clear the WDT
		} while(--uSec);		// 3
// dovrebbero essere 4...
  //return             ; 4

/*Delay		macro	Time				// fare una cosa simile...

  if (Time==1)
		goto	$ + 1		;2
		goto	$ + 1
		nop						;1
  		exitm
  endif

  if (Time==2)
		goto	$ + 1
		goto	$ + 1
		goto	$ + 1
		goto	$ + 1
		goto	$ + 1
  		exitm
  endif

	movlw	Time
	call	Delay_uS

	endm*/

	}




void Delay_S_(BYTE n) {				// circa n*100mSec

	do {
	  __delay_ms(100);
		} while(--n);
	}


void __delay_ms(BYTE n) {				// circa n ms

	do {
		__delay_us(250/4-4);
		__delay_us(250/4-4);
		__delay_us(250/4-4);
		__delay_us(250/4-4);
		} while(--n);
	}


/** EOF main.c *************************************************/


