typedef unsigned char * SHORTPTR;


#define SERNUM      1000
#define VERNUMH     1
#define VERNUML     0

//#define USA_SLEEP 1
//#define USA_RFID 1

enum {
	BEEP_STD_FREQ=100 /*60*/,		// 2.45KHz @16MHz, 4MHz il BUZZ cassaforte va meglio così!
//	BEEP_STD_FREQ=60,		// 4KHz @16MHz, 4MHz
	BEEP_ERR_FREQ=110		// 2.2KHz
	};



// il Timer1 conta ogni 1uSec*prescaler... (@4MHz CPUCLK => 1MHz) su PIC18
#define TMR1BASE ((65536-(12500))+2)		// 10Hz per orologio +correzione


    


void EEscrivi_(SHORTPTR addr,BYTE n);
BYTE EEleggi(SHORTPTR addr);
void EEscriviDword(SHORTPTR addr,DWORD n);
DWORD EEleggiDword(SHORTPTR addr);
void EEscriviWord(SHORTPTR addr,WORD n);
WORD EEleggiWord(SHORTPTR addr);
#define EEcopiaARAM(p) { *p=EEleggi(p); }
#define EEcopiaAEEPROM(p) EEscrivi_(p,*p)



	
void __delay_us(BYTE );
void __delay_ms(BYTE );
void Delay_S_(BYTE );
#define Delay_S() Delay_S_(10)
#define SPI_DELAY() Delay_SPI()


	// 100KHz e' OK per MicroChip 24LC16 (memorie) e TC74 (temp); Sensirion SHT11 va fino a oltre 1MHz; l'SMBus del chipset VIA andava a 16KHz; il clock-generator per PC 9248-95 va al max a 100KHz
#define DELAY_SPI_FAST 5
#define DELAY_SPI_SLOW 50


BYTE handle_events(void);
void updateUI(void);


void SetBeep(BYTE);
void StdBeep(void);
void ErrorBeep(void);



