/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        io_cfg.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 2.30.01+
 * Company:         fuck Microchip Technology, Inc.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       21/07/04    Original.
 * Dario Greggio        28/02/05    .
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

/** I N C L U D E S *************************************************/

/** T R I S *********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0



/** I N P U T / O U T P U T *****************************************************/
// port A
// display mux

// port B
// pulsanti

// port C



#define BeepBit			2
#define m_BeepBit		LATCCbits.LATC2

#define Puls4Bit			7
#define m_Puls4Bit	PORTBbits.RB7
#define Puls3Bit			6
#define m_Puls3Bit	PORTBbits.RB6
#define Puls2Bit			5
#define m_Puls2Bit	PORTBbits.RB5
#define Puls1Bit			4
#define m_Puls1Bit	PORTBbits.RB4

#define PulsRBit			0
#define m_PulsRBit	PORTBbits.RB0

#define LedVBit		0
#define m_LedVBit		LATAbits.LATA0
#define LedRBit		1
#define m_LedRBit		LATAbits.LATA1

#define CoilBit		5
#define m_CoilBit		LATAbits.LATA5


  
#define m_SPICS2Bit LATAbits.LATA3



#define LedRVal		(1 << LedRBit)
#define LedVVal		(1 << LedVBit)



#endif //IO_CFG_H
