/********************************************************************
 FileName:     mfrc522.c
 Dependencies: See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Compiler:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

	v. anche PICBell24.c 2021

********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  -----------------------------------------
  1.0   Initial release


Dedicato a me! (e in morte di italiani e austriaci e inglesi)

********************************************************************/

#ifndef MFRC_C
#define MFRC_C

/** INCLUDES *******************************************************/

#include <generictypedefs.h>
#include <delays.h>
#include <timers.h>
#include <reset.h>
#include <pwm.h>
#include <spi.h>
#include <portb.h>
#include <stdio.h>
#include "cassaforte.h"
#include "io_cfg.h"


#ifdef USA_RFID			// in .h :)
#define USE_MFRC522
#endif
#ifdef USE_MFRC522
#include "mfrc522.h"
#define MFRC_I2C_SPI			// #defined per SPI
#ifndef MFRC_I2C_SPI
const static BYTE MFRC_subAddress=0x50;		// https://forum.arduino.cc/index.php?topic=442750.0
#endif


BYTE DoSPIXFer(BYTE);
#define DoSPIStart() {   m_SPICS2Bit = 0; }
#define DoSPIStop() {   m_SPICS2Bit = 1; }

// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
const rom BYTE MFRC522_firmware_referenceV0_0[] = {
	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const rom BYTE MFRC522_firmware_referenceV1_0[] = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const rom BYTE MFRC522_firmware_referenceV2_0[] = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
	};
// Clone
// Fudan Semiconductor FM17522 (0x88)
const rom BYTE FM17522_firmware_reference[] = {
	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
	};


Uid uid;								// Used by PICC_ReadCardSerial().


/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
signed char MFRC_WriteRegister(enum MFRC_Register reg, BYTE val) {		// per RFId, da Arduino/github

#ifdef MFRC_I2C_SPI
	DoSPIStart();
	DoSPIXFer(reg);
	DoSPIXFer(val);
	DoSPIStop();
	return 0;
#else
	DoI2CStart();
	if(!DoI2CMO(MFRC_subAddress)) {		// mando address
		DoI2CMO(reg);
		DoI2CMO(val);
		return 1;
		}
	DoI2CStop();
	return 0;
#endif
	}

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
signed char MFRC_WriteRegisterArray(enum MFRC_Register reg,	///< The register to write to. One of the PCD_Register enums.
									BYTE count,			///< The number of bytes to write to the register
									BYTE *values		///< The values to write. Byte array.
								) {

#ifdef MFRC_I2C_SPI
	DoSPIStart();
	DoSPIXFer(reg);
	while(count--)
		DoSPIXFer(*values++);
	DoSPIStop();
#else
	DoI2CStart();
	if(!DoI2CMO(MFRC_subAddress)) {		// mando address
		DoI2CMO(reg);
		while(count--)
			DoI2CMO(*values++);
		}
	DoI2CStop();
#endif
	} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
int MFRC_ReadRegister(enum MFRC_Register reg) {
	int n=-1;

#ifdef MFRC_I2C_SPI
	DoSPIStart();
	DoSPIXFer(reg | 0x80);
	n=DoSPIXFer(0);
	DoSPIStop();
#else
	DoI2CStart();
	if(!DoI2CMO(MFRC_subAddress)) {		// mando address
		DoI2CMO(reg | 0x80);
		DoI2CStart();
		DoI2CMO(MFRC_subAddress | 1);
		n=DoI2CMI(0);
		}
	DoI2CStop();
#endif

	return n;
	}

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
signed char MFRC_ReadRegisterArray(enum MFRC_Register reg,	///< The register to read from. One of the PCD_Register enums.
								BYTE count,			///< The number of bytes to read
								BYTE *values,		///< Byte array to store the values in.
								BYTE rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	BYTE address = 0x80 | reg;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	BYTE index = 0;							// Index in values array.

	if(count == 0)
		return 0;

	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));

	count--;								// One read is performed outside of the loop
#ifdef MFRC_I2C_SPI
	DoSPIStart();
	DoSPIXFer(address);
	if(rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		BYTE mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		BYTE value = DoSPIXFer(address);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
		}
	while(index < count) {
		values[index] = DoSPIXFer(address);	// Read value and tell that we want to read the same address again.
		index++;
		}
	values[index] = DoSPIXFer(0);			// Read the final byte. Send 0 to stop reading.
	DoSPIStop();
#else
	DoI2CStart();
	if(!DoI2CMO(MFRC_subAddress)) {		// mando address
		return -1;
		}
	DoI2CMO(address);					// Tell MFRC522 which address we want to read
	DoI2CStart();
	DoI2CMO(MFRC_subAddress | 1);
	if(rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		BYTE mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		BYTE value = DoI2CMI(1);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
		}
	while(index < count) {
		values[index] = DoI2CMI(1);	// Read value and tell that we want to read the same address again.
		index++;
		}
	values[index] = DoI2CMI(0);			// Read the final byte. Send 0 to stop reading.
	DoI2CStop();
#endif
	return index;
	} // End MFRC_ReadRegister()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void MFRC_Reset(void) {
	BYTE count = 0;

	MFRC_WriteRegister(CommandReg, MFRC_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		__delay_ms(50);
		} while((MFRC_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
	} // End PCD_Reset()
/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void MFRC_AntennaOn(void) {
	BYTE value = MFRC_ReadRegister(TxControlReg);
	if((value & 0x03) != 0x03) {
		MFRC_WriteRegister(TxControlReg, value | 0x03);
		}
	} // End MFRC_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void MFRC_AntennaOff(void) {
	MFRC_ClearRegisterBitMask(TxControlReg, 0x03);
	} // End MFRC_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 * 
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
BYTE MFRC_GetAntennaGain(void) {
	return MFRC_ReadRegister(RFCfgReg) & (0x07 << 4);
	} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void MFRC_SetAntennaGain(BYTE mask) {

	if(MFRC_GetAntennaGain() != mask) {						// only bother if there is a change
		MFRC_ClearRegisterBitMask(RFCfgReg, (0x07 << 4));		// clear needed to allow 000 pattern
		MFRC_SetRegisterBitMask(RFCfgReg, mask & (0x07 << 4));	// only set RxGain[2:0] bits
		}
	} // End PCD_SetAntennaGain()

signed char MFRC_Init(void) {		// per RFId, da Arduino/github

	OpenSPI(SPI_FOSC_64, MODE_10,SMPMID );		// questa si auto-setta RC3-5 come dovuto per SPI
  
	MFRC_Reset();		// non usiamo pin reset...

	MFRC_WriteRegister(TxModeReg, 0x00);
	MFRC_WriteRegister(RxModeReg, 0x00);
	// Reset ModWihReg
	MFRC_WriteRegister(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	MFRC_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	MFRC_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	MFRC_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	MFRC_WriteRegister(TReloadRegL, 0xE8);
	
	MFRC_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	MFRC_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	MFRC_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)

	return MFRC_ReadRegister(TPrescalerReg);			// check ok
	}

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode MFRC_TransceiveData(BYTE *sendData,		///< Pointer to the data to transfer to the FIFO.
													BYTE sendLen,		///< Number of bytes to transfer to the FIFO.
													BYTE *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													BYTE *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													BYTE *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													BYTE rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													BYTE checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	BYTE waitIRq = 0x30;		// RxIRq and IdleIRq
	return MFRC_CommunicateWithPICC(MFRC_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
	} // End MFRC_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode MFRC_CommunicateWithPICC(BYTE command,		///< The command to execute. One of the MFRC_Command enums.
														BYTE waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														BYTE *sendData,		///< Pointer to the data to transfer to the FIFO.
														BYTE sendLen,		///< Number of bytes to transfer to the FIFO.
														BYTE *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
														BYTE *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														BYTE *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														BYTE rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														BYTE checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	WORD i;
	BYTE errorRegValue;
	BYTE _validBits=0;
	BYTE controlBuffer[2];
	int status;

	// Prepare values for BitFramingReg
	BYTE txLastBits = validBits ? *validBits : 0;
	BYTE bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	MFRC_WriteRegister(CommandReg, MFRC_Idle);			// Stop any active command.
	MFRC_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	MFRC_WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	MFRC_WriteRegisterArray(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	MFRC_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	MFRC_WriteRegister(CommandReg, command);				// Execute the command
	if(command == MFRC_Transceive)
		MFRC_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	
	// Wait for the command to complete.
	// In MFRC_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	for(i=2000; i>0; i--) {
		BYTE n = MFRC_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if(n & waitIRq) 					// One of the interrupts that signal success has been set.
			break;
		if(n & 0x01) 						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		__delay_us(20);
		}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if(i == 0)
		return STATUS_TIMEOUT;
	
	// Stop now if any errors except collisions were detected.
	errorRegValue = MFRC_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if(errorRegValue & 0x13) 	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
  
	
	// If the caller wants data back, get it from the MFRC522.
	if(backData && backLen) {
		BYTE n = MFRC_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if(n > *backLen)
			return STATUS_NO_ROOM;

		*backLen = n;											// Number of bytes returned
		MFRC_ReadRegisterArray(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = MFRC_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if(validBits)
			*validBits = _validBits;
		}
	
	// Tell about collisions
	if(errorRegValue & 0x08) 		// CollErr
		return STATUS_COLLISION;
	
	// Perform CRC_A validation if requested.
	if(backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if(*backLen == 1 && _validBits == 4)
			return STATUS_MIFARE_NACK;

		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if(*backLen < 2 || _validBits != 0)
			return STATUS_CRC_WRONG;

		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		status = MFRC_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if(status != STATUS_OK)
			return status;
		if((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
			return STATUS_CRC_WRONG;
		}
	
	return STATUS_OK;
	} // End MFRC_CommunicateWithPICC()

/**
 * Sets the bits given in mask in register reg.
 */
void MFRC_SetRegisterBitMask(enum MFRC_Register reg,	///< The register to update. One of the PCD_Register enums.
										BYTE mask			///< The bits to set.
									) { 
	BYTE tmp;
	tmp = MFRC_ReadRegister(reg);
	MFRC_WriteRegister(reg, tmp | mask);			// set bit mask
	} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void MFRC_ClearRegisterBitMask(enum MFRC_Register reg,	///< The register to update. One of the PCD_Register enums.
										BYTE mask			///< The bits to clear.
									  ) {
	BYTE tmp;
	tmp = MFRC_ReadRegister(reg);
	MFRC_WriteRegister(reg, tmp & (~mask));		// clear bit mask
	} // End PCD_ClearRegisterBitMask()

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode MFRC_CalculateCRC(BYTE *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
												BYTE length,	///< In: The number of bytes to transfer.
												BYTE *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	WORD i;

	MFRC_WriteRegister(CommandReg, MFRC_Idle);		// Stop any active command.
	MFRC_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	MFRC_WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	MFRC_WriteRegisterArray(FIFODataReg, length, data);	// Write data to the FIFO
	MFRC_WriteRegister(CommandReg, MFRC_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for(i=5000; i>0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		BYTE n = MFRC_ReadRegister(DivIrqReg);
		if(n & 0x04) {									// CRCIRq bit set - calculation done
			MFRC_WriteRegister(CommandReg, MFRC_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = MFRC_ReadRegister(CRCResultRegL);
			result[1] = MFRC_ReadRegister(CRCResultRegH);
			return STATUS_OK;
			}
		__delay_us(20);		// 
		}

	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
	} // End MFRC_CalculateCRC()


// prova lettura
int readTAG(void) {
	int successRead;

  MFRC_Init();    // Initialize MFRC522 Hardware

  //If you set Antenna Gain to Max it will increase reading distance
  MFRC_SetAntennaGain(RxGain_max);

  do {
    successRead = getID();  // sets successRead to 1 when we get read from reader otherwise 0
		} while(!successRead);   //the program will not go further while you are not getting a successful read

	}


int getID(void) {

  // Getting ready for Reading PICCs
  if(!PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
    return 0;
  	}
  if(!PICC_ReadCardSerial()) {   //Since a PICC placed get Serial and continue
    return 0;
  	}
  // There are Mifare PICCs which have 4 byte or 7 byte UID care if you use 7 byte PICC
  // I think we should assume every PICC as they have 4 byte UID
  // Until we support 7 byte PICCs
//  Serial.println(F("Scanned PICC's UID:"));
//  for ( uint8_t i = 0; i < 4; i++) {  //
//    readCard[i] = uid.uidByte[i];
//    Serial.print(readCard[i], HEX);
//  }
//  Serial.println("");
  PICC_HaltA(); // Stop reading
  return 1;
	}

void ShowReaderDetails(void) {
  // Get the MFRC522 software version
  BYTE v = MFRC_ReadRegister(VersionReg);
//  Serial.print(F("MFRC522 Software Version: 0x"));
//  Serial.print(v, HEX);
//  if (v == 0x91)
//    Serial.print(F(" = v1.0"));
//  else if (v == 0x92)
//    Serial.print(F(" = v2.0"));
//  else
//    Serial.print(F(" (unknown),probably a chinese clone?"));
//  Serial.println("");
  // When 0x00 or 0xFF is returned, communication probably failed
  if ((v == 0x00) || (v == 0xFF)) {
//    Serial.println(F("WARNING: Communication failure, is the MFRC522 properly connected?"));
//    Serial.println(F("SYSTEM HALTED: Check connections."));
    // Visualize system is halted
//    digitalWrite(greenLed, LED_OFF);  // Make sure green LED is off
//    digitalWrite(blueLed, LED_OFF);   // Make sure blue LED is off
//    digitalWrite(redLed, LED_ON);   // Turn on red LED
    while(TRUE); // do not go further
  	}
	}

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_Select(Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											BYTE validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	BYTE uidComplete;
	BYTE selectDone;
	BYTE useCascadeTag;
	BYTE cascadeLevel = 1;
	enum StatusCode result;
	BYTE count;
	BYTE checkBit;
	BYTE index;
	BYTE uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	signed char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	BYTE buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	BYTE bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	BYTE rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	BYTE txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	BYTE *responseBuffer;
	BYTE responseLength;
	BYTE bytesToCopy;
	BYTE collisionPos;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if(validBits > 80)
		return STATUS_INVALID;
	
	// Prepare MFRC522
	MFRC_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = FALSE;
	while(!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch(cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = FALSE;						// Never used in CL3.
				break;
			default:
				return STATUS_INTERNAL_ERROR;
				break;
			}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if(currentLevelKnownBits < 0)
			currentLevelKnownBits = 0;

		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if(useCascadeTag)
			buffer[index++] = PICC_CMD_CT;

		bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if(bytesToCopy) {
			BYTE maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if(bytesToCopy > maxBytes)
				bytesToCopy = maxBytes;

			for(count=0; count < bytesToCopy; count++)
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if(useCascadeTag)
			currentLevelKnownBits += 8;
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = FALSE;
		while(!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if(currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = MFRC_CalculateCRC(buffer, 7, &buffer[7]);
				if(result != STATUS_OK) {
					return result;
					}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
				}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
				}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			MFRC_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = MFRC_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, FALSE);
			if(result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				BYTE valueOfCollReg = MFRC_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if(valueOfCollReg & 0x20)  // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue

				collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if(collisionPos == 0) 
					collisionPos = 32;
				if(collisionPos <= currentLevelKnownBits)  // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;

				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
				}
			else if(result != STATUS_OK) {
				return result;
				}
			else { // STATUS_OK
				if(currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = TRUE; // No more anticollision 
					// We continue below outside the while.
					}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
					}
				}
			} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for(count=0; count < bytesToCopy; count++)
			uid->uidByte[uidIndex + count] = buffer[index++];
		
		// Check response SAK (Select Acknowledge)
		if(responseLength != 3 || txLastBits != 0)  // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
			
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = MFRC_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if(result != STATUS_OK)
			return result;

		if((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
			return STATUS_CRC_WRONG;
		if(responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
			}
		else {
			uidComplete = TRUE;
			uid->sak = responseBuffer[0];
			}
		} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
	} // End PICC_Select()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_RequestA(BYTE *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								BYTE *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
								) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
	} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_WakeupA(BYTE *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								BYTE *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
								) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
	} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
enum StatusCode PICC_REQA_or_WUPA(BYTE command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
								BYTE *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								BYTE *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
								) {
	BYTE validBits;
	enum StatusCode status;
	
	if(bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
		}
	MFRC_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = MFRC_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, FALSE);
	if(status != STATUS_OK)
		return status;
	if(*bufferSize != 2 || validBits != 0) 		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;

	return STATUS_OK;
	} // End PICC_REQA_or_WUPA()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
enum StatusCode PICC_HaltA(void) {
	enum StatusCode result;
	BYTE buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = MFRC_CalculateCRC(buffer, 2, &buffer[2]);
	if(result != STATUS_OK)
		return result;
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = MFRC_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, FALSE);
	if(result == STATUS_TIMEOUT)
		return STATUS_OK;
	if(result == STATUS_OK)  // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;

	return result;
	} // End PICC_HaltA()

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
BYTE PICC_IsNewCardPresent(void) {
	BYTE bufferATQA[2];
	BYTE bufferSize = sizeof(bufferATQA);
	enum StatusCode result;

	// Reset baud rates
	MFRC_WriteRegister(TxModeReg, 0x00);
	MFRC_WriteRegister(RxModeReg, 0x00);
	// Reset ModWihReg
	MFRC_WriteRegister(ModWidthReg, 0x26);

	result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
	} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
BYTE PICC_ReadCardSerial(void) {
	enum StatusCode result = PICC_Select(&uid,0);
	return (result == STATUS_OK);
	} // End 

BYTE DoSPIXFer(BYTE d) {
  BYTE n;
  
   SSPCON1bits.SSPOV = 0;  // Reset overflow bit
   SSPBUF = d;     // Write character to SPI buffer
#ifndef __DEBUG
   while(!SSPSTATbits.BF)
     ClrWdt();
#endif
   n=SSPBUF;
//  while(SPI1STATbits.SPITBF)   // Wait until transmission is started
//    ;
  //*csport |= cspinmask;
   return n;
	}

#endif			// MFRC522 RFId



