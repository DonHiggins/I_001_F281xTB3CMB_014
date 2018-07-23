// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     SPI.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef SPIx_H
#define SPIx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

void InitSpi( void );
void spi_disableAllSpiDevices();
Uint16 spi_ReadSpiFlashStatus( void );
void spi_ReadSpiFlashRDID(Uint16* memoryType, Uint16* memoryCapacity);
Uint16 spi_ReleasePowerdownRES( void );
void spi_SetFlashWriteEnable( void );
void spi_BulkEraseFlash( void );
bool spi_ReadFlash(Uint16* address, Uint16 countWordsToRead,Uint16* destBuff);
void spi_ClockFlashToFpgaStart(Uint16* address);
void spi_ClockFlashToFpga(Uint16 countWordsToRead);
bool spi_WriteFlash(Uint16* address, Uint16 countWordsToWrite,Uint16* sourceBuff);


void diag_SelectSpiFlash( void );

#endif /* SPIx_H */
