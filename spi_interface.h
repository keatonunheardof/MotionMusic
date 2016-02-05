// SPI Interface Header File 
//      
//      filename:   spi_interface.h
//      author:     Keaton Scheible
//

#include <stdint.h>

#ifndef __SPI_INTERFACE_H__
#define __SPI_INTERFACE_H__

// Function Prototypes
void SPI_Init(void);
void SPI_Transmit(uint8_t address, uint8_t data_out);
uint8_t SPI_Receive(uint8_t address);
#endif
