// This is an SPI Interface that was developed for the ATMega328p
//      
//      filename:   spi_interface.c
//      author:     Keaton Scheible
//
#define F_CPU 8000000L

#include <avr/io.h>
#include <util/delay.h>
#include "spi_interface.h"

// Initialize ATMega328p for SPI Transmition
void SPI_Init(void){

    // Set MOSI, SCK and SS to output, and MISO to input
    DDRB = 0x2C;

    // Set CS High
    PORTB |= (1<<PB2);

    // SPI enabled, master, low polarity, MSB 1st
    SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA); 
   
    // Set SCK to run at I/O Clock/2
    SPSR = (1<<SPI2X); 
}

// Send SPI Data
void SPI_Transmit(uint8_t address, uint8_t data_out){
   
    // Drive CS Low    
    PORTB &= ~(1<<PB2);

    // Load address of where to write data
    SPDR = address;
    
    // Wait until 8 clock cycles are done
    while(!(SPSR & (1<<SPIF))){}
    
    // Load data into SPDR for transmition
    SPDR = data_out;
    
    // Wait until 8 clock cycles are done
    while(!(SPSR & (1<<SPIF))){}
    
    // Drive CS High    
    PORTB |= (1<<PB2);
    
}


// Read SPI Data
uint8_t SPI_Receive(uint8_t address){
    
    // Drive CS Low    
    PORTB &= ~(1<<PB2);
    
    // Load address of where to read data
    SPDR = (0x80 | address);
    
    // Wait until 8 clock cycles are done
    while(!(SPSR & (1<<SPIF))){}
    
    // Load dummy to receive return transmition
    SPDR = 0x00;
    
    // Wait until 8 clock cycles are done
    while(!(SPSR & (1<<SPIF))){}
    
    // Drive CS High    
    PORTB |= (1<<PB2);
    
    // Return incoming data from SPDR
    return(SPDR);  
}





