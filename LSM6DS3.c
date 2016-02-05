#define F_CPU           8000000UL
#define BAUD            19200UL 
#define MY_UBRR         F_CPU/(16*BAUD)-1

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart_interface.h"
#include "spi_interface.h"
#include "LSM6DS3.h"


//*****************************************************************************
// Initialize LSM6DS3 Module
//*****************************************************************************
// 
// This function initializes the LSM6DS3 module. 

void init_LSM6DS3(){

    // Wait for at least 20 ms for accelerometer startup sequence to finish
    _delay_ms(30);

}


//*****************************************************************************
// Write to LSM6DS3
//*****************************************************************************
// 
// This function is used to write to a register on the LSM6DS3. It first 
// attempts to set the register, and then it reads back the value of the 
// register. If the value that is read back is the same value that was sent, it
// prints a success message over UART. If the value that is read back is not 
// the same as the value that was sent, it prints a fail message, along with 
// the returned value.

void write_LSM6DS3(uint8_t address, uint8_t data){

    char data_str[3];
    uint8_t data_in; 

    // Transmit data
    SPI_Transmit(address, data);    

    // Read data
    data_in = SPI_Receive(address);    

    // Check if transfer was successful
    if(data_in == data) UART_Transmit_String("SUCCESS!\r\n");
    else{ UART_Transmit_String("FAIL!    Return Value: "); 
        sprintf(data_str,"%d",data_in);
        UART_Transmit_String(data_str);
        UART_Transmit_String("\r\n");}

}


//*****************************************************************************
// Read from LSM6DS3
//*****************************************************************************
// 
// This function is used to read from register on the LSM6DS3 and then it 
// returns the value that is in that register. 

uint8_t read_LSM6DS3(uint8_t address){

    uint8_t data_in;

    // Read from LSM6DS3
    data_in = SPI_Receive(address);         

    return data_in;
}


//*****************************************************************************
// Initialize Accelerometer
//*****************************************************************************
//  
// This function initializes the accelerometer. Listed Below are the parameters
// that the accelerometer is initialized to.

void init_accel(){

    // Enable X,Y and Z accelerometer axes
    write_LSM6DS3(CTRL9_XL, 0x38); 

    // Set accelerometer ODR to 1.66KHz and acceleration range to +/- 8G's
    set_bits(CTRL1_XL, ((1<<ODR_XL3) | (1<<FS_XL0) | (1<<FS_XL1)));
    
    // Enable data ready interrupt on INT1
    set_bits(INT1_CTRL, (1<<INT1_DRDY_XL));

}

//*****************************************************************************
// Initialize Gyroscope
//*****************************************************************************
// 
// This function initializes the accelerometer. Listed Below are the parameters
// that the accelerometer is initialized to.

void init_gyro(){

    // Enable X,Y and Z gyroscope axes
    write_LSM6DS3(CTRL10_C, 0x38);   

    // Set gyroscope ODR to 1.66KHz angular rate to to 1000 degrees/second
    set_bits(CTRL2_G, ((1<<ODR_G3)| (1<<FS_G1)));
    
    // Enable gyroscope data ready interrupt on INT1
    set_bits(INT2_CTRL, (1<<INT2_DRDY_G));   

}

//*****************************************************************************
// Check Accelerometer Status
//*****************************************************************************
// 
// This function checks the status register to see if new accelerometer data
// is available. If there is data available, it returns 1, otherwise it returns
// 0.

uint8_t accel_data_avail(){

    uint8_t status;

    // Read status register
    status = SPI_Receive(STATUS_REG);

    // If Accelerometer data is ready
    if(status & (1<<XLDA))
        return 1;
    else
        return 0;
}

//*****************************************************************************
// Check Gyroscope Status
//*****************************************************************************
// 
// This function checks the status register to see if new gyroscope data
// is available. If there is data available, it returns 1, otherwise it returns
// 0.

uint8_t gyro_data_avail(){

    uint8_t status; 

    // Read status register
    status = SPI_Receive(STATUS_REG);

    // If Accelerometer data is ready
    if(status & (1<<GDA))
        return 1;
    else
        return 0;

}


//*****************************************************************************
// Set Register Bits 
//*****************************************************************************
// 
// This function sets specific register bits in the LSM6DS3 while leaving the 
// other bits unchanged.
 
void set_bits(uint8_t address, uint8_t bits_to_set){

    uint8_t curr_bits;
    uint8_t new_byte;

    // Get current bits in register
    curr_bits = read_LSM6DS3(address);

    // Save new byte
    new_byte = curr_bits | bits_to_set;

    // Write new byte to register
    write_LSM6DS3(address, new_byte);

}


