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
#include "twi_interface.h"
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
// This function is used to write to a register on the LSM6DS3. The byte that 
// is in value gets written to the register location specified by address. 

void write_LSM6DS3(uint8_t address, uint8_t value){

    uint8_t wr_buf[2];

    // Save address to address buffer
    wr_buf[0] = address;
    wr_buf[1] = value;

    // Write value to register address
    twi_start_wr(LSM6DS3_WRITE, wr_buf, 2);
    
    // Wait until data is sent
    while(twi_busy());

}


//*****************************************************************************
// Read from LSM6DS3
//*****************************************************************************
// 
// This function is used to read from the register specified by address on the 
// LSM6DS3. The result is then saved into rd_buf. 

void read_LSM6DS3(uint8_t address, uint8_t *rd_buf){

    uint8_t addr_buf[1];

    // Save address to address buffer
    addr_buf[0] = address;

    // Write register address of where to read data from
    twi_start_wr(LSM6DS3_WRITE, addr_buf, 1);

    // Recieve data from register address and store in rd_buf
    twi_start_rd(LSM6DS3_READ, rd_buf, 1);

    // Wait until data is recieved
    while(twi_busy());
}


//*****************************************************************************
// Initialize Accelerometer
//*****************************************************************************
//  
// This function initializes the accelerometer. Listed Below are the parameters
// that the accelerometer is initialized to.
void init_accel(){
    
    // Enable X,Y and Z accelerometer axes
    set_bits(CTRL9_XL, Xen_XL | Yen_XL | Zen_XL); 

    // Set accelerometer ODR to 1.66KHz and acceleration range to +/- 8G's
    set_bits(CTRL1_XL, ODR_XL2 | FS_XL0 | FS_XL1);

    // Enable data ready interrupt on INT1
    set_bits(INT1_CTRL, INT1_DRDY_XL);

}

//*****************************************************************************
// Initialize Gyroscope
//*****************************************************************************
// 
// This function initializes the accelerometer. Listed Below are the parameters
// that the accelerometer is initialized to.

void init_gyro(){

    // Enable X,Y and Z gyroscope axes
    set_bits(CTRL10_C, Xen_G | Yen_G | Zen_G); 

    // Set gyroscope ODR to 1.66KHz angular rate to to 1000 degrees/second
    set_bits(CTRL2_G, ODR_G2 | FS_G1);

    // Enable gyroscope data ready interrupt on INT1
    set_bits(INT2_CTRL, INT2_DRDY_G);   

}

//*****************************************************************************
// Check Accelerometer Status
//*****************************************************************************
// 
// This function checks the status register to see if new accelerometer data
// is available. If there is data available, it returns 1, otherwise it returns
// 0.

uint8_t accel_data_avail(){

    static uint8_t status_buf[1];

    // Read status register
    read_LSM6DS3(STATUS_REG, status_buf);

    // If Accelerometer data is ready
    if(status_buf[0] & XLDA)
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

    static uint8_t status_buf[1]; 

    // Read status register
    read_LSM6DS3(STATUS_REG, status_buf);

    // If Gyroscope data is ready
    if(status_buf[0] & GDA)
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

    uint8_t new_byte;
    uint8_t curr_bits[1];

    // Get current bits in register
    read_LSM6DS3(address, curr_bits);

    // Save new byte
    new_byte = curr_bits[0] | bits_to_set;

    // Write new byte to register
    write_LSM6DS3(address, new_byte);
    
}


//*****************************************************************************
// Clear Register Bits 
//*****************************************************************************
// 
// This function clears specific register bits in the LSM6DS3 while leaving the 
// other bits unchanged.

void clear_bits(uint8_t address, uint8_t bits_to_clear){

    uint8_t new_byte;
    uint8_t curr_bits[1];

    // Get current bits in register
    read_LSM6DS3(address, curr_bits);

    // Save new byte
    new_byte = curr_bits[0] &~ bits_to_clear;

    // Write new byte to register
    write_LSM6DS3(address, new_byte);
    
}


//*****************************************************************************
// Check Register Bit 
//*****************************************************************************
// 
// This function checks a specific register bit in the LSM6DS3 and returns a 1
// if the bit is set and a 0 if the bit is clear.

uint8_t check_bit(uint8_t address, uint8_t bit_to_check){

    uint8_t bit_result;
    uint8_t curr_bits[1];

    // Get current bits in register
    read_LSM6DS3(address, curr_bits);

    // Check bit
    bit_result = curr_bits[0] &= bit_to_check;

    if(bit_result == 0)
        return 0;

    return 1;

}
