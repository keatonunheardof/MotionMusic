// accel_functions.c       
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "accel_functions.h"
#include "uart_functions.h"
#include <util/delay.h>

uint8_t accel_wr_buf[2];
uint8_t accel_rd_buf[2];

// Set LM73 mode for reading temperature by loading pointer register.
// This is done outside of the normal interrupt mode of operation 

void init_accel(){

    // Load with temperature pointer address
    accel_wr_buf[0] = LM73_PTR_TEMP;

    // Start the TWI write process 
    twi_start_wr(LM73_ADDRESS, accel_wr_buf, 2);

}

uint16_t read_accel(uint8_t celsius){

    uint16_t accel_temp;

    // Read temperature data from LM73
    twi_start_rd(LM73_ADDRESS, accel_rd_buf, 2);  

    // Wait for read to finish
    _delay_ms(2);   

    // Assemble the two bytes read back into one 16-bit value
    accel_temp = (accel_rd_buf[0]<<8) | accel_rd_buf[1];  

    // Right justify the temperature bits 
    accel_temp = (accel_temp>>5);

    // If user wants the temperature in celsius
    if (celsius == 1){

        // Convert to celsius
        accel_temp = (accel_temp>>2);
    }

    // If user wants the temperature in farenheit
    else{

        // Convert to fareneit
        accel_temp = ((accel_temp*9/5)>>2) +32;
    }

    return accel_temp;
}


uint16_t get_remote_temp(uint8_t celsius){

    // A place to assemble the temperature from the accel
    uint16_t accel_temp;

    // High byte of temperature data
    uint8_t accel_temp_high;

    // Low byte of temperature data
    uint8_t accel_temp_low;

    // Transmit celsius or farenheit
    uart_transmit(celsius);

    // Receive high byte of temp data
    accel_temp_high = uart_receive();

    // Receive low byte of temp data
    accel_temp_low = uart_receive();

    // Concatenate temperature data
    accel_temp = (accel_temp_high << 8) | accel_temp_low;

    return accel_temp;
}

uint16_t get_local_temp(uint8_t celsius){

    uint16_t accel_temp;

    // Read temperature data from LM73
    twi_start_rd(LM73_ADDRESS, accel_rd_buf, 2);  

    // Wait for read to finish
    _delay_ms(2);   

    // Assemble the two bytes read back into one 16-bit value
    accel_temp = (accel_rd_buf[0]<<8) | accel_rd_buf[1];  

    // Right justify the temperature bits 
    accel_temp = (accel_temp>>5);

    // If user wants the temperature in celsius
    if (celsius == 1){

        // Convert to celsius
        accel_temp = (accel_temp>>2);
    }

    // If user wants the temperature in farenheit
    else{

        // Convert to fareneit
        accel_temp = ((accel_temp*9/5)>>2) +32;
    }

    return accel_temp;
}


