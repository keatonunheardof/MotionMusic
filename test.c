#define F_CPU       8000000UL
#define BAUD        19200UL 

#define MY_UBRR     F_CPU/(16*BAUD)-1

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart_interface.h"
#include "twi_interface.h"
#include "LSM6DS3.h"



//*****************************************************************************
// Main Program
//*****************************************************************************

int main(){

    uint8_t rd_buf[1];

    char str[6];

    // Initialize UART
    UART_Init(MY_UBRR);

    init_twi();	

    sei();
        
    init_LSM6DS3();
    
    rd_buf[0] = check_bit(CTRL9_XL,SOFT_EN);
    sprintf(str,"%02X\r\n",rd_buf[0]);
    UART_Transmit_String(str);

    set_bits(CTRL9_XL, SOFT_EN);
    rd_buf[0] = check_bit(CTRL9_XL,SOFT_EN);
    sprintf(str,"%02X\r\n",rd_buf[0]);
    UART_Transmit_String(str);

    clear_bits(CTRL9_XL, SOFT_EN);
    rd_buf[0] = check_bit(CTRL9_XL,SOFT_EN);
    sprintf(str,"%02X\r\n",rd_buf[0]);
    UART_Transmit_String(str);
    
    return 0;
}
