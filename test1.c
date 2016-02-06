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


uint8_t wr_buf[1];
uint8_t rd_buf[1];

//*****************************************************************************
// Main Program
//*****************************************************************************

int main(){

    char str[6];

    // Initialize UART
    UART_Init(MY_UBRR);

    init_twi();	

    sei();

    wr_buf[0] = WHO_AM_I;	

    while(1){

        twi_start_wr(LSM6DS3_WRITE,wr_buf,1);

        twi_start_rd(LSM6DS3_READ,rd_buf,1);

        sprintf(str,"%02X\r\n", rd_buf[0]);

        UART_Transmit_String(str);

    }

    return 0;
}
