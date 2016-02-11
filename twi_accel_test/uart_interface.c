// This is a UART Interface that was developed for the ATMega328p
//      
//      filename: uart_interface.c
//      author:   Keaton Scheible
//

#include <avr/io.h>
#include <stdlib.h>
#include "uart_interface.h"

// Initialize UART 
void UART_Init( uint16_t ubrr){

    // Set baud rate 
    UBRR0H = (char)(ubrr>>8);
    UBRR0L = (char)ubrr;

    // Enable receiver and transmitter 
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);

    // Set frame format: 8 data bits, 2 stop bits 
    UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);
}

// UART Transmit
void UART_Transmit( char data ){

    // Wait for empty transmit buffer 
    while (!( UCSR0A & (1<<UDRE0))){}

    // Put data into buffer, sends the data
    UDR0 = data;
}

// UART Receive
char UART_Receive( void ){

    // Wait for data to be received 
    while ( !(UCSR0A & (1<<RXC0)) ){}

    // Get and return received data from buffer 
    return UDR0;
}

// Send string over UART
void UART_Transmit_String( char str[] ){

    int i = 0;
    
    // Transmit until a null character is reached
    while(str[i] != 0x00){
        UART_Transmit(str[i]);    
        i++;
    }
}

// Receive a byte of data over UART from serial monitor 
uint8_t UART_Receive_Byte( void ){

    char byte_str[2];
    uint8_t byte;

    // Get high nibble from user
    byte_str[1] = UART_Receive();
    
    // Print high nibble to serial monitor
    UART_Transmit(byte_str[1]);

    // Get low nibble from user
    byte_str[0] = UART_Receive();
    
    // Print low nibble to serial monitor
    UART_Transmit(byte_str[0]);

    // Print new line
    UART_Transmit('\n');

    // Convert ascii characters to an integer from 0 to 255
    byte = (atoh(byte_str[1])<<4) + atoh(byte_str[0]);
        
    return byte;  
}

// Convert ASCII character to hex number
uint8_t atoh( char ascii_char ){
    
    // If ASCII character is between A and F
    if(ascii_char >= 'A' && ascii_char <= 'F')
        return ascii_char - 'A' + 10;
    
    // If ASCII character is between a and f
    else if(ascii_char >= 'a' && ascii_char <= 'f')
        return ascii_char - 'a' + 10;
    
    // If ASCII character is between 0 and 9
    else if(ascii_char >= '0' && ascii_char <= '9')
        return ascii_char - '0';

    else return 0;
}




