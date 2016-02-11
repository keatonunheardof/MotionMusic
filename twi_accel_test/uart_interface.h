// UART Interface Header File
//      
//      filename:   uart_interface.h
//      author:     Keaton Scheible
//

#include <stdint.h>

#ifndef __UART_INTERFACE_H__
#define __UART_INTERFACE_H__ 

// Function Prototypes
void UART_Init( uint16_t ubrr);
void UART_Transmit( char data );
char UART_Receive( void );
void UART_Transmit_String( char str[] );
uint8_t UART_Receive_Byte( void );
uint8_t atoh(char);

#endif
