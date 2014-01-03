#ifndef USART_HEADER
#define USART_HEADER

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>

/* (RXD)  PD0 pin 14
     (TXD)  PD1  pin 15
*/

// The UDRE Flag indicates if the transmit buffer (UDR) is ready to receive new data
#define UARTTransmitBufferEmpty() ( UCSRA & (1<<UDRE))

// This flag bit is set when there are unread data in the receive buffer and cleared after UDR read
#define UARTDataAvailable() (UCSRA & (1<<RXC))

void UART_init(void);

void UART_transmit( unsigned char data );
unsigned char UART_receive( void );

#endif
