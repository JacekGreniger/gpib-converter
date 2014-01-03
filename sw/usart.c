#include "usart.h"

#define FOSC 12000000UL // Clock Speed
#define BAUD 115200UL
#define MYUBRR FOSC/8/BAUD-1 //U2X set to 1

/* USART on */
void UART_init (void) {
  unsigned int ubrr = MYUBRR;
  UBRRH = (unsigned char)(ubrr>>8);
  UBRRL = (unsigned char)ubrr;
  UCSRA = (1<<U2X);
  /* Enable receiver and transmitter */
  UCSRB = (1<<RXEN) | (1<<TXEN);
  /* Set frame format: 8data, 1 stop bit, no parity */
  UCSRC = (1<<URSEL) | (3<<UCSZ0);
}

void UART_transmit( unsigned char data ) {
  /* Wait for empty transmit buffer */
  while ( !UARTTransmitBufferEmpty() );
  /* Put data into buffer, sends the data */
  UDR = data;
}

unsigned char UART_receive( void ) {
  /* Wait for data to be received */
  while ( !UARTDataAvailable() );
  /* Get and return received data from buffer */
  return UDR;
}
