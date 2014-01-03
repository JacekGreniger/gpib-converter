/* GPIB to USB converter
   ATMega32 + FT232RL
   clocked with external 12MHz taken from FT232RL (output 1)
   version 4, 20.08.2012
   
   Added feature:
    - possible to exit from printer mode
    - added help under command "?"
    - backspace support
    - commands scroll
    - printer mode, red led 
    - listen mode
    - M command for sending data without EOI
*/


#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <stdbool.h>
#include <ctype.h>
#include "usart.h"
#include "avr/pgmspace.h"
#include <avr/interrupt.h>

#define F_CPU 12000000UL  
#include <util/delay.h>

#define DEFAULT_ADDRESS 21

/*
GPIB Connector pinout

Pin | Nazwa | Opis               | Source            | Atmega pin |	
----+-------+--------------------+-------------------+------------+--------
1   | DIO1  | Data bit 1 (LSB)   | Talker            | PA0	37    | X3-4
2   | DIO2  | Data bit 2         | Talker            | PA1	36    | X3-3
3   | DIO3  | Data bit 3         | Talker            | PA2	35    | X3-2
4   | DIO4  | Data bit 4         | Talker            | PA3	34    | X3-1
5   | EOI   | End Or Indentity   | Talker/Controller | PC7	26    | X4-1
6   | DAV   | Data Valid         | Controller        | PC6	25    | X4-2
7   | NRFD  | Not Ready For Data | Listener          | PC5	24    | X4-3
8   | NDAC  | No Data Accepted   | Listener          | PC4	23    | X4-4
9   | IFC   | Interface Clear    | Controller        | PC3	22    | X5-1
10  | SRQ   | Service Request    | Talker            | PC2	21    | X5-2
11  | ATN   | Attention          | Controller        | PC1	20    | X5-3
12  |       | Ekran              |                   |            |
13  | DIO5  | Data bit 5         | Talker            | PA4	33    | X2-4
14  | DIO6  | Data bit 6         | Talker            | PA5	32    | X2-3
15  | DIO7  | Data Bit 7         | Talker            | PA6	31    | X2-2
16  | DIO8  | Data bit 8 (MSB)   | Talker            | PA7	30    | X2-1
17  | REN   | Remote Enabled     | Controller        | PC0	19    | X5-4
18  |       | GND DAV            |                   |            |
19  |       | GND NRFD           |                   |            |
20  |       | GND NDAC           |                   |            |
21  |       | GND IFC	         |                   |            |
22  |       | GND SRQ	         |                   |            |
23  |       | GND ATN	         |                   |	          |
24  |       | GND data           |                   |	          | X3-5
*/

#define EOI (_BV(PC7))  //pin 26 ATmega, pin 5 GPIB
#define DAV (_BV(PC6))  //pin 25 ATmega, pin 6 GPIB
#define NRFD (_BV(PC5)) //pin 24 ATmega, pin 7 GPIB, output
#define NDAC (_BV(PC4)) //pin 23 ATmega, pin 8 GPIB, output
#define IFC (_BV(PC3))  //pin 22 ATmega, pin 9 GPIB
#define SRQ (_BV(PC2))  //pin 21 ATmega, pin 10 GPIB
#define ATN (_BV(PC1))  //pin 20 ATmega, pin 11 GPIB
#define REN (_BV(PC0))  //pin 19 ATmega, pin 17 GPIB

#define SetLed(x) ( PORTD = ((x==0)?(PORTD | _BV(PD2)) : (PORTD & ~_BV(PD2)) ))

#define SetEOI(x) ( PORTC = (x)? (PORTC | EOI) : (PORTC & ~EOI) )
#define SetDAV(x) ( PORTC = (x)? (PORTC | DAV) : (PORTC & ~DAV) )
#define SetNRFD(x) ( PORTC = (x)? (PORTC | NRFD) : (PORTC & ~NRFD) )
#define SetNDAC(x) ( PORTC = (x)? (PORTC | NDAC) : (PORTC & ~NDAC) )
#define SetIFC(x) ( PORTC = (x)? (PORTC | IFC) : (PORTC & ~IFC) )
#define SetSRQ(x) ( PORTC = (x)? (PORTC | SRQ) : (PORTC & ~SRQ) )
#define SetATN(x) ( PORTC = (x)? (PORTC | ATN) : (PORTC & ~ATN) )
#define SetREN(x) ( PORTC = (x)? (PORTC | REN) : (PORTC & ~REN) )

#define ESC_KEY_UP 0x41
#define ESC_KEY_DOWN 0x42
#define ESC_KEY_RIGHT 0x43
#define ESC_KEY_LEFT 0x44

#define MAX_COMMANDS 15
#define BUF_SIZE 64
#define GPIB_BUF_SIZE 128
#define GPIB_MAX_RECEIVE_TIMEOUT 50000
#define GPIB_MAX_TRANSMIT_TIMEOUT 50000
#define EMPTY_LINE 1

#define HELP_LINES 18
#define HELP_STRING_LEN 64
const char helpStrings[HELP_LINES][HELP_STRING_LEN] PROGMEM = {
  "GPIB to USB converter v4\r\n\r\n",
  "Transmit commands, OK/TIMEOUT/ERROR\r\n",
  "  <D> Data (ATN false), <M> Data without EOI\r\n",
  "  <C> Command (ATN true)\r\n",
  "  <T> Hex transmit (0C - command, 0D - data)\r\n",
  "Receive commands (receives until EOI,max 127 bytes)\r\n",
  "  <X> ASCII, <payload> or TIMEOUT\r\n",
  "  <Y> BINARY, <length><payload>\r\n",
  "  <Z> HEX, <length><payload>\r\n",
  "  <P> Continous read (plotter mode)\r\n",
  "General commands\r\n",
  "  <A> Set/get converter talk address\r\n",
  "  <S> Get REQ/SRQ/LISTEN state (1 if true)\r\n",
  "  <R> Set REMOTE mode (REN true)\r\n",
  "  <L> Set LOCAL mode (REN false)\r\n",
  "  <I> Generate IFC pulse\r\n",
  "  <E> Get/set echo on(E1)/off(E0)\r\n",
  "  <H> Commands history\r\n"
};


typedef enum {OFF = 0, SLOW, FAST} ledBlinking_t;
ledBlinking_t ledBlinking = OFF;
unsigned char listenAddress = DEFAULT_ADDRESS;
unsigned char msgEndSeq = 0;
unsigned char remoteState = 0;

/* ======================================================= */

void GPIO_init() {
  // DDR = 1 output
  // DDR = 0 input
  DDRD = 0x04; // PD2 output
  PORTD = 0x04; // leds off,

  DDRB = _BV(PB6); // PB6 output
  PORTB = _BV(PB5) | _BV(PB7); // PB5,PB7 pull-up
}


int uart_putchar(char ch, FILE* file)
{
  UART_transmit(ch);
  return ch;
}


void ReconfigureGPIO_GPIBReceiveMode()
{
  DDRA = 0x00; // PA0-PA7 inputs
  PORTA = 0xff; // pullup on
  
  DDRC = IFC | ATN | REN | NRFD | NDAC; // these lines are outputs, other as inputs
  PORTC = IFC | ATN | (remoteState?0:REN) | EOI | DAV | SRQ; // pullup on
}


void ReconfigureGPIO_GPIBNormalMode()
{
  DDRA = 0xff; // data lines are outputs
  PORTA = 0x00; // output level 0
  
  DDRC = IFC | ATN | REN | EOI | DAV; // these lines are outputs
  PORTC = IFC | ATN | (remoteState?0:REN) | EOI | DAV | SRQ | NRFD | NDAC; // pullup on
}


int GPIB_Receive(unsigned char * buf, unsigned char bufLength, unsigned char * receivedLength)
{
  unsigned char index = 0;
  unsigned char c;
  unsigned int timeout;

  do
  {
    SetNRFD(1); //ready for receiving data
    //-1 & 5
    
    timeout = 0;
    while (PINC & DAV) // waiting for falling edge
    {
      //_delay_ms(1);
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNRFD(0);
        return 0;
      }
    }
    // 0
    
    SetNRFD(0); //not ready for receiving data
    // 1
    
    c = ~PINA; //read data

    buf[index++] = c;

    SetNDAC(1); //data accepted
    //2
    
    while (!(PINC & DAV)) // waiting for rising edge
    {
      //_delay_ms(1);
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNDAC(0);
        return 0;
      }
    }
    //3
    
    SetNDAC(0);
    //4
  } while ((index < bufLength) /*&& (c != 13)*/);
  *receivedLength = index;
  return 255;
}


int GPIB_Receive_till_eoi(unsigned char * buf, unsigned char bufLength, unsigned char * receivedLength)
{
  unsigned char index = 0;
  unsigned char c;
  unsigned char eoi = 0;
  unsigned int timeout;

  do
  {
    SetNRFD(1); //ready for receiving data
    //-1 & 5
    
    timeout = 0;
    while (PINC & DAV) // waiting for falling edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNRFD(0);
        return 0;
      }
    }
    // 0
    
    if ((PINC & EOI) == 0)
      eoi = 1;
    
    SetNRFD(0); //not ready for receiving data
    // 1
    
    c = ~PINA; //read data

    buf[index++] = c;

    SetNDAC(1); //data accepted
    //2
    
    while (!(PINC & DAV)) // waiting for rising edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNDAC(0);
        return 0;
      }
    }
    //3
    
    SetNDAC(0);
    //4
  } while ((index < bufLength) && (eoi == 0));
  *receivedLength = index;
  return 255;
}


int GPIB_Receive_till_lf(unsigned char * buf, unsigned char bufLength, unsigned char * receivedLength)
{
  unsigned char index = 0;
  unsigned char c;
  unsigned int timeout;

  //SetNDAC(0);
  //SetNRFD(0);

  do
  {
    SetNRFD(1); //ready for receiving data
    //-1 & 5
    
    timeout = 0;
    while (PINC & DAV) // waiting for falling edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNRFD(0);
        return 0;
      }
    }
    // 0
       
    SetNRFD(0); //not ready for receiving data
    // 1
    
    c = ~PINA; //read data

    buf[index++] = c;

    SetNDAC(1); //data accepted
    //2
    
    while (!(PINC & DAV)) // waiting for rising edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNDAC(0);
        return 0;
      }
    }
    //3
    
    SetNDAC(0);
    //4
  } while ((index < bufLength) && (c != 10));
  *receivedLength = index;
  return 255;
}


int GPIB_Transmit(unsigned char * buf, unsigned char bufLength, unsigned char eoi)
{
  unsigned char index = 0;
  unsigned int timeout;
  
  if ((0 == bufLength) || ((PINC & NRFD) && (PINC & NDAC)))
    return 0;
  
  do
  {
    if ((index+1 == bufLength) && eoi)
      SetEOI(0); // last byte
    
    //transmit debug    
    //printf("%02x ", buf[index]);
    
    PORTA = ~buf[index];
    index++;
    
    _delay_us(100);
     
    timeout = 0;
    while (!(PINC & NRFD)) // waiting for high on NRFD
    {
      timeout++;
      if (timeout > GPIB_MAX_TRANSMIT_TIMEOUT)
      {
        SetEOI(1);
        return 0;
      }
    }
    
    SetDAV(0);
    _delay_us(100);
   
    while (!(PINC & NDAC)) // waiting for high on NDAC
    {
      timeout++;
      if (timeout > GPIB_MAX_TRANSMIT_TIMEOUT)
      {
        SetEOI(1);
        SetDAV(1);
        return 0;
      }
    }
    
    SetEOI(1);
    SetDAV(1);
    //4
  } while ((index < bufLength));

  //printf("\r\n");
  return 255;
}


void ShowHelp()
{
  char buf[64];
  unsigned char i;

  for (i=0; i<HELP_LINES; i++)
  {
    memcpy_P(buf, helpStrings[i], HELP_STRING_LEN);
    printf("%s", buf);
  }
}


char UART_RcvEscapeSeq()
{
  while (!UARTDataAvailable());
  if (UART_receive() != 0x5B)
    return 0;
  while (!UARTDataAvailable());
  return UART_receive();
}

#define T0_INIT 128 //177 = 255-78  daje 10ms dla 8MHz, 79=255-156 dla 16MHz

SIGNAL (SIG_OVERFLOW0) {
  static unsigned char timCnt = 0;
  static unsigned char led = 0;
  TCNT0 = T0_INIT; 
  if (OFF == ledBlinking)
    return;
	
  timCnt++;
  if (timCnt >= ((ledBlinking==SLOW)?25:5))
  {
    timCnt = 0;
    led = !led;
    SetLed(led);
  }
}


#define ishexdigit(x) \
       (((x >= '0') && (x <= '9')) ||   \
        ((x >= 'A') && (x <= 'F')))


unsigned char hex2dec(unsigned char x)
{
  if ((x >= '0') && (x <= '9'))
    return (x-'0');
  else
    return (10+x-'A');
}


unsigned char CheckHexMsg(unsigned char * buf, unsigned char len, unsigned char *outputMsg, unsigned char *outputLen, unsigned char *eoi)
{
  unsigned char i; 
  *eoi = 1; //default

  if (('D'==toupper(buf[1])) && (';'==buf[len-1]))
  {
    --len;
    *eoi = 0;
  }

  if ((len & 0x01) || (len < 4))
    return 0; //msg length is not even
    
  if (('0'!=buf[0]) || (('C'!=toupper(buf[1])) && ('D'!=toupper(buf[1]))))
    return 0;
    
  for (i=2; i<len; i++)
  {
    if (!ishexdigit(toupper(buf[i])))
      return 0;
  }

  *outputLen = 0;
  
  for (i=2; i<len; i=i+2)
  {
    *outputMsg = hex2dec(toupper(buf[i])) << 4;
    *outputMsg += hex2dec(toupper(buf[i+1]));
    //printf("%d ", *outputMsg);
    outputMsg++;
    *outputLen += 1;
  }
  //printf("\r\n");
  return 1;
}

char commandsHistory[BUF_SIZE*MAX_COMMANDS];
char savedCommands = 0;
char selectedCommand = 0;

unsigned char listenMode = 0;
unsigned char listenMode_prev = 0;

unsigned char buf[BUF_SIZE+4];
unsigned char msgBuf[BUF_SIZE+4];
unsigned char gpibBuf[GPIB_BUF_SIZE];

void main(void) 
{
  unsigned char bufPos = 0;
  unsigned char cursorPos = 0;
  unsigned char localEcho = 1;
  unsigned char c;
  int i;
  unsigned char gpibIndex = 0;
  unsigned char command = 0;
  int result = 0;
  unsigned char msgLen = 0;
  unsigned char msgEOI = 1;

  GPIO_init();
  
/* timer & interrupt initialize */
  TIMSK = _BV(TOIE0);        // wlacz obsluge przerwan T/C0
  TCNT0 = T0_INIT;         // wartosc poczatkowa T/C0
  TCCR0 = _BV(CS00)|_BV(CS02); // preskaler 1024
  sei();
  
  ReconfigureGPIO_GPIBNormalMode();
  UART_init();
  fdevopen(uart_putchar, NULL);
  
#if 1 
  if (0 == (PINB & _BV(PB5))) // printer mode
  {
    ledBlinking = SLOW;
    ReconfigureGPIO_GPIBReceiveMode();
    _delay_ms(1);

    while (1)
    {
      result = GPIB_Receive(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
      if (gpibIndex != 0)
      {
        for (i=0; i<gpibIndex; i++)
          UART_transmit(gpibBuf[i]);
      }
      else
      {
        _delay_ms(10);
      }
    }
  }

  localEcho = (PINB & _BV(PB7))?1:0;
#endif

  SetLed(1);
  
  while (1) //main loop
  {
    selectedCommand = savedCommands;
    
    if (localEcho && !bufPos)
      printf("<GPIB> ");
      
    do
    {
      while (!UARTDataAvailable());
      c = UART_receive();

      if (0x08 == c) //backspace
      {
        if ((bufPos > 0) && (cursorPos == bufPos))
        {
          --bufPos;
          --cursorPos;
          if (localEcho)
          {
            UART_transmit(0x08);
            UART_transmit(' ');
            UART_transmit(0x08);
          }
        }
        else if ((bufPos > 0) && (cursorPos > 0))
        {
          --bufPos;
          --cursorPos;
          memmove(&buf[cursorPos], &buf[cursorPos+1], bufPos-cursorPos);
          if (localEcho)
          {
            UART_transmit(0x08);
            buf[bufPos] = 0;
            printf("%s ", &buf[cursorPos]);
            for (i=cursorPos; i<(bufPos+1); i++)
              UART_transmit(0x08);
          }
        }
      }
      else if (10 == c) //ignore LF
      {
      }
/*      else if (9 == c) //tab key
      {
        printf("<bufPos=%d cursorPos=%d>", bufPos, cursorPos);
      }
*/      else if (0x1b == c) //escape character
      {
        switch (UART_RcvEscapeSeq())
        {
          case ESC_KEY_UP:
            selectedCommand = selectedCommand?selectedCommand-1:0;
            memmove(&buf[0], &commandsHistory[selectedCommand*BUF_SIZE], BUF_SIZE);
            if (localEcho)
            {
              while (cursorPos < bufPos)
              {
                UART_transmit(' ');
                cursorPos++;
              }
              while (bufPos--)
              {
                UART_transmit(0x08);
                UART_transmit(' ');
                UART_transmit(0x08);
              }
              printf("%s", &buf[0]);
            }
            bufPos = strlen((char*)&buf[0]);
            cursorPos = bufPos;
            break;
            
          case ESC_KEY_DOWN:
            if ((selectedCommand+1) == savedCommands) //current command is last command in buffer
            {
              selectedCommand = savedCommands;
              if (localEcho)
              {
                while (cursorPos < bufPos)
                {
                  UART_transmit(' ');
                  cursorPos++;
                }
                while (bufPos--)
                {
                  UART_transmit(0x08);
                  UART_transmit(' ');
                  UART_transmit(0x08);
                }
              }
              bufPos = 0;
              cursorPos = 0;
            }
            else if ((selectedCommand+1) < savedCommands) // <MAX_COMMANDS
            {
              selectedCommand++;
              memmove(&buf[0], &commandsHistory[selectedCommand*BUF_SIZE], BUF_SIZE);
              if (localEcho)
              {
                while (cursorPos < bufPos)
                {
                  UART_transmit(' ');
                  cursorPos++;
                }
                while (bufPos--)
                {
                  UART_transmit(0x08);
                  UART_transmit(' ');
                  UART_transmit(0x08);
                }
                printf("%s", &buf[0]);
              }
              bufPos = strlen((char*)&buf[0]);
              cursorPos = bufPos;
            }
            break;
            
          case ESC_KEY_LEFT:
            if (cursorPos)
            {
              --cursorPos;
              if (localEcho)
              {
                UART_transmit(0x1B);
                UART_transmit(0x5B);
                UART_transmit('D');
              }
            }
            break;
            
          case ESC_KEY_RIGHT:
            if (cursorPos < bufPos)
            {
              cursorPos++;
              if (localEcho)
              {
                UART_transmit(0x1B);
                UART_transmit(0x5B);
                UART_transmit('C');
              }
            }
            break;
            
          default:
            break;
        }
      }
      else if (13 == c)
      {
        if (localEcho)
        {
          UART_transmit(13); //CR
          UART_transmit(10); //LF
        }
		
        if (bufPos)
          command = toupper(buf[0]);
        else
          command = EMPTY_LINE;
      }
      else
      {
        if (bufPos < BUF_SIZE-1)
        {
          if (cursorPos == bufPos)
          {
            buf[bufPos++] = c;
            cursorPos++;
            if (localEcho)
              UART_transmit(c); //local echo
          }
          else
          {
            memmove(&buf[cursorPos+1], &buf[cursorPos], bufPos-cursorPos);
            buf[cursorPos++] = c;
            bufPos++;
            buf[bufPos] = 0;
            if (localEcho)
            {
              UART_transmit(c); //local echo
              printf("%s", &buf[cursorPos]);
              for (i=cursorPos; i<bufPos; i++)
                UART_transmit(0x08);
            }
          }
        }
      }
    } while (!command);
    

    if ('D' == command) //send data
    {
      if (!listenMode)
      {
        if (1 == msgEndSeq)
          buf[bufPos++] = 13; //CR
        else if (2==msgEndSeq)
          buf[bufPos++] = 10; //LF
        else if (3==msgEndSeq)
        {
          buf[bufPos++] = 13; //CR
          buf[bufPos++] = 10; //LF
        }
		
        result = GPIB_Transmit(buf+1, bufPos-1, 1); 
        if (result == 255) // transmit ok
          printf("OK\r\n");
        else //timeout
          printf("TIMEOUT\r\n");

        if ((1==msgEndSeq) || (2==msgEndSeq))
          --bufPos;
        else if (3==msgEndSeq)
          bufPos -= 2;
      }
      else
        printf("ERROR\r\n");	  
    }
    else if ('M' == command) //send data without EOI
    {
      if (!listenMode)
      {
        if (1 == msgEndSeq)
          buf[bufPos++] = 13; //CR
        else if (2==msgEndSeq)
          buf[bufPos++] = 10; //LF
        else if (3==msgEndSeq)
        {
          buf[bufPos++] = 13; //CR
          buf[bufPos++] = 10; //LF
        }
		
        result = GPIB_Transmit(buf+1, bufPos-1, 0); 
        if (result == 255) // transmit ok
          printf("OK\r\n");
        else //timeout
          printf("TIMEOUT\r\n");

        if ((1==msgEndSeq) || (2==msgEndSeq))
          --bufPos;
        else if (3==msgEndSeq)
          bufPos -= 2;
      }
      else
        printf("ERROR\r\n");	  
    }
    else if ('C' == command) //send command
    {
      for (i=1; i<bufPos; i++)
      {
        if ((buf[i] == '?') || (buf[i] == (64+listenAddress)))//unlisten
        {
          listenMode = 0;
          ledBlinking = OFF;
          SetLed(1);
        }
        else if (buf[i] == (32+listenAddress))
        {
          listenMode = 1;
          ledBlinking = FAST;
        }
      }

      if (1 == msgEndSeq)
        buf[bufPos++] = 13; //CR
      else if (2==msgEndSeq)
        buf[bufPos++] = 10; //LF
      else if (3==msgEndSeq)
      {
        buf[bufPos++] = 13; //CR
        buf[bufPos++] = 10; //LF
      }

      ReconfigureGPIO_GPIBNormalMode();

      SetATN(0);
      _delay_us(100);
      result = GPIB_Transmit(buf+1, bufPos-1, 1);
     
      if (result == 255) // transmit ok
        printf("OK\r\n");
      else //timeout
        printf("TIMEOUT\r\n");

      SetATN(1);
	  
      if ((1==msgEndSeq) || (2==msgEndSeq))
        --bufPos;
      else if (3==msgEndSeq)
        bufPos -= 2;
      
      if (listenMode)
        ReconfigureGPIO_GPIBReceiveMode();
      else
        ReconfigureGPIO_GPIBNormalMode();
		  
      listenMode_prev = listenMode;
    }
    else if ('R' == command)
    {
      SetREN(0);
      remoteState = 1;
      printf("OK\r\n");
    }
    else if ('L' == command)
    {
      SetREN(1);
      remoteState = 0;
      printf("OK\r\n");
    }
    else if ('I' == command)
    {
      SetIFC(0);
      _delay_ms(1);
      SetIFC(1);
      if (listenMode)
      {
        listenMode = 0;
        ledBlinking = OFF;
        SetLed(1);
        ReconfigureGPIO_GPIBNormalMode();
      }
      printf("OK\r\n");
    }
    else if ('S' == command)
    {
      UART_transmit(remoteState?'1':'0');
      UART_transmit((0 == (PINC & SRQ))?'1':'0');
      UART_transmit(listenMode?'1':'0');
      UART_transmit(13);
      UART_transmit(10);
    }
    else if ('P' == command)
    {
      listenMode_prev = 0; //cancel listen mode
      listenMode = 0; //cancel listen mode
      ledBlinking = SLOW;
      ReconfigureGPIO_GPIBReceiveMode();
//      if (localEcho)
//        printf("PRINTER MODE, send <ESC> to return to normal mode\r\n");
        
      _delay_ms(1);
      while (c != 27)
      {
        if (UARTDataAvailable())
          c = UART_receive();
          
        result = GPIB_Receive(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
        if (gpibIndex != 0)
        {
          for (i=0; i<gpibIndex; i++)
            UART_transmit(gpibBuf[i]);
        }
        else
        {
          _delay_ms(10);
        }
      }
      c = 0;
      ReconfigureGPIO_GPIBNormalMode();
      ledBlinking = OFF;
      SetLed(1);
    }
    else if ('X' == command) //ascii receive
    {
      if (!listenMode)
      {
        ReconfigureGPIO_GPIBReceiveMode();
        _delay_ms(1);
      }
      result = GPIB_Receive_till_eoi(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
      
      if (gpibIndex != 0)
      {
        gpibBuf[gpibIndex] = 0;
        printf("%s",gpibBuf);
      }
      else
        printf("TIMEOUT\r\n");

      if (!listenMode)
        ReconfigureGPIO_GPIBNormalMode();
    }
    else if ('Y' == command) //binary receive
    {
      if (!listenMode)
      {
        ReconfigureGPIO_GPIBReceiveMode();
        _delay_ms(1);
      }

      result = GPIB_Receive_till_eoi(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
      UART_transmit(gpibIndex);
      for (i=0; i<gpibIndex; i++)
      {
        UART_transmit(gpibBuf[i]);
      }
      
      if (!listenMode)
        ReconfigureGPIO_GPIBNormalMode();
    }
    else if ('Z' == command) //hex receive
    {
      if (!listenMode)
      {
        ReconfigureGPIO_GPIBReceiveMode();
        _delay_ms(1);
      }
      result = GPIB_Receive_till_eoi(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);

      printf("%02x", gpibIndex);
      for (i=0; i<gpibIndex; i++)
        printf("%02x",gpibBuf[i]);
      printf("\r\n");
	  
      if (!listenMode)
        ReconfigureGPIO_GPIBNormalMode();
    }
    else if ('?' == command)
    {
      ShowHelp();
    }
    else if ('E' == command)
    {
      if (bufPos == 1)
        printf("%d\r\n", localEcho);
      else if ((bufPos==2) && ('0' == buf[1]))
      {
        localEcho = 0;
        printf("OK\r\n");
      }
      else if ((bufPos==2) && ('1' == buf[1]))
      {
        localEcho = 1;
        printf("OK\r\n");
      }
      else
        printf("ERROR\r\n");
    }
    else if ('H' == command) //show history
    {
      for (i=0; i<savedCommands; i++)
        printf("%d: %s\r\n", i, &commandsHistory[i*BUF_SIZE]);
        
      command = 0; //to avoid saving this command in history
    }
    else if ('A' == command) //listen address
    {
      if (bufPos == 1)
        printf("%02d\r\n", listenAddress);
      else if ((bufPos==3) && isdigit(buf[1]) && isdigit(buf[2])) 
      {
        i = atoi((char*)buf+1);
        if ((i>=0) && (i<=30))
        {
          listenAddress = i;
          printf("OK\r\n");
        }
        else
          printf("ERROR\r\n");
      }
      else
        printf("ERROR\r\n");
    }
    else if ('Q' == command) 
    {
      if (bufPos == 1)
        printf("%d\r\n", msgEndSeq);
      else if ((2==bufPos) && (('0'==buf[1]) ||  ('1'==buf[1]) || ('2'==buf[1]) || ('3'==buf[1])))
      {
        if ('0'==buf[1])
          msgEndSeq = 0;
        else if ('1'==buf[1])
          msgEndSeq = 1;
        else if ('2'==buf[1])
          msgEndSeq = 2;
        else if ('3'==buf[1])
          msgEndSeq = 3;
		  
        printf("OK\r\n");
      }
      else
        printf("ERROR\r\n");
    }
    else if ('T' == command) 
    {
      if (CheckHexMsg(&buf[1], bufPos-1, msgBuf, &msgLen, &msgEOI))
      {
        if ('D' == toupper(buf[2])) //send data
        {	
          result = GPIB_Transmit(msgBuf, msgLen, msgEOI); 
          if (result == 255) // transmit ok
            printf("OK\r\n");
          else //timeout
            printf("TIMEOUT\r\n");
        }
        else //send command
        {
          for (i=0; i<msgLen; i++)
          {
            if ((msgBuf[i] == '?') || (msgBuf[i] == (64+listenAddress)))//unlisten
            {
              listenMode = 0;
              ledBlinking = OFF;
              SetLed(1);
            }
            else if (buf[i] == (32+listenAddress))
            {
              listenMode = 1;
              ledBlinking = FAST;
            }
          }

          ReconfigureGPIO_GPIBNormalMode();

          SetATN(0);
          _delay_us(100);
          result = GPIB_Transmit(msgBuf, msgLen, 1);
     
          if (result == 255) // transmit ok
            printf("OK\r\n");
          else //timeout
            printf("TIMEOUT\r\n");

          SetATN(1);
	       
          if (listenMode)
            ReconfigureGPIO_GPIBReceiveMode();
          else
            ReconfigureGPIO_GPIBNormalMode();
		  
          listenMode_prev = listenMode;
        }      
      }
      else
        printf("ERROR\r\n");
    }
    else
    {
      if (bufPos)
        printf("WRONG COMMAND\r\n");
      command = 0;
    }

    if (command && bufPos)
    {
      buf[bufPos] = 0; //add string termination

      //avoids saving same command twice    
      if ((savedCommands > 0) && (0 == strcmp(&commandsHistory[(savedCommands-1)*BUF_SIZE], (char*)&buf[0])))
      {
        command = 0;
      }
      else //save command
      {
        if (savedCommands < MAX_COMMANDS)
        {
          memmove(&commandsHistory[savedCommands*BUF_SIZE], &buf[0], BUF_SIZE);
          ++savedCommands;
        }
        else
        {
          memmove(&commandsHistory[0], &commandsHistory[BUF_SIZE], BUF_SIZE*(savedCommands-1));
          memmove(&commandsHistory[(savedCommands-1)*BUF_SIZE], &buf[0], BUF_SIZE);
        }
      }
    }
	
    command = 0;
    bufPos = 0;
    cursorPos = 0;
    buf[0] = 0;
  } //end of endless loop block
}
