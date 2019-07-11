#include <ioCC2530.h>

#include "printf.h"

#define BV(n)  (1 << (n))

#define int8     signed char
#define uint8  unsigned char
#define int16    signed short
#define uint16 unsigned short

#define UART0_PERCFG_BV    BV(0) // UART0 I/O location: 0-Alt_1, 1-Alt_2
#define UART0_ALT_1_RX_TX  0x0C  // UART0 I/O at Alt_1: P0_3-TX, P0_2-RX
#define UART0_CSR_MODE_BV  BV(7) // Select mode: 0-SPI, 1-UART
#define UART0_RECEIVER_BV  BV(6) // Enable receiver: 0-disable, 1-enable
#define UART0_STOP_BIT_BV  BV(1) // Stop bit level: 0-low, 1-high
#define UART0_CONTROL      UART0_STOP_BIT_BV

void ON_32MOSC( void );
void delay_ms( uint16 ms );
void UART0_Init( void );
void UART0_Send( uint8 *buffer, uint16 len );

void ON_32MOSC( void )
{
    CLKCONCMD &= ~0x40;
    while(CLKCONSTA & 0x40);
    CLKCONCMD &= ~0x47;
}
void delay_ms( uint16 ms )
{
    uint16 i,j;

    for ( i=0; i<ms; i++ )
        for (j=0; j<535*2; j++);
}
void UART0_Init( void )
{
  PERCFG &= ~UART0_PERCFG_BV;   // Set UART0 to Alt_1: P0_3-TX, P0_2-RX
  P0SEL  |=  UART0_ALT_1_RX_TX; // Enable peripheral mode for I/O
  U0CSR  |=  UART0_CSR_MODE_BV; // Work in UART mode

  U0BAUD = 216;                 // set the baud rate to 115200
  U0GCR  = 11;

  U0UCR  = UART0_CONTROL;       // 8 bits, no parity, 1 stop bit, stop bit high
}
void UART0_Send( uint8 *buffer, uint16 len )
{
  uint16 i;

  for( i=0; i<len; i++ )
  {
    U0DBUF = buffer[i];
    while( UTX0IF == 0);
    UTX0IF = 0;
  }
}

void _putchar(char character)
{
  UART0_Send( (uint8 *)&character, 1 );
}

void main( void )
{
  ON_32MOSC();

  UART0_Init();

  UART0_Send( "CC2530 UART0 OK !\r\n", sizeof( "CC2530 UART0 OK !\r\n" ) - 1 );

  while( 1 )
  {
    delay_ms( 1000 );
  }
}
