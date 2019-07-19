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

void printf_test( void );

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

  printf_test();

  while( 1 )
  {
    delay_ms( 1000 );
  }
}

void printf_test( void )
{
  printf( "  signed  char *  :  %d byte(s), %2d bit(s)\r\n", sizeof(   signed char * ),    8*sizeof(   signed char * ) );
  printf( "unsigned  char *  :  %d byte(s), %2d bit(s)\r\n", sizeof( unsigned char * ),    8*sizeof( unsigned char * ) );
  printf( "\r\n" );

  printf( "  signed short *  :  %d byte(s), %2d bit(s)\r\n", sizeof(   signed short * ),   8*sizeof(   signed short * ) );
  printf( "unsigned short *  :  %d byte(s), %2d bit(s)\r\n", sizeof( unsigned short * ),   8*sizeof( unsigned short * ) );
  printf( "\r\n" );

  printf( "  signed  long *  :  %d byte(s), %2d bit(s)\r\n", sizeof(   signed long * ),    8*sizeof(   signed long * ) );
  printf( "unsigned  long *  :  %d byte(s), %2d bit(s)\r\n", sizeof( unsigned long * ),    8*sizeof( unsigned long * ) );
  printf( "\r\n" );

  printf( "         float *  :  %d byte(s), %2d bit(s)\r\n", sizeof( float * ),            8*sizeof( float * ) );
  printf( "        double *  :  %d byte(s), %2d bit(s)\r\n", sizeof( double * ),           8*sizeof( double * ) );
  printf( "\r\n" );

  printf( "  signed char     : %d byte(s),  %2d bit(s)\r\n", sizeof(   signed char ),      8*sizeof(   signed char ) );
  printf( "unsigned char     : %d byte(s),  %2d bit(s)\r\n", sizeof( unsigned char ),      8*sizeof( unsigned char ) );
  printf( "\r\n" );

  printf( "  signed short    : %d byte(s),  %2d bit(s)\r\n", sizeof(   signed short ),     8*sizeof(   signed short ) );
  printf( "unsigned short    : %d byte(s),  %2d bit(s)\r\n", sizeof( unsigned short ),     8*sizeof( unsigned short ) );
  printf( "\r\n" );

  printf( "  signed int      : %d byte(s),  %2d bit(s)\r\n", sizeof(   signed int ),       8*sizeof(   signed int ) );
  printf( "unsigned int      : %d byte(s),  %2d bit(s)\r\n", sizeof( unsigned int ),       8*sizeof( unsigned int ) );
  printf( "\r\n" );

  printf( "  signed long     : %d byte(s),  %2d bit(s)\r\n", sizeof(   signed long ),      8*sizeof(   signed long ) );
  printf( "unsigned long     : %d byte(s),  %2d bit(s)\r\n", sizeof( unsigned long ),      8*sizeof( unsigned long ) );
  printf( "\r\n" );

  printf( "  signed long long: %d byte(s),  %2d bit(s)\r\n", sizeof(   signed long long ), 8*sizeof(  signed long long ) );
  printf( "unsigned long long: %d byte(s),  %2d bit(s)\r\n", sizeof( unsigned long long ), 8*sizeof( unsigned long long ) );
  printf( "\r\n" );

  printf( "         float    : %d byte(s),  %2d bit(s)\r\n", sizeof( float ),              8*sizeof( float ) );
  printf( "        double    : %d byte(s),  %2d bit(s)\r\n", sizeof( double ),             8*sizeof( double ) );
  printf( "\r\n" );

  printf( "        size_t    : %d byte(s),  %2d bit(s)\r\n", sizeof( size_t ),             8*sizeof( size_t ) );
  printf( "     ptrdiff_t    : %d byte(s),  %2d bit(s)\r\n", sizeof( ptrdiff_t ),          8*sizeof( ptrdiff_t ) );
  printf( "\r\n" );

//  printf( "      intmax_t    : %d byte(s),  %2d bit(s)\r\n", sizeof( intmax_t ),           8*sizeof( intmax_t ) );
//  printf( "     uintmax_t    : %d byte(s),  %2d bit(s)\r\n", sizeof( uintmax_t ),          8*sizeof( uintmax_t ) );
//  printf( "\r\n" );
//
//  printf( "      intptr_t    : %d byte(s),  %2d bit(s)\r\n", sizeof( intptr_t ),           8*sizeof( intptr_t ) );
//  printf( "     uintptr_t    : %d byte(s),  %2d bit(s)\r\n", sizeof( uintptr_t ),          8*sizeof( uintptr_t ) );
//  printf( "\r\n" );

  printf( "%8.2f\r\n", -23.45 );
  printf( "%8.2f\r\n", 456.89 );
  printf( "\r\n" );

  printf( "%8d\r\n", 32765 );
  printf( "%8d\r\n", 32766 );
  printf( "%8d\r\n", 32767 );
  printf( "%8d\r\n", 32768 ); // intentionally: maybe warning and wrong, overflow
  printf( "%8d\r\n", 32769 ); // intentionally: maybe warning and wrong, overflow
  printf( "%8d\r\n", 32770 ); // intentionally: maybe warning and wrong, overflow
  printf( "\r\n" );

  printf( "%8u\r\n", 32765 );
  printf( "%8u\r\n", 32766 );
  printf( "%8u\r\n", 32767 );
  printf( "%8u\r\n", 32768 ); // intentionally: maybe warning but right
  printf( "%8u\r\n", 32769 ); // intentionally: maybe warning but right
  printf( "%8u\r\n", 32770 ); // intentionally: maybe warning but right
  printf( "\r\n" );

  printf( "%8u\r\n", 65533 ); // intentionally: maybe warning but right
  printf( "%8u\r\n", 65534 ); // intentionally: maybe warning but right
  printf( "%8u\r\n", 65535 ); // intentionally: maybe warning but right
  printf( "%8u\r\n", 65536 ); // intentionally: maybe warning and wrong, overflow
  printf( "%8u\r\n", 65537 ); // intentionally: maybe warning and wrong, overflow
  printf( "%8u\r\n", 65538 ); // intentionally: maybe warning and wrong, overflow
  printf( "\r\n" );

  int16 testv1  = 32535;
  uint16 testv2 = 65535;
  printf( "%8d, %8u\r\n", 32535,  65535 ); // intentionally
  printf( "%8d, %8u\r\n", testv1, testv2 );
  printf( "%8d, %8u\r\n", 32535,  testv2 );
  printf( "%8d, %8u\r\n", testv1, 65535 ); // intentionally
  printf( "\r\n" );

  printf( "%8d, %8d, %8d\r\n", 30000, 32764, 32766 );
  printf( "\r\n" );

  printf( "%8u, %8u, %8u\r\n", 60000,  62764,  62766 ); // intentionally
  printf( "%8u, %8u, %8u\r\n", 60000U, 62764U, 62766U );
  printf( "\r\n" );

  printf( "%8d, %8d, %8u, %8u, %8u\r\n", 32765, 32766, 62769,  65533,  64000 ); // intentionally
  printf( "%8d, %8d, %8u, %8u, %8u\r\n", 32765, 32766, 62769U, 65533U, 64000U );
  printf( "\r\n" );

  printf( "%8d, %8d, %8u, %8u, %8d, %8u\r\n", 32765, 32766, 32768,  32769,  32765, 65533 ); // intentionally
  printf( "%8d, %8d, %8u, %8u, %8d, %8u\r\n", 32765, 32766, 32768U, 32769U, 32765, 65533U );
}
