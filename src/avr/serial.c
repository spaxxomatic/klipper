// AVR serial port code.
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <avr/interrupt.h> // USART_RX_vect
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/serial_irq.h" // serial_rx_byte
#include "command.h" // DECL_CONSTANT_STR
#include "sched.h" // DECL_INIT
#include "gpio.h" // spi_setup
#include "internal.h" // GPIO
#include "pgm.h" // READP

#if CONFIG_MACH_atmega168 || CONFIG_MACH_atmega328 || CONFIG_MACH_atmega328p
static const uint8_t MISO = GPIO('B', 4), MOSI = GPIO('B', 3);
static const uint8_t SCK = GPIO('B', 5), SS = GPIO('B', 2);
static const uint8_t TX_REQ_PIN = GPIO('B', 1);
DECL_CONSTANT_STR("BUS_PINS_spi", "PB4,PB3,PB5");
#elif CONFIG_MACH_atmega644p || CONFIG_MACH_atmega1284p
static const uint8_t MISO = GPIO('B', 6), MOSI = GPIO('B', 5);
static const uint8_t SCK = GPIO('B', 7), SS = GPIO('B', 4);
DECL_CONSTANT_STR("BUS_PINS_spi", "PB6,PB5,PB7");
#elif CONFIG_MACH_at90usb1286 || CONFIG_MACH_at90usb646 \
      || CONFIG_MACH_atmega32u4 || CONFIG_MACH_atmega1280 \
      || CONFIG_MACH_atmega2560
static const uint8_t MISO = GPIO('B', 3), MOSI = GPIO('B', 2);
static const uint8_t SCK = GPIO('B', 1), SS = GPIO('B', 0);
DECL_CONSTANT_STR("BUS_PINS_spi", "PB3,PB2,PB1");
#endif


// Reserve serial pins
#if CONFIG_SERIAL_PORT == 0
 #if CONFIG_MACH_atmega1280 || CONFIG_MACH_atmega2560
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PE0,PE1");
 #else
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD1,PD0");
 #endif
#elif CONFIG_SERIAL_PORT == 1
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD2,PD3");
#elif CONFIG_SERIAL_PORT == 2
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PH0,PH1");
#else
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PJ0,PJ1");
#endif

// Helper macros for defining serial port aliases
#define AVR_SERIAL_REG1(prefix, id, suffix) prefix ## id ## suffix
#define AVR_SERIAL_REG(prefix, id, suffix) AVR_SERIAL_REG1(prefix, id, suffix)

// Serial port register aliases
#define UCSRxA AVR_SERIAL_REG(UCSR, CONFIG_SERIAL_PORT, A)
#define UCSRxB AVR_SERIAL_REG(UCSR, CONFIG_SERIAL_PORT, B)
#define UCSRxC AVR_SERIAL_REG(UCSR, CONFIG_SERIAL_PORT, C)
#define UBRRx AVR_SERIAL_REG(UBRR, CONFIG_SERIAL_PORT,)
#define UDRx AVR_SERIAL_REG(UDR, CONFIG_SERIAL_PORT,)
#define UCSZx1 AVR_SERIAL_REG(UCSZ, CONFIG_SERIAL_PORT, 1)
#define UCSZx0 AVR_SERIAL_REG(UCSZ, CONFIG_SERIAL_PORT, 0)
#define U2Xx AVR_SERIAL_REG(U2X, CONFIG_SERIAL_PORT,)
#define RXENx AVR_SERIAL_REG(RXEN, CONFIG_SERIAL_PORT,)
#define TXENx AVR_SERIAL_REG(TXEN, CONFIG_SERIAL_PORT,)
#define RXCIEx AVR_SERIAL_REG(RXCIE, CONFIG_SERIAL_PORT,)
#define UDRIEx AVR_SERIAL_REG(UDRIE, CONFIG_SERIAL_PORT,)

#if defined(USART_RX_vect)
// The atmega168 / atmega328 doesn't have an ID in the irq names
#define USARTx_RX_vect USART_RX_vect
#define USARTx_UDRE_vect USART_UDRE_vect
#else
#define USARTx_RX_vect AVR_SERIAL_REG(USART, CONFIG_SERIAL_PORT, _RX_vect)
#define USARTx_UDRE_vect AVR_SERIAL_REG(USART, CONFIG_SERIAL_PORT, _UDRE_vect)
#endif


void
serial_init(void)
{
    UCSRxA = CONFIG_SERIAL_BAUD_U2X ? (1<<U2Xx) : 0;
    uint32_t cm = CONFIG_SERIAL_BAUD_U2X ? 8 : 16;
    UBRRx = DIV_ROUND_CLOSEST(CONFIG_CLOCK_FREQ, cm * CONFIG_SERIAL_BAUD) - 1UL;
    UCSRxC = (1<<UCSZx1) | (1<<UCSZx0);
    UCSRxB = (1<<RXENx) | (1<<TXENx) | (1<<RXCIEx) | (1<<UDRIEx);
    
}
DECL_INIT(serial_init);


void
spi_init(void)
{
    //gpio_out_setup(SS, 0);
    gpio_in_setup(SCK, 0);
    gpio_in_setup(MOSI, 0);
    gpio_out_setup(MISO, 0);      
  //setup spi as slave
  //pinMode(MISO,OUTPUT);

  //pinMode(MISO,OUTPUT);     
   SPCR=(1<<SPE)|(1<<SPIE);  // Turn on SPI in Slave Mode, turn on interrupt
  //digitalWrite(TX_REQ_PIN, LOW);  
  //pinMode(TX_REQ_PIN, OUTPUT);
   tx_req_pin = gpio_out_setup(TX_REQ_PIN, 0);      
}
DECL_INIT(spi_init);

// Rx interrupt - encoder positon data
ISR(USARTx_RX_vect)
{
    serial_rx_byte(UDRx);
}

uint_fast8_t packet_len = 0;
uint_fast8_t dbg = 0;
ISR (SPI_STC_vect)
{  
    if (packet_len >0){  
        serial_rx_byte( SPDR);
        packet_len--;
        return;
    }
    if (SPDR == NULL_BYTE_MASTER){
        SPDR = serial_get_tx_byte_escaped();

        return;
    }
    if (SPDR == MESSAGE_SYNC){
        serial_rx_byte( SPDR);
        return;
    }
    if (SPDR < MESSAGE_MAX){
        serial_rx_byte( SPDR);
        packet_len = SPDR-1;
    };
 }


ISR(USARTx_UDRE_vect)
{
    /*uint8_t data;
    int ret = serial_get_tx_byte(&data);
    if (ret)
        UCSRxB &= ~(1<<UDRIEx);
    else
        UDRx = data;
    */
     UDRx = 'a';
}