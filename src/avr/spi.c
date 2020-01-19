// Serial Peripheral Interface (SPI) support
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_atmega644p
#include "command.h" // shutdown
#include "gpio.h" // spi_setup
#include "internal.h" // GPIO
#include "pgm.h" // READP
#include "sched.h" // sched_shutdown
#include "stepper.h" // command_position_adjust

DECL_ENUMERATION("spi_bus", "spi", 0);

#if CONFIG_MACH_atmega168 || CONFIG_MACH_atmega328 || CONFIG_MACH_atmega328p
static const uint8_t MISO = GPIO('B', 4), MOSI = GPIO('B', 3);
static const uint8_t SCK = GPIO('B', 5), SS = GPIO('B', 2);
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

static const uint8_t TX_REQ_PIN = GPIO('B', 1);
struct gpio_out tx_req_pin;

void
spi_init(void)
{
  //setup spi as slave
  //gpio_out_setup(SS, 0);
  gpio_in_setup(SCK, 0);
  gpio_in_setup(MOSI, 0);
  gpio_out_setup(MISO, 0);      

  SPCR=(1<<SPE)|(1<<SPIE);  // Turn on SPI in Slave Mode, turn on interrupt
  //tx_req_pin = gpio_out_setup(TX_REQ_PIN, 0);      
}
DECL_INIT(spi_init);

// Ths spi interface is used to receive the actual position information from the linear encoders
ISR (SPI_STC_vect)
{  
    //the first two bits endcode the OID (0 to 4)
    //the rest of 6 bits are the delta information
    uint8_t oid = SPDR>>6;
    int8_t pos_delta = SPDR&0x3f;
    command_position_adjust(oid, pos_delta);
    SPDR = MESSAGE_SYNC;
}
