
#ifndef SPI_MCU_COMM_H
#define SPI_MCU_COMM_H

#define SPI_SLAVE_IRQ_PIN 7 //wiring pi 7 is BCM 4 - GPIO 7

#define SPI_DELAY 0
#define SPI_SPEED 10000
#define SPI_BITS 8

struct serialqueue;
struct serialqueue * spiqueue_alloc(char* spi_device, int write_only);
void close_spi() ;

int spi_read();
int spi_write(struct serialqueue *sq, char* tx_buff, int len, int is_retransmit);


#endif