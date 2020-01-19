
#ifndef SPI_MCU_COMM_H
#define SPI_MCU_COMM_H

#define SPI_SLAVE_IRQ_PIN_WIRINGPI 7 //wiring pi 7 is BCM 4 - GPIO 7
#define SPI_SLAVE_IRQ_GPIO_PIN 4 //wiring pi 7 is BCM 4 - GPIO 7

#define SPI_DELAY 0
#define SPI_SPEED 80000
#define SPI_BITS 8

int setup_spi_comm(char* spi_device, uint32_t speed);

void close_spi() ;
void check_mcu_data_pending();

#define MCU_DATA_PENDING getPinStatus(SPI_SLAVE_IRQ_GPIO_PIN) > 0

//int spi_read();
int spi_sendbyte(char tbyte);
int spi_write(char* inp_buff, int buff_len);


#endif