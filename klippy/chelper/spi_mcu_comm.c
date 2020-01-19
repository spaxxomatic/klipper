#include <stdarg.h>
#include <fcntl.h> // fcntl
#include <math.h> // ceil
#include <poll.h> // poll
#include <pthread.h> // pthread_mutex_lock
#include <stddef.h> // offsetof
#include <stdint.h> // uint64_t
#include <stdio.h> // snprintf
#include <stdlib.h> // malloc
#include <string.h> // memset
#include <errno.h>
#include <unistd.h> // pipe
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "compiler.h" // __visible
#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include "gpio.h"
#endif 

#include "serialqueue.h" //MESSAGE_SYNC , serialqueue_alloc, etc
#include "spi_mcu_comm.h"
#include "trace.h"

int spi_fd = 0;

int slaveSignalHasBeenSetup = 0;

uint8_t spi_mode = SPI_MODE_0;
uint8_t spi_bits = SPI_BITS;
uint32_t spi_speed ;

void setupSlaveIrq(){
    if (slaveSignalHasBeenSetup) return;
    #ifdef USE_WIRINGPI
    wiringPiSetup();
    pinMode(SPI_SLAVE_IRQ_PIN_WIRINGPI, INPUT);
	// Bind to interrupt
	wiringPiISR(SPI_SLAVE_IRQ_PIN, INT_EDGE_FALLING, &handleSlaveIrq);
    #else
    //using pin polling
    setup_io(SPI_SLAVE_IRQ_GPIO_PIN);
    #endif
    slaveSignalHasBeenSetup = 1;
}

struct spi_ioc_transfer xfer[2];

void prepare_read_buff(){
    memset(&xfer[0], 0, sizeof (xfer[0]));
    memset(&xfer[1], 0, sizeof (xfer[1]));
	xfer[0].delay_usecs = SPI_DELAY;
	xfer[0].speed_hz = spi_speed;
	xfer[0].bits_per_word = SPI_BITS;
}

int fd_is_valid(int fd)
{
    return fcntl(fd, F_GETFD) != -1 || errno != EBADF;
}


void __visible close_spi() {
    if (spi_fd != 0){
        close(spi_fd);
    }
}

int __visible setup_spi_comm(char* spi_device, uint32_t speed){
    spi_speed = speed;
	if (spi_fd != 0) {
		if (fd_is_valid(spi_fd))
			return spi_fd;
		else  {
			pabort("SPI has been opened but the handle is invalid");
		}
	}
	//setupSlaveIrq();
    prepare_read_buff();
	spi_fd = open(spi_device, O_RDWR);
	if (spi_fd < 0)
		pabort("can't open spi device");
	
	int ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode);
	if (ret == -1)
		pabort("can't get spi mode");

	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits);    
	if (ret == -1)
		pabort("can't get bits per word");

	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	trace_msg(2,"SPI opened\n");
	trace_msg(2,"SPI speed: %d KHz\n", spi_speed/1000);
    
    return spi_fd;
}
/*
int spi_read() {
	int ret = 0;
    // If the MCU data tranfer pin is still high, we have data to read
    if (getPinStatus(SPI_SLAVE_IRQ_GPIO_PIN) == 0) {        
        return spi_read_buff.len ;
    }
    //trace_msg(3,"rx1 ! ");    
    int packet_len = 0;
    if (spi_read_buff.this_message_len > 0){
        trace_msg(0,"ERR: read while buffer not clean\n");
        return -1;
    }

    //We have been called by the timer that checks the MCU transfer requests (SQPT_READ_SPI)
    //We have to read two bytes, the first byte must be escape and the second the message length
    char buff[3] = {NULL_BYTE_MASTER, NULL_BYTE_MASTER, NULL_BYTE_MASTER}; //need to set this value so that the MCU does not confuse it with a data packet
    xfer[0].rx_buf = (unsigned long)&buff;
    xfer[0].len = 2;
    xfer[0].tx_buf = (unsigned long)&buff;
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (ret < 1) 
        pabort("can't read spi message");
    
    trace_buffer("RXLEN ", buff, 3);
    return spi_read_buff.len;
}
*/

int spi_sendbyte(char tbyte)
{
	int ret = 0;
    
    trace_buffer("WB", (char*) &tbyte, 1);
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)&tbyte,
		.len = 1,
		.speed_hz = spi_speed,
		.bits_per_word = SPI_BITS,
	};

    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        trace_msg(0,"can't send spi data\n");
        ret = -1;
    }
    
    return ret;
}

int spi_write(char* inp_buff, int buff_len)
{
	int ret = 0;
    
    char* rx_buff = malloc(buff_len); 
    
    trace_buffer("WR", (char*) inp_buff, buff_len);
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)inp_buff,
		.rx_buf = (unsigned long)rx_buff,
		.len = buff_len,
		.delay_usecs = SPI_DELAY,
		.speed_hz = spi_speed,
		.bits_per_word = SPI_BITS,
	};

    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        trace_msg(0,"can't send spi data\n");
        ret = -1;
    }
    free(rx_buff);
    
    return ret;
}
