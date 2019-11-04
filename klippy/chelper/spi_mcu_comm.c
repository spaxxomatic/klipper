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
//#include <termios.h> // tcflush
#include <unistd.h> // pipe
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "compiler.h" // __visible
#include <wiringPi.h>

#include "serialqueue.h" //MESSAGE_SYNC , serialqueue_alloc, etc
#include "spi_mcu_comm.h"
#include "trace.h"

int spi_fd = 0;
struct serialqueue* thissq;
//extern struct serialqueue * serialqueue_alloc(int serial_fd, int write_only);
//extern void handle_rx_data(struct serialqueue *sq, char* data, int len);


// Handler for pin interrupt
void handleSlaveIrq(void) {
    trace_msg(1,"Slave IRQ");
    spi_read();
}

int irqHasBeenSetup = 0;

void setupSlaveIrq(){
    if (irqHasBeenSetup) return;
    wiringPiSetup();
    pinMode(SPI_SLAVE_IRQ_PIN, INPUT);
	// Bind to interrupt
	wiringPiISR(SPI_SLAVE_IRQ_PIN, INT_EDGE_FALLING, &handleSlaveIrq);
    irqHasBeenSetup = 1;
}


typedef struct  __spi_read_buff{
	char data[4096];
	int len;
} t_spi_read_buff;

t_spi_read_buff spi_read_buff;

struct spi_ioc_transfer xfer[2];

char sync_signal = MESSAGE_SYNC;
void prepare_read_buff(){
    memset(&xfer[0], 0, sizeof (xfer[0]));
    memset(&xfer[1], 0, sizeof (xfer[1]));
	xfer[0].delay_usecs = SPI_DELAY;
	xfer[0].speed_hz = SPI_SPEED;
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

int setup_spi_comm(char* spi_device){
    uint8_t spi_mode = SPI_MODE_0;
    uint8_t spi_bits = SPI_BITS;
    uint32_t spi_speed = SPI_SPEED; //TODO: move to the ini file

	if (spi_fd != 0) {
		if (fd_is_valid(spi_fd))
			return spi_fd;
		else  {
			pabort("SPI has been opened but the handle is invalid");
		}
	}
	set_trace_level(3);
	setupSlaveIrq();
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

	trace_msg(1,"SPI opened \r\n");
	trace_msg(1,"SPI speed: %d KHz\r\n", spi_speed/1000);
	return spi_fd;
}

void cleanup_and_send(char* buff, int len){
    char* rx_clean_buff = malloc(len); //when we write len bytes, we can receive len-1
      //handle_rx_data(sq, &rx_buff[1], len-1);
        int i = 0;
        int j=0;
        for (i = 0; i < len; i++){
            if (buff[i] != 0xFF){
                rx_clean_buff[j++] = buff[i];
            }
        }

        printf("RC:  ");
        for ( i = 0; i < j; i++) {
                printf("%.2X:", rx_clean_buff[i]);
            }
            printf("\r\n");	

        handle_rx_data(thissq, rx_clean_buff, j);
    free(rx_clean_buff);
}
//handler for slave data tranfer requests
int spi_read()
{
	int ret = 0;    
    //char* spi_buff = malloc(len+1); 
    //spi_buff[0] = 0x00; //indicates read request
    
    //memset(spi_buff, 0, len+1);
    int len = 32; //nutiu TODO - this should come from slave
    
    char* buff = malloc(len);
    memset(buff, 0xFF, len);
    xfer[0].rx_buf = (unsigned long)buff;
	xfer[0].len = len;
    xfer[0].tx_buf = (unsigned long)buff;
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer);

    /*xfer[0].tx_buf = (unsigned long)&sync_signal;
	xfer[0].len = 1;    
    xfer[1].rx_buf = (unsigned long)&spi_read_buff.data;
	xfer[1].len = len;
    
	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(2), xfer);
    */
	trace_msg(0,"SPI read !!");
	if (ret < 1) {
        pabort("can't read spi message");
    }else{
        //the first byte is dirt because of the way SPI works, so the second one is our status
        trace_msg(2,"RX: ");
        int i = 0;
        for ( i = 0; i < len; i++) {
            trace_msg(2,"%.2X:", buff[i]);
        }
        trace_msg(2,"\r\n");
    }
    //cleanup_and_send(spi_read_buff.data, len);
    cleanup_and_send(buff, len);
    //handle_rx_data(thissq, spi_read_buff.data, len);
    return ret;
}

int spi_write(struct serialqueue *sq,  char* inp_buff, int buff_len, int is_retransmit)
{
	int ret = 0;
    int len = 48;
    char* rx_buff = malloc(len); //when we write len bytes, we can receive len-1
    char* tx_buff = malloc(len); //when we write len bytes, we can receive len-1

    memset(rx_buff, 0xFF, len);
    memset(tx_buff, 0xFF, len);
    memcpy(tx_buff,inp_buff, buff_len);

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buff,
		.rx_buf = (unsigned long)rx_buff,
		.len = len,
		.delay_usecs = SPI_DELAY,
		.speed_hz = SPI_SPEED,
		.bits_per_word = SPI_BITS,
	};

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr, len);
    int i = 0;
	if (ret < 1) {
        trace_msg(0,"can't send spi message");
        ret = -1;
    }else{

        trace_msg(2,"TX%i ", len);
        
        for ( i = 0; i < len; i++) {
            printf("%.2X:", tx_buff[i]);
        }
		printf("\r\n");
        
        printf("TXR: ");
        for ( i = 0; i < len; i++) {
            printf("%.2X:", rx_buff[i]);
        }
        printf("\r\n");
                
        cleanup_and_send(rx_buff, len);
    }
    free(rx_buff);
    return ret;
}

int _orig_spi_write(struct serialqueue *sq, char* tx_buff, int len, int is_retransmit)
{
	int ret = 0;
    char* rx_buff = malloc(len); //when we write len bytes, we can receive len-1

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buff,
		.rx_buf = (unsigned long)rx_buff,
		.len = len,
		.delay_usecs = SPI_DELAY,
		.speed_hz = SPI_SPEED,
		.bits_per_word = SPI_BITS,
	};

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr, len);
    int i = 0;
	if (ret < 1) {
        trace_msg(0,"can't send spi message");
        ret = -1;
    }else{

        trace_msg(2,"TX%i ", len);
        
        for ( i = 0; i < len; i++) {
            printf("%.2X:", tx_buff[i]);
        }
		printf("\r\n");
        
        printf("TXR: ");
        for ( i = 0; i < len; i++) {
            printf("%.2X:", rx_buff[i]);
        }
        printf("\r\n");
                
        cleanup_and_send(rx_buff, len);
    }
    free(rx_buff);
    return ret;
}

struct serialqueue * __visible
spiqueue_alloc(char* spi_device, int write_only)
{
    thissq = serialqueue_alloc(setup_spi_comm(spi_device), write_only);   
	return thissq;
}