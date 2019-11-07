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

//yes, global variable, absolute buggy in case we have more than one SPI interface
struct serialqueue* thissq = 0; 

//extern struct serialqueue * serialqueue_alloc(int serial_fd, int write_only);
//extern void handle_rx_data(struct serialqueue *sq, char* data, int len);

// Handler for pin interrupt
void handleSlaveIrq(void) {
    trace_msg(1,"Slave IRQ\n");
    //spi_read();
    if (thissq){ //in case we're getting an irq before being inistialized
        //schedule a data transmission        
        kick_mcu_data_transfer(thissq);
    }
}

int irqHasBeenSetup = 0;

uint8_t spi_mode = SPI_MODE_0;
uint8_t spi_bits = SPI_BITS;
uint32_t spi_speed ;

void setupSlaveIrq(){
    if (irqHasBeenSetup) return;
    wiringPiSetup();
    pinMode(SPI_SLAVE_IRQ_PIN, INPUT);
	// Bind to interrupt
	wiringPiISR(SPI_SLAVE_IRQ_PIN, INT_EDGE_FALLING, &handleSlaveIrq);
    irqHasBeenSetup = 1;
}

t_spi_read_buff spi_read_buff;

struct spi_ioc_transfer xfer[2];

char sync_signal = MESSAGE_SYNC;

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

int setup_spi_comm(char* spi_device, uint32_t speed){
    spi_speed = speed;
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

	trace_msg(1,"SPI opened\n");
	trace_msg(1,"SPI speed: %d KHz\n", spi_speed/1000);
	return spi_fd;
}

void check_and_buffer(char* buff, int len){
    int i = 0;
    for ( i = 0; i < len; i++) {
        char rbyte = buff[i];
        if (rbyte != MESSAGE_ESCAPE){ 
            //if this is the first non-empty byte outside of a message, it must be the beginning of a message
            if (spi_read_buff.this_message_len == 0){
                spi_read_buff.this_message_len = rbyte ; //+1 because we need to add also the length byte to the buff
                trace_msg(2,"Msg begin %i bytes\n", rbyte);

            }
        }
        if (spi_read_buff.this_message_len > 0){
            spi_read_buff.data[spi_read_buff.len++] = rbyte;
            spi_read_buff.this_message_len --;
        }
    }
    if (spi_read_buff.this_message_len > 0){
        //there are still bytes to read
        trace_msg(2,"Kick transfer for rest of %i bytes\n", spi_read_buff.this_message_len);
        kick_mcu_data_transfer(thissq);
    }
}

//handler for slave data tranfer requests
#define MSG_ENDMARK  0xFF

int
spi_read(struct serialqueue *sq)
{
	int ret = 0;
    int expected_bytes = 0;
    if (spi_read_buff.this_message_len > 0){
        //Due to the nature of the SPI, we have a simultaneous backchannel, and we use it to receive data while we're sending
        //If this_message_len is not 0, some data message was read by the TX routine but there is still data remaining on the remote
        //Means, the MCU triggered the read irq while we were sending data. 
        
        expected_bytes = spi_read_buff.this_message_len; 
    }else{
        //We have to read one byte to know the message length
        expected_bytes = MESSAGE_ESCAPE; //need to set this value so that the MCU does not confuse it with a data packet
        xfer[0].rx_buf = (unsigned long)&expected_bytes;
        xfer[0].len = 1;
        xfer[0].tx_buf = (unsigned long)&expected_bytes;
        ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer);
        if (ret < 1) 
            pabort("can't read spi message");
        //add this valid byte to the output buffer         
        trace_msg(3, "Asked for len, got %i \n", expected_bytes);
        if (expected_bytes<MESSAGE_MIN || expected_bytes>MESSAGE_MAX ){
            trace_msg(0, "Internal error. Len is invalid: %i\n", expected_bytes); 
            return -1;
        }
        spi_read_buff.data[spi_read_buff.len++] = expected_bytes;
    }
    
    //now read the message body
    int transmission_len = expected_bytes+2;
    char* buff = malloc(transmission_len); //last two must then be be MESSAGE_ESCAPE
    memset(buff, MESSAGE_ESCAPE, transmission_len); //we send only MESSAGE_ESCAPE's
    xfer[0].rx_buf = (unsigned long)buff;
	xfer[0].len = transmission_len;
    xfer[0].tx_buf = (unsigned long)buff;
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer);
	trace_msg(0,"SPI read !!");
	if (ret < 1) {
        pabort("can't read spi message");
    }else{
        trace_msg(2,"RX: ");
        int i = 0;
        for ( i = 0; i < expected_bytes; i++) {
            char rbyte = buff[i];
            trace_msg(2,"%.2X:", rbyte);
            spi_read_buff.data[spi_read_buff.len++] = rbyte;
        }
        trace_msg(2,"And the last two are %.2X and %.2X \r\n", buff[expected_bytes], buff[expected_bytes + 1]);

        trace_msg(2,"\r\n");
    }
    //check_and_buffer(buff, expected_bytes);
    spi_read_buff.this_message_len = 0;//done reading remote buffer
    return ret;
}


int spi_write(struct serialqueue *sq,  char* inp_buff, int buff_len, int is_retransmit)
{
	int ret = 0;
    
    char* rx_buff = malloc(buff_len); //when we write len bytes, we can receive len-1
    
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)inp_buff,
		.rx_buf = (unsigned long)rx_buff,
		.len = buff_len,
		.delay_usecs = SPI_DELAY,
		.speed_hz = spi_speed,
		.bits_per_word = SPI_BITS,
	};

    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    
    int i = 0;
	if (ret < 1) {
        trace_msg(0,"can't send spi message\n");
        ret = -1;
    }else{

        trace_msg(2,"TX%i ", tr.len);
        
        for ( i = 0; i < tr.len; i++) {
            trace_msg(2,"%.2X:", inp_buff[i]);
        }
		trace_msg(2,"\r\n");
        
        trace_msg(2,"TXR: ");
        for ( i = 0; i < tr.len; i++) {
            trace_msg(2,"%.2X:", rx_buff[i]);
        }
        trace_msg(2,"\r\n");        
        //cleanup_and_send(rx_buff, len);
        //check_and_buffer(rx_buff, tr.len);
        int j = 0;
        if (rx_buff[0] != MESSAGE_ESCAPE){ //the remote buff was not empty. Discard everything until sync
            for (j=0;j<tr.len;j++){
                if (rx_buff[j] == MESSAGE_SYNC)
                    break ;
            }
        }
        if (j>0) j++; //skip the sync since it marked the end of the lost message
        trace_msg(2,"j = %i \n", j);

        for ( i = j; i < tr.len; i++) {
            char rbyte = rx_buff[i];
            if (rbyte != MESSAGE_ESCAPE){ 
                //if this is the first non-empty byte outside of a message, it must be the beginning of a message
                if (spi_read_buff.this_message_len == 0){
                    spi_read_buff.this_message_len = rbyte ; //+1 because we need to add also the length byte to the buff
                    trace_msg(2,"Msg begin %i bytes\n", rbyte);

                }
            }
            if (spi_read_buff.this_message_len > 0){
                spi_read_buff.data[spi_read_buff.len++] = rbyte;
                spi_read_buff.this_message_len --;
            }
        }
        if (spi_read_buff.this_message_len > 0){
            //there are still bytes to read
            trace_msg(2,"Kick transfer for rest of %i bytes\n", spi_read_buff.this_message_len);
            kick_mcu_data_transfer(thissq);
        }        
    }
    free(rx_buff);
    return ret;
}

/*
int spi_write(struct serialqueue *sq,  char* inp_buff, int buff_len, int is_retransmit)
{
	int ret = 0;
    
    char* rx_buff = malloc(buff_len+1); //when we write len bytes, we can receive len-1
    
    //locate the MESSAGE_SYNC in the message, 
    //needed because the next received byte contains the length of the remote transmit buffer
    int sync_pos = -1;
    char* pos_first_sync =  memchr (inp_buff, MESSAGE_SYNC, buff_len);
    if (pos_first_sync!=NULL){
        sync_pos = pos_first_sync-inp_buff+1;
        trace_msg(3, "syn found at position %d.\n", sync_pos);
    }
    
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)inp_buff,
		.rx_buf = (unsigned long)rx_buff,
		.len = buff_len,
		.delay_usecs = SPI_DELAY,
		.speed_hz = spi_speed,
		.bits_per_word = SPI_BITS,
	};
    
    if (sync_pos==buff_len){ //send one more byte, so that we receive the remote buff len
        tr.len += 1;
        //the ioctl will of course read  over the end of the inp_puff, but one byte is not a big deal :) :) 
    }
	
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (sync_pos >=0){
        trace_msg(3, "Remote blen is %i\n",rx_buff[sync_pos]);
    }
    
    int i = 0;
	if (ret < 1) {
        trace_msg(0,"can't send spi message\n");
        ret = -1;
    }else{

        trace_msg(2,"TX%i ", tr.len);
        
        for ( i = 0; i < tr.len; i++) {
            printf("%.2X:", inp_buff[i]);
        }
		printf("\r\n");
        
        printf("TXR: ");
        for ( i = 0; i < tr.len; i++) {
            printf("%.2X:", rx_buff[i]);
        }
        printf("\r\n");
                
        //cleanup_and_send(rx_buff, len);
    }
    free(rx_buff);
    return ret;
}
*/
struct serialqueue * __visible
spiqueue_alloc(char* spi_device, int write_only, uint32_t speed)
{
    thissq = serialqueue_alloc(setup_spi_comm(spi_device, speed), write_only);   
	return thissq;
}