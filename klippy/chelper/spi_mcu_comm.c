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
#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include "gpio.h"
#endif 

#include "serialqueue.h" //MESSAGE_SYNC , serialqueue_alloc, etc
#include "spi_mcu_comm.h"
#include "trace.h"

int spi_fd = 0;

//yes, global variable, absolute buggy in case we have more than one SPI interface
struct serialqueue* thissq = 0; 

//extern struct serialqueue * serialqueue_alloc(int serial_fd, int write_only);
//extern void handle_rx_data(struct serialqueue *sq, char* data, int len);

#ifdef USE_WIRINGPI
// Handler for pin interrupt
void handleSlaveIrq(void) {
    trace_msg(1,"Slave IRQ\n");
    //spi_read();
    if (thissq){ //in case we're getting an irq before being inistialized
        //schedule a data transmission        
        kick_mcu_data_transfer(thissq);
    }
}
#endif 

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

t_spi_read_buff spi_read_buff;

struct spi_ioc_transfer xfer[2];

char sync_signal = MESSAGE_SYNC;
/*
void check_mcu_data_pending(){
    if (getPinStatus(SPI_SLAVE_IRQ_GPIO_PIN) > 0){
        trace_msg(2,"Data read RQ !!\n");
        kick_mcu_data_transfer(thissq);
    }
}
*/
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

void cleanup_mcu_buffer(){
    int i, stop_receiving = 0;

    while (getPinStatus(SPI_SLAVE_IRQ_GPIO_PIN)){
        if (stop_receiving){
            pabort("Buffer is empty but signal pin still high. MCU comm is broken");
        }
        trace_msg(2, "Emptying MCU buffer\n");
        //while starting, the MCU might have data in the buffer. Read and discard data until buffer is empty
        int buff_len = 64;
        char* rx_buff = malloc(buff_len); //when we write len bytes, we can receive len-1
        memset(rx_buff, MESSAGE_ESCAPE, buff_len);
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)rx_buff,
            .rx_buf = (unsigned long)rx_buff,
            .len = buff_len,
            .delay_usecs = SPI_DELAY,
            .speed_hz = spi_speed,
            .bits_per_word = SPI_BITS,
        };
        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
        int j=0;
        for (i = 0; i < buff_len; i++){
            if (rx_buff[i] == MESSAGE_ESCAPE ){
                j++;
            }
        }
        if (j>4) { 
            // we received lots of escapes, this is for sure not a message any more
            // the buffer is empty, pin must go low very soon
            stop_receiving = 1;
        }
        free(rx_buff);
        usleep(10000);
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
    
    cleanup_mcu_buffer();
	
    return spi_fd;
}
/*
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
*/
//handler for slave data tranfer requests
#define MSG_ENDMARK  0xFF

void trace_buffer(char* msg, char* buff, int len){
    
        trace_msg(2, msg);
        trace_msg(2, ": ");
        int i = 0;
        for ( i = 0; i < len; i++) {
            trace_msg(2,"%.2X:", buff[i]);
        }
		trace_msg(2,"\r\n");
}

int _transfer_mcu_bytes(int expected_bytes){ //will return the number of bytes still left on the MCU side
    //We read one more byte than requested, to check if another message follows 
    //or it's a MESSAGE_ESCAPE, which signals an empty buffer
    int transmission_len = expected_bytes+1; 
    
    char* buff = malloc(transmission_len); 
    memset(buff, MESSAGE_ESCAPE, transmission_len); //we send only MESSAGE_ESCAPE's
    xfer[0].rx_buf = (unsigned long)buff;
	xfer[0].len = transmission_len;
    xfer[0].tx_buf = (unsigned long)buff;
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer);
	trace_msg(0,"Will read %i + 1  ", expected_bytes);    
	if (ret < 1) {
        pabort("can't read spi message");
    }else{
        ret = 0;
        trace_buffer("Transfer", buff, transmission_len);
        int i = 0;
        for ( i = 0; i < expected_bytes; i++) {
            spi_read_buff.data[spi_read_buff.len++] = buff[i]; //todo - replace with memcpy
        }
        if (buff[expected_bytes-1] != MESSAGE_SYNC){
            trace_msg(2," !!Internal err!! Message end is not SYN but %.2X \r\n", buff[expected_bytes-1]);
        }
        char last_byte = buff[expected_bytes];
        trace_msg(2,"And the last is %.2X \r\n", last_byte);        
        if ( last_byte != MESSAGE_ESCAPE){ //there must be another mesage awaiting on the MCU side
            spi_read_buff.data[spi_read_buff.len++] = last_byte;
            ret = last_byte-1; //the rest of the packet
        }
    }
    return ret;
}

#define MAX_CONSECUTIVE_TRANSFER 10 //if more than that, something is bad

int read_all_mcu_data(int expected_bytes){
    int cycle= 0;         
    while (cycle++ < MAX_CONSECUTIVE_TRANSFER){
        expected_bytes = _transfer_mcu_bytes(expected_bytes);
        if (expected_bytes == 0)
            break;
    }
    if (cycle == MAX_CONSECUTIVE_TRANSFER)
        pabort("Internal error. MAX_CONSECUTIVE_TRANSFER exceeded");
    if (getPinStatus(SPI_SLAVE_IRQ_GPIO_PIN) != 0) {
        trace_msg(0, "Internal error. MCU pin is high at end of read\n"); 
    }
    //check_and_buffer(buff, expected_bytes);
    spi_read_buff.this_message_len = 0;//done reading remote buffer
    return spi_read_buff.len;
}

int spi_read() {
	int ret = 0;
    // If the MCU data tranfer pin is still high, we have data to read
    if (getPinStatus(SPI_SLAVE_IRQ_GPIO_PIN) == 0) {
        return spi_read_buff.len ;
    }
    trace_msg(3,"rx-1 ! ");    
    int packet_len = 0;
    if (spi_read_buff.this_message_len > 0){
        trace_msg(0,"ERR: read while buffer not clean\n");
        return -1;
    }

    //We have been called by the timer that checks the MCU transfer requests (SQPT_READ_SPI)
    //We have to read two bytes, the first byte must be escape and the second the message length
    char buff[2] = {MESSAGE_ESCAPE, MESSAGE_ESCAPE}; //need to set this value so that the MCU does not confuse it with a data packet
    xfer[0].rx_buf = (unsigned long)&buff;
    xfer[0].len = 2;
    xfer[0].tx_buf = (unsigned long)&buff;
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (ret < 1) 
        pabort("can't read spi message");
    
    int expected_bytes = 0;
    if (buff[0] != MESSAGE_ESCAPE) {
        //trace_msg(0,"ERR: first byte is %.2X \n",  buff[0]);
        packet_len = buff[0];    
        spi_read_buff.data[spi_read_buff.len++] = buff[0];
        spi_read_buff.data[spi_read_buff.len++] = buff[1];
        expected_bytes = packet_len - 2;
    }else if  (buff[1] != MESSAGE_ESCAPE) {
        packet_len = buff[1];
        spi_read_buff.data[spi_read_buff.len++] = buff[1];    
        expected_bytes = packet_len - 1;
    }else{
        trace_msg(0,"ERR: No len in the first two bytes \n");
        return -1;
    }
    trace_msg(3, "Asked for len, got %i %i \n", buff[0], buff[1]);
    if (packet_len<MESSAGE_MIN || packet_len>MESSAGE_MAX ){
        trace_msg(0, "Internal error. Len is invalid: %i\n", packet_len); 
        spi_read_buff.this_message_len = 0;//
        return -1;
    }
    //spi_read_buff.data[spi_read_buff.len++] = packet_len;
    read_all_mcu_data(expected_bytes);
    return spi_read_buff.len;
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
    
    if (spi_read_buff.len != 0 || spi_read_buff.this_message_len != 0){
        trace_msg(0," !! Error!! spi_write called while message not processed \n");
    }

    int i = 0;
	if (ret < 1) {
        trace_msg(0,"can't send spi message\n");
        ret = -1;
    }else{
        trace_buffer("TX", inp_buff, buff_len);
        trace_buffer("RX", rx_buff, buff_len);        
        while(i < buff_len){            
            if (rx_buff[i] != MESSAGE_ESCAPE){ //Whatever we receive besides an escape must be the beginning of a data packet
                spi_read_buff.this_message_len = rx_buff[i]; //there is message data in the read buffer
                while(spi_read_buff.this_message_len>0) {
                    //transfer the whole message or at least as much as the buffer contains
                    spi_read_buff.data[spi_read_buff.len++] = rx_buff[i++];
                    if (i == buff_len) break;
                    spi_read_buff.this_message_len --;
                }
                trace_buffer("DCD ", spi_read_buff.data, spi_read_buff.len);
            }
            i++;
        };

        if (spi_read_buff.len > 0){
            //there was some payload in data received 
            if (spi_read_buff.this_message_len > 0){
                //there are still bytes to be read, the last packet was not complete
                trace_msg(2,"Kick transfer for rest of %i bytes\n", spi_read_buff.this_message_len);
                //kick_mcu_data_transfer(thissq);
                read_all_mcu_data(spi_read_buff.this_message_len-1);
                /*if (_transfer_mcu_bytes(spi_read_buff.this_message_len) != 0){ //will read until all pending messages are transferred
                    trace_msg(2,"Read over end of buff but still data left");
                }*/
                
            }else{
                // a full packet waits in the buffer. Will be picked up by the next call to read
                //kick_mcu_data_transfer(thissq);
            }
            //spi_read_buff.this_message_len = 0;//done reading remote buffer
        }   
        kick_mcu_data_transfer(thissq);  //make sure the read is not suspended
    }
    free(rx_buff);
    
    return ret;
}

struct serialqueue * __visible
spiqueue_alloc(char* spi_device, int write_only, uint32_t speed)
{
    thissq = serialqueue_alloc(setup_spi_comm(spi_device, speed), write_only);   
	return thissq;
}