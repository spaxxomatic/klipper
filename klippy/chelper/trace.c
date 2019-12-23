#include <stdarg.h>
#include <stddef.h> 
#include <stdint.h> 
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <linux/types.h>
#include "compiler.h" // __visible

uint8_t trace_level = 0;
#define TRC_ERROR 0
#define TRC_WARN 1
#define TRC_INFO 2
#define TRC_DEBUG 3

char* trc_prefix[4] = {"ERROR: ", "WARN: ", "INFO: ", "DEBUG: "};

void __visible set_trace_level(uint8_t  t){
    trace_level = t;
}


void trace_msg(uint8_t level, const char* fmt, ...){
    if (level <= trace_level){
        printf(trc_prefix[level]);
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        printf("\n");
        fflush(stdout);
        va_end(args);
    }
}

#ifdef DEBUG
void trace_buffer(char* prefix, char* buff, int len){
       
        printf(prefix);
        printf(": ");
        int i = 0;
        for ( i = 0; i < len; i++) {
            printf("%.2X:", buff[i]);
        }
		printf("\n");
        fflush(stdout);  
}
#else
void trace_buffer(char* prefix, char* buff, int len){}
#endif

void pabort(const char *s)
{
	fprintf(stderr, "FATAL: ");
    fprintf(stderr, s);
    fprintf(stderr, "\n");
	abort();
}