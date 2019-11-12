#include <stdarg.h>
#include <stddef.h> 
#include <stdint.h> 
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <linux/types.h>

uint8_t trace_level = 0;
void set_trace_level(uint8_t  t){
    trace_level = t;
}


void trace_msg(uint8_t level, const char* fmt, ...){
    if (level <= trace_level){
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        fflush(stdout);
        va_end(args);        
    }
}

void pabort(const char *s)
{
	fprintf(stderr, "FATAL: ");
    fprintf(stderr, s);
    fprintf(stderr, "\n");
	abort();
}