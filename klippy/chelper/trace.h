
#ifndef _TRACE_H
#define _TRACE_H

void set_trace_level(uint8_t  t);

void trace_msg(uint8_t level, const char* fmt, ...);
void trace_buffer(char* prefix, char* buff, int len);
void pabort(const char *s);

#endif