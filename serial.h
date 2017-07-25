#ifdef __cplusplus
extern "C"{
#endif

#ifndef _SERIAL_H_
#define _SERIAL_H_
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include <string.h>

int init_uart(char *dev);
int free_uart(int fd);
int putchar_t(int fd, uint8_t c);
int putbuffer_t(int fd, uint8_t *buff, int len);
int getchar_t(int fd, uint8_t *c);
int getbuffer_t(int fd, uint8_t *buff, int len);
#endif

#ifdef __cplusplus
}
#endif
