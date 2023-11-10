#ifndef INC_RETARGET_H
#define INC_RETARGET_H

#include "common_defines.h"
#include <sys/stat.h>

void retarget_init(void);

int _write(int fd, char* ptr, int len);
int _read (int fd, char *ptr, int len);
int _isatty(int fd);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _fstat(int fd, struct stat* st);

#endif // INC_RETARGET_H