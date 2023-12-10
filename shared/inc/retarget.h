#ifndef INC_RETARGET_H
#define INC_RETARGET_H

#include "common_defines.h"
#include <sys/stat.h>

bool retarget_setup(void);

int _write(int fd, char* ptr, int len);
int _read (int fd, char *ptr, int len);
int _isatty(int fd);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _fstat(int fd, struct stat* st);
int _getpid(void);
int _kill(int pid, int sig);

#endif // INC_RETARGET_H