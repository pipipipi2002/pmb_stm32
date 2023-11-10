#include "retarget.h"
#include "uart.h"
#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <errno.h>
#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>


#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

/**
 * @brief Disable buffer to print out char immediately.
 * @details Needs to be called during setup after UART 
 *          setup and before any printf function call.
 * 
 */
void retarget_init(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
}

int _write(int fd, char* ptr, int len) {
    int i;
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART1, ptr[i]);
		return i;    
    }
    errno = EBADF;
    return -1;
}

int _read (int fd, char *ptr, int len) {
    if (fd == STDIN_FILENO) {
        PMB_uart1_readByte((uint8_t*) ptr);
        return 1;
    }
    errno = EBADF;
    return -1;
}

int _isatty(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

int _close(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

int _lseek(int fd, int ptr, int dir) {
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat* st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}


int _getpid(void) {
  return 1;
}

int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}