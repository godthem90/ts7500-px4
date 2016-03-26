#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define MW_ADR    0x18
#define MW_CONF   0x1a
#define MW_DAT1   0x1c
#define MW_DAT2   0x1e
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#ifndef TEMP_FAILURE_RETRY
# define TEMP_FAILURE_RETRY(expression) \
  (__extension__                                                              \
    ({ long int __result;                                                     \
       do __result = (long int) (expression);                                 \
       while (__result == -1L && errno == EINTR);                             \
       __result; }))
#endif


void sbus_poke16(unsigned int, unsigned short);
unsigned short sbus_peek16(unsigned int);

void winpoke16(unsigned int, unsigned short);
void xwinpoke16(unsigned int adr, unsigned short dat);
unsigned short winpeek16(unsigned int);
void winpoke32(unsigned int, unsigned int);
unsigned int winpeek32(unsigned int);
void winpoke8(unsigned int, unsigned char);
unsigned char winpeek8(unsigned int);

void sbus_peekstream16(int adr_reg, int dat_reg, int adr, unsigned char *buf, int n);
void sbus_pokestream16(int adr_reg, int dat_reg, int adr, unsigned char *buf, int n);

void sbuslock(void);
void sbusunlock(void);
void sbuspreempt(void); 


