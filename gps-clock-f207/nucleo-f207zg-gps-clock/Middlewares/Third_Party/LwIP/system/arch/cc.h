/*
 * cc.h
 *
 *  lwIP's compiler and platform header for this board (arm-none-eabi-gcc,
 *  newlib). lwIP's own arch.h fills in the rest: little endian, the
 *  stdint types, printf formats from inttypes.h.
 */

#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* errno values from newlib's <errno.h> (lwIP's err.c maps to them) */
#define LWIP_ERRNO_STDINCLUDE 1

/* LWIP_RAND: DHCP's transaction IDs, and the like. ethernetif.c. */
uint32_t lwip_port_rand(void);
#define LWIP_RAND() lwip_port_rand()

/* No diagnostic output (LWIP_DEBUG is off). An assertion keeps its message
   and line for a debugger and stops: lwIP's asserts catch configuration
   and locking mistakes, not run-time conditions. ethernetif.c. */
void lwip_port_assert(const char *msg, int line, const char *file);
#define LWIP_PLATFORM_DIAG(x) do { } while (0)
#define LWIP_PLATFORM_ASSERT(x) lwip_port_assert((x), __LINE__, __FILE__)

#ifdef __cplusplus
}
#endif

#endif /* LWIP_ARCH_CC_H */
