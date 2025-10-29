
/*
 * vl11_arena.h - Tiny static arena for VL53L8CX ULD allocations
 *
 * Drop-in usage inside vl53l8cx_api.c:
 *   #include "vl11_arena.h"
 *   #define malloc  VL11_MALLOC
 *   #define calloc  VL11_CALLOC
 *   #define free    VL11_FREE
 *
 * Then, before calling vl53l8cx_init() in your deck task:
 *   vl11_arena_reset(12*1024);   // choose a size that fits your ULD
 *
 * This keeps ULD allocations out of the FreeRTOS heap.
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VL11_ARENA_STORAGE_MAX
#define VL11_ARENA_STORAGE_MAX (16u * 1024u)
#endif

void   vl11_arena_reset(size_t bytes);
void * vl11_arena_alloc(size_t n);
void * vl11_arena_calloc(size_t count, size_t size);
void   vl11_arena_free(void *p);
size_t vl11_arena_bytes_used(void);
size_t vl11_arena_bytes_cap(void);

#define VL11_MALLOC(n)          vl11_arena_alloc((n))
#define VL11_CALLOC(c, s)       vl11_arena_calloc((c), (s))
#define VL11_FREE(p)            vl11_arena_free((p))

#ifdef __cplusplus
} /* extern "C" */
#endif
