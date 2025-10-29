
/*
 * vl11_arena.c - Tiny static arena for VL53L8CX ULD allocations
 */
#include "vl11_arena.h"
#include <string.h>

#if defined(DEBUG_PRINT)
  #define AR_LOG(...) DEBUG_PRINT(__VA_ARGS__)
#else
  #define AR_LOG(...) do{}while(0)
#endif

static uint8_t g_vl11_storage[VL11_ARENA_STORAGE_MAX];
static size_t  g_vl11_cap   = 0;
static size_t  g_vl11_off   = 0;

void vl11_arena_reset(size_t bytes)
{
    if (bytes > VL11_ARENA_STORAGE_MAX) {
        bytes = VL11_ARENA_STORAGE_MAX;
    }
    g_vl11_cap = bytes;
    g_vl11_off = 0;
    AR_LOG("VL11_ARENA: reset cap=%u bytes (max=%u)\n",
           (unsigned)g_vl11_cap, (unsigned)VL11_ARENA_STORAGE_MAX);
}

static inline size_t align_up(size_t v, size_t a)
{
    return (v + (a - 1u)) & ~(a - 1u);
}

void * vl11_arena_alloc(size_t n)
{
    if (n == 0u) n = 1u;
    size_t off = align_up(g_vl11_off, 4u);
    size_t end = off + n;
    if (end > g_vl11_cap) {
        AR_LOG("VL11_ARENA: OOM alloc(%u) used=%u cap=%u\n",
               (unsigned)n, (unsigned)g_vl11_off, (unsigned)g_vl11_cap);
        return NULL;
    }
    void* p = &g_vl11_storage[off];
    g_vl11_off = end;
    return p;
}

void * vl11_arena_calloc(size_t count, size_t size)
{
    /* Multiply with overflow guard */
    if (size != 0u && count > ((size_t)-1) / size) {
        return NULL;
    }
    size_t n = count * size;
    void* p = vl11_arena_alloc(n);
    if (p) {
        memset(p, 0, n);
    }
    return p;
}

void vl11_arena_free(void *p)
{
    (void)p; /* no-op */
}

size_t vl11_arena_bytes_used(void)
{
    return g_vl11_off;
}
size_t vl11_arena_bytes_cap(void)
{
    return g_vl11_cap;
}
