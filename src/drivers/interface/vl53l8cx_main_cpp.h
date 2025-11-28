#ifndef VL53L8CX_MAIN_H_
#define VL53L8CX_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Optional host/main entry. Implement in host-only file as:
   int vl53l8cx_main(void);
   If not linked, weak symbol resolution or NULL-checking will avoid calling. */
int vl53l8cx_main(void);

#ifdef __cplusplus
}
#endif

#endif // VL53L8CX_MAIN_H_