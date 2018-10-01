

/**
 *		For latency compensation purposes, the autopilot must store the
 *		previsous state.
 */

// Setting:
#define VISION_LATENCY_TIME_STEPS    2    ///< Time steps: note: must be at least 1


// Reset ...
extern void fifo_reset(void);

// Add a sample
extern void fifo_push(float x, float y, float z);

// Retrieve the oldest element
extern void fifo_pop(float *x, float *y, float *z);