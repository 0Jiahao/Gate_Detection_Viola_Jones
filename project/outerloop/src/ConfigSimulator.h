//
// Created by phil on 13/08/18.
//

#ifndef CONFIGSIMULATOR_H
#define CONFIGSIMULATOR_H

/**
 * VISION
 */


#define CAMERA_C_X 320
#define CAMERA_C_Y 240
#define GATE_LENGTH 2.6
#define GATE_HEIGHT -4.5
#define CAMERA_F 305
#define CAMERA__DISTORTION_K 1.0


#ifndef N_SAMPLES
#define N_SAMPLES 3000
#endif


#ifndef Y_MAX
#define Y_MAX 160
#endif


#ifndef Y_MIN
#define Y_MIN 0
//#define Y_MIN 61
#endif

#ifndef Cr_MAX
#define Cr_MAX 225
//#define Cr_MAX 255
#endif


#ifndef Cr_MIN
#define Cr_MIN 0
//#define Cr_MIN 0
#endif

#ifndef Cb_MAX
#define Cb_MAX 120
//#define Cb_MAX 113
#endif


#ifndef Cb_MIN
#define Cb_MIN 70
//#define Cb_MIN 87
#endif


#ifndef MIN_LENGTH
#define MIN_LENGTH 25
#endif

#ifndef MAX_HORIZONTAL_PIXELS
#define MAX_HORIZONTAL_PIXELS 3
#endif


#ifndef MIN_COLOR_FITNESS
#define MIN_COLOR_FITNESS 0.8
#endif

#ifndef MAX_COLOR_FITNESS_CORNER23
#define MAX_COLOR_FITNESS_CORNER23 0.2
#endif

#ifndef DEBUG_SNAKE_UP_DOWN
#define DEBUG_SNAKE_UP_DOWN 0
#endif

#ifndef DEBUG_CHECK_COLOR_BLOCK
#define DEBUG_CHECK_COLOR_BLOCK 0
#endif

#ifndef DEBUG_SNAKE_LEFT_RIGHT
#define DEBUG_SNAKE_LEFT_RIGHT 0
#endif

#ifndef WAIT_KEY_TIME
#define WAIT_KEY_TIME 0
#endif


#ifndef NUM_RANDOM_POINTS_IN_BLOCK
#define NUM_RANDOM_POINTS_IN_BLOCK 50
#endif


#ifndef THRESHOLD_COLOR_BLOCK
#define THRESHOLD_COLOR_BLOCK 0.3
#endif

/**
 *  FILTER
 */

#define STARTUP_TIME_FILTER_MS 5000
#define FILTER_INIT_X -16.0
#define FILTER_INIT_Y 0.0
#define FILTER_INIT_Z -4.5

/**
 * SCHEDULING
 */
//How fast do we run periodic tasks, filter + control:
#define F_PERIODIC_HZ 500.0

#define LOG_FRAMES false
#define LOG_VALUES true

#define F_LOG_HZ 100
#define F_LOG_FRAME_HZ 1

#define LOG_FRAME_H 240
#define LOG_FRAME_W 320

#define MAX_LOG 200000
#define MAX_LOG_FRAMES 1000

#endif //CONFIG_H
