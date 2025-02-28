#ifndef _HEXAPOD_THREADS_H_
#define _HEXAPOD_THREADS_H_

#include <pthread.h>
#include "hexapod.h"

/* Thread control flags */
typedef struct
{
    volatile int running;
    pthread_mutex_t lock;
    pthread_cond_t cond;
} thread_control_t;

/* Thread data structures */
typedef struct
{
    thread_control_t *control;
    imu_data_t *imu_data;
    int update_rate_hz;
} imu_thread_args_t;

typedef struct
{
    thread_control_t *control;
    hexapod_state_t *state;
    int update_rate_hz;
} gait_thread_args_t;

/* Thread functions */
void *imu_reader_thread(void *arg);
void *gait_controller_thread(void *arg);
void *servo_controller_thread(void *arg);

/* Thread management */
int hexapod_start_threads(hexapod_state_t *state);
void hexapod_stop_threads(void);

#endif /* _HEXAPOD_THREADS_H_ */
