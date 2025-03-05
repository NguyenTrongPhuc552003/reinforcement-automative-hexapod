#ifndef HEXAPOD_COMMON_H
#define HEXAPOD_COMMON_H

#include <stdint.h>
#include "protocol.h"
#include "hexerror.h"

/* Validation functions */
int hexapod_validate_leg_num(uint8_t leg_num);
int hexapod_validate_angles(const struct leg_position *position);
void hexapod_init_leg_position(struct leg_position *position);

/* Device communication */
int hexapod_open_device(void);
void hexapod_close_device(int fd);
int hexapod_send_command(int fd, unsigned long cmd, void *data);

/* Calibration */
int hexapod_set_calibration(int fd, uint8_t leg_num,
                            int16_t hip_offset, int16_t knee_offset,
                            int16_t ankle_offset);

/* Version information */
void hexapod_print_version(void);

#endif /* HEXAPOD_COMMON_H */
