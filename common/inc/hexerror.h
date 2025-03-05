#ifndef _HEXERROR_H_
#define _HEXERROR_H_

/* Success/Error codes */
#define HEXAPOD_SUCCESS 0   /* Operation successful */
#define HEXAPOD_ERROR -1    /* Generic error */
#define HEXAPOD_EINVAL -2   /* Invalid argument */
#define HEXAPOD_ERANGE -3   /* Value out of range */
#define HEXAPOD_ENODEV -4   /* No such device */
#define HEXAPOD_EIO -5      /* I/O error */
#define HEXAPOD_ETIMEOUT -6 /* Operation timed out */
#define HEXAPOD_ENOTINIT -7 /* Not initialized */
#define HEXAPOD_EBUSY -8    /* Device or resource busy */
#define HEXAPOD_ENOMEM -9   /* Out of memory */

/* Error handling functions */
const char *hexapod_strerror(int error_code);
void hexapod_perror(const char *msg);

#endif /* _HEXERROR_H_ */
