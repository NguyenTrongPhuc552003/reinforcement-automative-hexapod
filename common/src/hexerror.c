#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "hexerror.h"

/**
 * Convert error code to string message
 */
const char *hexapod_strerror(int code)
{
    switch (code)
    {
    case HEXAPOD_SUCCESS:
        return "Success";
    case HEXAPOD_ERROR:
        return "Generic error";
    case HEXAPOD_EINVAL:
        return "Invalid argument";
    case HEXAPOD_ERANGE:
        return "Value out of range";
    case HEXAPOD_ENODEV:
        return "No such device";
    case HEXAPOD_EIO:
        return "I/O error";
    case HEXAPOD_ETIMEOUT:
        return "Operation timed out";
    case HEXAPOD_ENOTINIT:
        return "Not initialized";
    case HEXAPOD_EBUSY:
        return "Device or resource busy";
    case HEXAPOD_ENOMEM:
        return "Out of memory";
    default:
        return "Unknown error";
    }
}

/**
 * Print error message with description
 */
void hexapod_perror(const char *msg)
{
    if (msg)
    {
        fprintf(stderr, "%s: %s\n", msg, strerror(errno));
    }
    else
    {
        fprintf(stderr, "%s\n", strerror(errno));
    }
}
