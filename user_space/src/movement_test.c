// movement_test.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <unistd.h>
#include "hexapod_ioctl.h"

#define DEVICE_PATH "/dev/hexapod"

static int g_fd = -1;
static volatile int g_running = 1;

// Movement patterns
typedef struct {
    const char *name;
    int id;
    const char *description;
} movement_pattern_t;

static const movement_pattern_t PATTERNS[] = {
    {"tripod", 0, "Tripod gait - Stable, fast movement"},
    {"wave",   1, "Wave gait - Smooth, slower movement"},
    {"ripple", 2, "Ripple gait - Fluid, medium speed"}
};

static const int NUM_PATTERNS = sizeof(PATTERNS) / sizeof(PATTERNS[0]);

// Core functions
static void signal_handler(int signo) {
    g_running = 0;
}

static int initialize(void) {
    g_fd = open(DEVICE_PATH, O_RDWR);
    if (g_fd < 0) {
        perror("Failed to open device");
        return -1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    return 0;
}

static void cleanup(void) {
    if (g_fd >= 0) {
        // Stop any active movement
        struct movement_command cmd = {
            .pattern = 0,
            .speed = 0,
            .direction = 0
        };
        ioctl(g_fd, MOVEMENT_SET_PATTERN, &cmd);
        close(g_fd);
        g_fd = -1;
    }
}

static void list_patterns(void) {
    printf("\nAvailable movement patterns:\n");
    for (int i = 0; i < NUM_PATTERNS; i++) {
        printf("%d: %s - %s\n", i, PATTERNS[i].name, PATTERNS[i].description);
    }
    printf("\n");
}

static int find_pattern_by_name(const char *name) {
    for (int i = 0; i < NUM_PATTERNS; i++) {
        if (strcasecmp(PATTERNS[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static void print_usage(const char *program) {
    printf("Usage: %s <pattern> <speed> <direction>\n", program);
    printf("  pattern   - Movement pattern name or ID\n");
    printf("  speed     - Movement speed (1-100)\n");
    printf("  direction - Movement direction (-1:reverse, 0:stop, 1:forward)\n");
    printf("\nUse '%s --list' to see available patterns\n", program);
}

static int execute_movement(const char *pattern_str, int speed, int direction) {
    int pattern_id;

    // Try to parse as number first
    if (sscanf(pattern_str, "%d", &pattern_id) != 1) {
        // Try to find by name
        pattern_id = find_pattern_by_name(pattern_str);
    }

    if (pattern_id < 0 || pattern_id >= NUM_PATTERNS) {
        fprintf(stderr, "Invalid pattern: %s\n", pattern_str);
        return -1;
    }

    if (speed < 1 || speed > 100) {
        fprintf(stderr, "Speed must be between 1 and 100\n");
        return -1;
    }

    if (direction < -1 || direction > 1) {
        fprintf(stderr, "Direction must be -1, 0, or 1\n");
        return -1;
    }

    struct movement_command cmd = {
        .pattern = pattern_id,
        .speed = speed,
        .direction = direction
    };

    if (ioctl(g_fd, MOVEMENT_SET_PATTERN, &cmd) < 0) {
        perror("Failed to set movement pattern");
        return -1;
    }

    printf("Executing %s pattern (speed: %d, direction: %d)\n",
           PATTERNS[pattern_id].name, speed, direction);
    return 0;
}

int main(int argc, char *argv[]) {
    if (argc == 2 && strcmp(argv[1], "--list") == 0) {
        list_patterns();
        return 0;
    }

    if (argc != 4) {
        print_usage(argv[0]);
        return 1;
    }

    if (initialize() < 0) {
        return 1;
    }

    int result = execute_movement(argv[1], atoi(argv[2]), atoi(argv[3]));
    
    if (result == 0) {
        printf("Press Ctrl+C to stop...\n");
        while (g_running) {
            sleep(1);
        }
    }

    cleanup();
    return result != 0;
}