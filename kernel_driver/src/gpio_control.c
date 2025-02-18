#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../include/gpio_control.h"

#define MAX_GPIO_PINS 18  // Adjust based on your hexapod needs

struct gpio_pin {
    unsigned int pin;
    bool is_active;
    char label[32];
};

static struct gpio_pin hexapod_gpios[MAX_GPIO_PINS];
static int num_gpios = 0;

int gpio_init_pin(unsigned int pin, bool is_output, const char *label) {
    if (num_gpios >= MAX_GPIO_PINS) {
        pr_err("Hexapod GPIO: Maximum number of GPIO pins reached\n");
        return -ENOMEM;
    }

    if (gpio_request(pin, label)) {
        pr_err("Hexapod GPIO: Failed to request GPIO %d\n", pin);
        return -EBUSY;
    }

    if (is_output) {
        if (gpio_direction_output(pin, 0)) {
            pr_err("Hexapod GPIO: Failed to set GPIO %d as output\n", pin);
            gpio_free(pin);
            return -EINVAL;
        }
    } else {
        if (gpio_direction_input(pin)) {
            pr_err("Hexapod GPIO: Failed to set GPIO %d as input\n", pin);
            gpio_free(pin);
            return -EINVAL;
        }
    }

    hexapod_gpios[num_gpios].pin = pin;
    hexapod_gpios[num_gpios].is_active = true;
    strncpy(hexapod_gpios[num_gpios].label, label, sizeof(hexapod_gpios[num_gpios].label) - 1);
    num_gpios++;

    return 0;
}

void gpio_cleanup(void) {
    int i;
    for (i = 0; i < num_gpios; i++) {
        if (hexapod_gpios[i].is_active) {
            gpio_free(hexapod_gpios[i].pin);
            hexapod_gpios[i].is_active = false;
        }
    }
    num_gpios = 0;
}

int gpio_set_value_safe(unsigned int pin, int value) {
    int i;
    for (i = 0; i < num_gpios; i++) {
        if (hexapod_gpios[i].pin == pin && hexapod_gpios[i].is_active) {
            gpio_set_value(pin, value);
            return 0;
        }
    }
    return -EINVAL;
}

int gpio_get_value_safe(unsigned int pin) {
    int i;
    for (i = 0; i < num_gpios; i++) {
        if (hexapod_gpios[i].pin == pin && hexapod_gpios[i].is_active) {
            return gpio_get_value(pin);
        }
    }
    return -EINVAL;
}
