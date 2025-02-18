#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

int gpio_init_pin(unsigned int pin, bool is_output, const char *label);
void gpio_cleanup(void);
int gpio_set_value_safe(unsigned int pin, int value);
int gpio_get_value_safe(unsigned int pin);

#endif /* GPIO_CONTROL_H */
