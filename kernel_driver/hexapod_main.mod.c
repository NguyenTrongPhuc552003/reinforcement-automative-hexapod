#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xa0a7cae5, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x6bc3fbc0, __VMLINUX_SYMBOL_STR(__unregister_chrdev) },
	{ 0x71873fdc, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x12da5bb2, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xb08f9e48, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0x25f9c0a4, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0x47229b5c, __VMLINUX_SYMBOL_STR(gpio_request) },
	{ 0x53aa5f51, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x52c7f37c, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0xfc3c9dac, __VMLINUX_SYMBOL_STR(__register_chrdev) },
	{ 0x3dcf8362, __VMLINUX_SYMBOL_STR(filp_close) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
	{ 0x28cc25db, __VMLINUX_SYMBOL_STR(arm_copy_from_user) },
	{ 0xe4ca3b4f, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0xf4fa543b, __VMLINUX_SYMBOL_STR(arm_copy_to_user) },
	{ 0x7f23a6b3, __VMLINUX_SYMBOL_STR(mutex_trylock) },
	{ 0xe707d823, __VMLINUX_SYMBOL_STR(__aeabi_uidiv) },
	{ 0xfa2a45e, __VMLINUX_SYMBOL_STR(__memzero) },
	{ 0x5f754e5a, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x328a05f1, __VMLINUX_SYMBOL_STR(strncpy) },
	{ 0x2e7c4960, __VMLINUX_SYMBOL_STR(gpiod_direction_input) },
	{ 0xa08b248e, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0xba7fa7c1, __VMLINUX_SYMBOL_STR(gpiod_direction_output_raw) },
	{ 0x23f22844, __VMLINUX_SYMBOL_STR(i2c_unregister_device) },
	{ 0x68ba5a9, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xfe990052, __VMLINUX_SYMBOL_STR(gpio_free) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x9d669763, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x89df42aa, __VMLINUX_SYMBOL_STR(gpiod_set_raw_value) },
	{ 0x37af6c70, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x8f678b07, __VMLINUX_SYMBOL_STR(__stack_chk_guard) },
	{ 0x8d232dcf, __VMLINUX_SYMBOL_STR(gpiod_get_raw_value) },
	{ 0x1327b810, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x11915b78, __VMLINUX_SYMBOL_STR(i2c_new_device) },
	{ 0x9361fa47, __VMLINUX_SYMBOL_STR(filp_open) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "0DE758DDA0B9304A326CB53");
