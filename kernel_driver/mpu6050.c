#define pr_fmt(fmt) "%s %s: " fmt, KBUILD_MODNAME, __func__
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include "mpu6050.h"

#define MPU_NAME "mpu6050"
#define MEM_SIZE 1024
#define MPU_ADDR 0x68
#define WHO_AM_I_ADDR 0x75
#define USER_CTRL 0x6A          // User control register
#define PWR_MGMT_ADDR 0x6B      // Power Management Register
#define ACCEL_CONFIG_ADDR 0x1C  // Accelerometer Configuration (AFSEL)
#define TEMP_ADDR 0x41          // Temperature sensor address
#define ACC_XOUT0 0x3B          // First register for Accelerometer X
#define FIFO_R_W 0x74           // FIFO buffer
#define FIFO_COUNT_H 0x72       // FIFO Count Register 15:8 (0x73 for 7:0)
#define FIFO_EN 0x23            // Which sensor measurements are loaded
                                // into the FIFO buffer.
#define SMPRT_DIV 0x19          // divider from the gyroscope output rate
                                // used to generate the Sample Rate

const uint32_t sensitivity_afssel[4] = {16384, 8192, 4096, 2048};

static int I2C_BUS = 1;
static struct i2c_adapter *mpu_adapter;
static struct i2c_client *mpu_client;
dev_t devNr = 0;
static struct class *dev_class;
static struct cdev mpu_cdev;
struct kobject *kobj_ref;
unsigned char *kernel_buffer;

static mpu6050 mpu_info;
struct xyz_data acc_read;
unsigned int fifo_count;

ssize_t sysfs_acc_show(struct kobject *kobj, struct kobj_attribute *attr,
                       char *buf);
ssize_t sysfs_fifo_count_read(struct kobject *kobj, struct kobj_attribute *attr,
                              char *buf);

struct kobj_attribute mpu_acc_attr = __ATTR(xyz_data, 0660, sysfs_acc_show,
                                            NULL);
struct kobj_attribute fifo_count_attr = __ATTR(fifo_count, 0660,
                                               sysfs_fifo_count_read, NULL);

static struct attribute *dev_attrs[] = {
  &mpu_acc_attr.attr,
  &fifo_count_attr.attr,
  NULL,
};

/**
 * MPU_Write_Reg() - Writes to a register.
 * @reg: Register that will be written.
 * @value: Value that will be written to the register.
 *
 * Return: Number of bytes written.
 */
static int MPU_Write_Reg(unsigned char reg, unsigned int value) {
  unsigned char send_buffer[2] = {reg, value};
  int ret;

  ret = i2c_master_send(mpu_client, send_buffer, 2);
  if (ret != 2) {
    pr_err("Failed writing register");
  }
  return ret;
}

/**
 * MPU_Read_Reg() - Read a single register.
 * @reg: Register that will be read.
 *
 * Return: bytes read.
 */
static int MPU_Read_Reg(unsigned char reg, unsigned char *rec_buf) {
  struct i2c_msg msg[2];
  int ret;

  msg[0].addr = mpu_client->addr;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = &reg;
  msg[1].addr = mpu_client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len = 1;
  msg[1].buf = rec_buf;

  ret = i2c_transfer(mpu_client->adapter, msg, 2);
  if (ret < 0) {
    pr_err("Erro reading register 0x%X", reg);
  }
  return ret;
}

/**
 * MPU_Burst_Read() - Read multiple registers in sequence.
 * @reg: First register of the reading sequence.
 * @length: Number of registers to be read.
 * @rec_buffer: Pointer to a buffer that will store the read data.
 *
 * Return: bytes sent and received
 */
static int MPU_Burst_Read(unsigned char start_reg, unsigned int length,
                          unsigned char *rec_buffer) {
  int ret;
  struct i2c_msg msg[2];

  msg[0].addr = mpu_client->addr;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  msg[1].addr = mpu_client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len = length;
  msg[1].buf = rec_buffer;
  ret = i2c_transfer(mpu_client->adapter, msg, 2);
  if (ret < 0) {
    pr_err("Erro burst reading register 0x%X\n", start_reg);
  }
  return ret;
}

void read_accelerometer_axis(struct xyz_data *acc) {
  unsigned char test_buf[6];
  MPU_Burst_Read(ACC_XOUT0, 6, test_buf);
  acc->x = (test_buf[0] << 8) + test_buf[1];
  acc->y = (test_buf[2] << 8) + test_buf[3];
  acc->z = (test_buf[4] << 8) + test_buf[5];
}

void read_fifo_count(int *count) {
  unsigned char test_buf[2];
  MPU_Burst_Read(FIFO_COUNT_H, 2, test_buf);
  *count = (test_buf[0] << 8) + test_buf[1];
}

static long mpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg) { //NOLINT
  int count;
  switch (cmd) {
  case READ_ACCELEROMETER:
    read_fifo_count(&count);
    if (count > 10) {
      read_accelerometer_axis(&acc_read);
      if (copy_to_user((struct xyz_data*)arg, &acc_read,
                       sizeof(xyz_data)) != 0) {
        pr_err("Failed READ_ACCELEROMETER");
      }
    } else {
      pr_err("Not enough samples in FIFO: %d", count);
    }
    break;
  case MPU_INFO:
    if (copy_to_user((struct mpu6050*)arg, &mpu_info, sizeof(mpu6050)) != 0) {
      pr_err("Failed MPU_INFO");
      break;
    } else {
      break;
    }
  default:
    pr_info("IOCTL command defaulted");
    break;
  }
  return 0;
}

ssize_t sysfs_fifo_count_read(struct kobject *kobj, struct kobj_attribute *attr,
                              char *buf) {
  unsigned char test_buf[2];
  char outbuf[10];
  MPU_Burst_Read(FIFO_COUNT_H, 2, test_buf);
  fifo_count = (test_buf[0] << 8) + test_buf[1];
  snprintf(outbuf, sizeof(outbuf), "0x%X", fifo_count);
  return snprintf(buf, sizeof(outbuf), "%s", outbuf);
}

ssize_t sysfs_acc_show(struct kobject *kobj, struct kobj_attribute *attr,
                       char *buf) {
  unsigned char output_data[20];
  read_accelerometer_axis(&acc_read);
  snprintf(output_data, sizeof(output_data), "%d %d %d", acc_read.x,
           acc_read.y, acc_read.z);
  return snprintf(buf, sizeof(output_data), "%s", output_data);
}

static struct i2c_board_info mpu_board_info = {
  I2C_BOARD_INFO(MPU_NAME, MPU_ADDR),
};

static struct i2c_device_id mpu_id[] = {
  {MPU_NAME, 0}, {}
};
MODULE_DEVICE_TABLE(i2c, mpu_id);

static int mpu_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  unsigned char who_am_i;
  uint8_t AFS_SEL = 0x01;
  pr_info("Initializing driver");
  MPU_Read_Reg(WHO_AM_I_ADDR, &who_am_i);
  if (who_am_i != MPU_ADDR) {
    pr_err("Bad device address: 0x%X", who_am_i);
  } else {
    pr_info("Found device on: 0x%X", who_am_i);
  }

  MPU_Write_Reg(PWR_MGMT_ADDR, 0x01);
  mpu_info.sensitivity = sensitivity_afssel[AFS_SEL];
  MPU_Write_Reg(USER_CTRL, 0x68);
  MPU_Write_Reg(ACCEL_CONFIG_ADDR, 0x08);
  MPU_Write_Reg(FIFO_EN, 0x08);
  MPU_Write_Reg(SMPRT_DIV, 0x4F);
  pr_info("Done probing");
  return 0;
}

static int mpu_i2c_remove(struct i2c_client *client) {
  pr_info("Removing\n");
  MPU_Write_Reg(PWR_MGMT_ADDR, 0x80);
  return 0;
}

static int mpu_open(struct inode *inode, struct file *file) {
  pr_info("Opened fd");
  return 0;
}
static int mpu_release(struct inode *inode, struct file *file) {
  pr_info("Closed fd");
  return 0;
}

static ssize_t mpu_read(struct file *filp, char __user *buf, size_t len,
                        loff_t *off) {
  int ret;
  unsigned char test_buf;
  ret = MPU_Read_Reg(0x75, &test_buf);
  buf[0] = test_buf;
  return ret;
}

static ssize_t mpu_write(struct file *filp, const char *buf, size_t len,
                         loff_t *off) {
  int ret;
  if (len > 2) {
    pr_err("Too many fields written: only two permitted (reg, value)");
    return -1;
  }
  ret = MPU_Write_Reg(buf[0], buf[1]);
  return ret;
}

static struct i2c_driver mpu_driver = {
  .driver = {
    .name = MPU_NAME,
    .owner = THIS_MODULE,
  },
  .probe = mpu_probe,
  .remove = mpu_i2c_remove,
  .id_table = mpu_id,
};

static struct file_operations mpu_fops = {
  .owner = THIS_MODULE,
  .open = mpu_open,
  .read = mpu_read,
  .write = mpu_write,
  .release = mpu_release,
  .unlocked_ioctl = mpu_ioctl,
};

static int __init mpu_init(void) {
  if (alloc_chrdev_region(&devNr, 0, 1, MPU_NAME) < 0) {
    pr_err("Failed to allocate chr dev number");
    return -1;
  }

  cdev_init(&mpu_cdev, &mpu_fops);
  if (cdev_add(&mpu_cdev, devNr, 1) < 0) {
    pr_err("Could not add cdev.");
    goto r_class;
  }

  dev_class = class_create(THIS_MODULE, "mpu_class");
  if (dev_class == NULL) {
    pr_err("Failed to create device class");
    goto r_class;
  }

  if (device_create(dev_class, NULL, devNr, NULL, MPU_NAME) < 0) {
    pr_err("Failed to create the device");
    goto r_device;
  }

  kobj_ref = kobject_create_and_add(MPU_NAME, fs_kobj);
  if (sysfs_create_file(kobj_ref, dev_attrs[0]) < 0) {
    pr_err("Failed to create kobject!");
    goto r_sysfs;
  }
  if (sysfs_create_file(kobj_ref, dev_attrs[1]) < 0) {
    pr_err("Failed to create kobject!");
    goto r_sysfs;
  }

  mpu_adapter = i2c_get_adapter(I2C_BUS);
  if (mpu_adapter != NULL) {
    mpu_client = i2c_new_client_device(mpu_adapter, &mpu_board_info);
    if (mpu_client != NULL) {
      i2c_add_driver(&mpu_driver);
    }
  }
  i2c_put_adapter(mpu_adapter);

  if ((kernel_buffer = kmalloc(MEM_SIZE, GFP_KERNEL)) == 0) {
    pr_err("Cannot allocate memory in kernel\n");
    goto r_device;
  }

  return 0;

r_sysfs:
  kobject_put(kobj_ref);
  sysfs_remove_file(fs_kobj, &mpu_acc_attr.attr);
r_device:
  class_destroy(dev_class);
r_class:
  unregister_chrdev_region(devNr, 1);
  return -1;
}

static void __exit mpu_exit(void) {
  kfree(kernel_buffer);
  kobject_put(kobj_ref);
  sysfs_remove_file(kernel_kobj, dev_attrs[0]);
  sysfs_remove_file(kernel_kobj, dev_attrs[1]);
  device_destroy(dev_class, devNr);
  class_destroy(dev_class);
  unregister_chrdev_region(devNr, 1);
  i2c_unregister_device(mpu_client);
  i2c_del_driver(&mpu_driver);
  pr_info("Driver removed\n");
}

module_init(mpu_init);
module_exit(mpu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filipe do Ó Cavalcanti");
MODULE_DESCRIPTION("MPU6050 Kernel Module");
MODULE_VERSION("0.1.0");
