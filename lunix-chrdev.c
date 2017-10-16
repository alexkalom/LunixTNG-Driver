/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Modified by: Alexandros Kalomoiros
 *
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
//#include <linux/irqflags.h>

#include "lunix-chrdev.h"
#include "lunix-lookup.h"
#include "lunix.h"
/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int
lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state) {
    struct lunix_sensor_struct *sensor;

    WARN_ON(!(sensor = state->sensor));

    // if buf_timestamp != sensor_lastupdate we have new data,
    // and thus we need to update.
    if (sensor->msr_data[BATT]->last_update != state->buf_timestamp)
        return 1;

    // otherwise return 0
    return 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state) {
    struct lunix_sensor_struct *sensor;
    uint32_t val;

    WARN_ON(!(sensor = state->sensor));

    debug("entering state update\n");

    unsigned long flags;
    spin_lock_irqsave(&sensor->lock, flags);

    // check if new data exist
    if (lunix_chrdev_state_needs_refresh(state)) {

        // Grab the raw data quickly, hold the spinlock for as little as
        // possible.

        state->buf_timestamp = sensor->msr_data[BATT]->last_update;
        val = sensor->msr_data[state->type]->values[0];

        spin_unlock_irqrestore(&sensor->lock, flags);

        // Now we can take our time to format them, holding only the private
        // state semaphore
        long res;

        // take actual value based on the type of measurment
        switch (state->type) {
        case BATT:
            res = lookup_voltage[val];
            break;
        case TEMP:
            res = lookup_temperature[val];
            break;
        case LIGHT:
            res = lookup_light[val];
        }

        // format data
        int i;
        if (res >= 0)
            state->buf_data[0] = '+';
        else {
            state->buf_data[0] = '-';
            res = res * -1;
        }
        for (i = 0; i < 3; i++) {
            state->buf_data[6 - i] = '0' + res % 10;
            res = res / 10;
        }
        state->buf_data[3] = '.';
        for (i = 0; i < 2; i++) {
            state->buf_data[2 - i] = '0' + res % 10;
            res = res / 10;
        }
        state->buf_lim = 7;
        state->buf_data[state->buf_lim++] = '\n';

        return 0;
    }

    spin_unlock_irqrestore(&sensor->lock, flags);

    debug("leaving state update\n");
    return -EAGAIN;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp) {
    /* Declarations */
    int ret;

    debug("entering\n");
    ret = -ENODEV;
    if ((ret = nonseekable_open(inode, filp)) < 0)
        goto out;

    /*
     * Associate this open file with the relevant sensor based on
     * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
     */
    unsigned int minor = iminor(inode);
    unsigned int sensor_idx = minor >> 3;
    unsigned int type = minor & 7;

    /* Allocate a new Lunix character device private state structure */
    struct lunix_chrdev_state_struct *private_state =
        kzalloc(sizeof(*private_state), GFP_KERNEL);
    if (!private_state) {
        printk(KERN_ERR "Failed to allocate memory for private state\n");
        ret = -ENOMEM;
        goto out;
    }

    private_state->type = type;
    private_state->sensor = &lunix_sensors[sensor_idx];
    private_state->buf_lim = 0;
    private_state->buf_timestamp =
        0; // every timestamp has a value greater than zero.
    sema_init(&private_state->lock, 1);

    filp->private_data = private_state;

out:
    debug("leaving, with ret = %d\n", ret);
    return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp) {
    /* On release we need to release back to
    kernel every resourse that it's not needed any more */

    kfree(filp->private_data);
    return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd,
                               unsigned long arg) {
    /* Why? */
    return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf,
                                 size_t cnt, loff_t *f_pos) {
    ssize_t ret;

    struct lunix_sensor_struct *sensor;
    struct lunix_chrdev_state_struct *state;

    state = filp->private_data;
    WARN_ON(!state);

    sensor = state->sensor;
    WARN_ON(!sensor);

    if (down_interruptible(&state->lock))
        return -ERESTARTSYS;
    /*
     * If the cached character device state needs to be
     * updated by actual sensor data (i.e. we need to report
     * on a "fresh" measurement, do so
     */

    // if there is no such device return EOF?

    // after reading the whole buffer, ask for new data.
    if (*f_pos >= state->buf_lim)
        *f_pos = 0;

    if (*f_pos == 0) {
        while (lunix_chrdev_state_update(state) == -EAGAIN) {
            up(&state->lock);
            if (wait_event_interruptible(
                    sensor->wq, lunix_chrdev_state_needs_refresh(state)))
                return -ERESTARTSYS;
            if (down_interruptible(&state->lock))
                return -ERESTARTSYS;
        }
    }

    if (*f_pos + cnt > state->buf_lim)
        cnt = state->buf_lim - *f_pos;

    if (copy_to_user(usrbuf, state->buf_data, cnt)) {
        ret = -EFAULT;
        goto out;
    }
    *f_pos += cnt;
    ret = cnt;

out:
    up(&state->lock);
    return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma) {
    return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = {
    .owner = THIS_MODULE,
    .open = lunix_chrdev_open,
    .release = lunix_chrdev_release,
    .read = lunix_chrdev_read,
    .unlocked_ioctl = lunix_chrdev_ioctl,
    .mmap = lunix_chrdev_mmap};

int lunix_chrdev_init(void) {
    /*
     * Register the character device with the kernel, asking for
     * a range of minor numbers (number of sensors * 8 measurements / sensor)
     * beginning with LINUX_CHRDEV_MAJOR:0
     */
    int ret;
    dev_t dev_no;
    unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

    debug("initializing character device\n");
    cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
    lunix_chrdev_cdev.owner = THIS_MODULE;

    dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);

    /* register_chrdev_region */
    ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");
    if (ret < 0) {
        debug("failed to register region, ret = %d\n", ret);
        goto out;
    }

    /* cdev_add */
    ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
    if (ret < 0) {
        debug("failed to add character device\n");
        goto out_with_chrdev_region;
    }
    debug("completed successfully\n");
    return 0;

out_with_chrdev_region:
    unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
    return ret;
}

void lunix_chrdev_destroy(void) {
    dev_t dev_no;
    unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

    debug("entering\n");
    dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
    cdev_del(&lunix_chrdev_cdev);
    unregister_chrdev_region(dev_no, lunix_minor_cnt);
    debug("leaving\n");
}
