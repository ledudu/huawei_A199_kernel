#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mux.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>	   /* copy_from_user() */
#include <linux/debugfs.h>
#include <linux/platform_device.h>

#include "tpa2015.h"

#define LOG_TAG "TPA2015"

#define PRINT_INFO  0
#define PRINT_WARN  0
#define PRINT_DEBUG 0
#define PRINT_ERR   1

#if PRINT_INFO
#define logi(fmt, ...) printk("[" LOG_TAG "][I]" fmt "\n", ##__VA_ARGS__)
#else
#define logi(fmt, ...)
#endif

#if PRINT_WARN
#define logw(fmt, ...) printk("[" LOG_TAG "][W]" fmt "\n", ##__VA_ARGS__)
#else
#define logw(fmt, ...)
#endif

#if PRINT_DEBUG
#define logd(fmt, ...) printk("[" LOG_TAG "][D]" fmt "\n", ##__VA_ARGS__)
#else
#define logd(fmt, ...)
#endif

#if PRINT_ERR
#define loge(fmt, ...) printk("[" LOG_TAG "][E]" fmt "\n", ##__VA_ARGS__)
#else
#define loge(fmt, ...)
#endif

#define IOMUX_BLOCK_NAME "block_audio_spk"

static struct tpa2015_platform_data *pdata = NULL;
static struct mutex tpa2015_lock;
static struct iomux_block *tpa2015_iomux_block = NULL;
static struct block_config *tpa2015_block_config = NULL;
#ifdef CONFIG_DEBUG_FS
struct dentry *tpa2015_debugfs;
static int tpa2015_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
    unsigned int value = 0;
	char buf[32];
    memset(buf,0,32);
    value = gpio_get_value(pdata->gpio_tpa2015_en);
	sprintf(buf,"%d\n",value);
    return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}
static int tpa2015_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret=0;
    char buf[32];
    size_t buf_size;
    unsigned int value;
    memset(buf,0,32);
    buf_size = min(count, (sizeof(buf)-1));
    if (copy_from_user(buf, user_buf, buf_size))
	    return -EFAULT;
    kstrtoint(buf, 10, &value);
    if(value){
    	ret = blockmux_set(tpa2015_iomux_block, tpa2015_block_config, NORMAL);
        if (0 > ret) {
            loge("%s: set iomux to gpio normal error", __FUNCTION__);
            goto err_exit;
        }
        gpio_set_value(pdata->gpio_tpa2015_en, 1);
    }
    else{
        gpio_set_value(pdata->gpio_tpa2015_en, 0);
        ret = blockmux_set(tpa2015_iomux_block, tpa2015_block_config, LOWPOWER);
        if (0 > ret) {
            loge("%s: set iomux to gpio lowpower error", __FUNCTION__);
            goto err_exit;
        }
    }
err_exit:
    return buf_size;     
}
static int default_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;
	return 0;
}
static const struct file_operations tpa2015_list_fops = {
	.read =     tpa2015_read,
	.write =    tpa2015_write,
	.open =		default_open,
	.llseek =	default_llseek,
};
#endif

static int tpa2015_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int tpa2015_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long tpa2015_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned int value = 0;
    mutex_lock(&tpa2015_lock);
    switch (cmd) {
    case TPA2015_ENABLE:
        ret = blockmux_set(tpa2015_iomux_block, tpa2015_block_config, NORMAL);
        if (0 > ret) {
            loge("%s: set iomux to gpio normal error", __FUNCTION__);
            goto err_exit;
        }
		
        gpio_set_value(pdata->gpio_tpa2015_en, 1);
		value = gpio_get_value(pdata->gpio_tpa2015_en);
		loge("%s: enable pdata->gpio_tpa2015_en = %d", __FUNCTION__,value);
        break;
    case TPA2015_DISABLE:
        gpio_set_value(pdata->gpio_tpa2015_en, 0);
		value = gpio_get_value(pdata->gpio_tpa2015_en);
		loge("%s: disable pdata->gpio_tpa2015_en = %d", __FUNCTION__,value);
        ret = blockmux_set(tpa2015_iomux_block, tpa2015_block_config, LOWPOWER);
        if (0 > ret) {
            loge("%s: set iomux to gpio lowpower error", __FUNCTION__);
            goto err_exit;
        }
        break;
    default:
        loge("%s: invalid command %d", __FUNCTION__, _IOC_NR(cmd));
        ret = -EINVAL;
        break;
    }

err_exit:
    mutex_unlock(&tpa2015_lock);
    return ret;
}

static const struct file_operations tpa2015_fops = {
    .owner          = THIS_MODULE,
    .open           = tpa2015_open,
    .release        = tpa2015_release,
    .unlocked_ioctl = tpa2015_ioctl,
};

static struct miscdevice tpa2015_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = TPA2015_NAME,
    .fops   = &tpa2015_fops,
};

static int __devinit tpa2015_probe(struct platform_device *pdev)
{
    int ret = -ENODEV;

    logi("%s", __FUNCTION__);

    pdata = pdev->dev.platform_data;
    if (NULL == pdata) {
        loge("%s: platform data is NULL", __FUNCTION__);
        return -EINVAL;
    }

    tpa2015_iomux_block = iomux_get_block(IOMUX_BLOCK_NAME);
    if (!tpa2015_iomux_block) {
        loge("%s: get iomux block error", __FUNCTION__);
        return -ENODEV;
    }

    tpa2015_block_config = iomux_get_blockconfig(IOMUX_BLOCK_NAME);
    if (!tpa2015_block_config) {
        loge("%s: get block config error", __FUNCTION__);
        return -ENODEV;
    }

    /* request gpio */
    ret = gpio_request(pdata->gpio_tpa2015_en, TPA2015_NAME);
    if (0 > ret) {
        loge("%s: gpio request enable pin failed", __FUNCTION__);
        return ret;
    }

    /* set gpio output & set value low */
    ret = gpio_direction_output(pdata->gpio_tpa2015_en, 0);
    if (0 > ret) {
        loge("%s: set gpio direction failed", __FUNCTION__);
        gpio_free(pdata->gpio_tpa2015_en);
        return ret;
    }

    ret = misc_register(&tpa2015_device);
    if (ret) {
        loge("%s: tpa2015_device register failed", __FUNCTION__);
        gpio_free(pdata->gpio_tpa2015_en);
        return ret;
    }
#ifdef CONFIG_DEBUG_FS  
	if (!debugfs_create_file("tpa2015", 0644, NULL, NULL,
				 &tpa2015_list_fops))
		pr_warn("PA: Failed to create tpa2015 debugfs file\n");
#endif
    return ret;
}

static int __devexit tpa2015_remove(struct platform_device *pdev)
{
    logi("%s", __FUNCTION__);
    gpio_free(pdata->gpio_tpa2015_en);
    return 0;
}

static struct platform_driver tpa2015_driver = {
    .driver = {
        .name  = TPA2015_NAME,
        .owner = THIS_MODULE,
    },
    .probe  = tpa2015_probe,
    .remove = __devexit_p(tpa2015_remove),
};

static int __init tpa2015_init(void)
{
    logi("%s", __FUNCTION__);
    mutex_init(&tpa2015_lock);
    return platform_driver_register(&tpa2015_driver);
}

static void __exit tpa2015_exit(void)
{
    logi("%s", __FUNCTION__);
    platform_driver_unregister(&tpa2015_driver);
}

module_init(tpa2015_init);
module_exit(tpa2015_exit);

MODULE_DESCRIPTION("tpa2015 driver");
MODULE_LICENSE("GPL");
