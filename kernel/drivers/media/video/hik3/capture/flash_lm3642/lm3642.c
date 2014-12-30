#include <linux/mux.h>
#include <linux/kernel.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include "../isp/k3_ispv1_io.h"
#include "../isp/sensor_common.h"
#include <mach/lm3642.h>
#include <linux/delay.h>
#include <linux/time.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#include "../isp/cam_log.h"
#define FLASH_CHIP_ID_ADDR 0x00
#define FLASH_CHIP_ID_MASK 0x07
#define FLASH_CHIP_ID 0x00

#define FLASH_TORCH_LEVEL_0 0
#define FLASH_TORCH_LEVEL_1 1
#define FLASH_TORCH_LEVEL_2 2
#define FLASH_TORCH_LEVEL_3 3

static struct device_attribute lm3642_led;

static camera_flashlight lm3642_intf;
struct i2c_client *lm3642_client;

static unsigned int brightness_level = 0;
static int strobe0_pin;

static int lm3642_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lm3642_remove(struct i2c_client *client);
static int test_flash(void);
static void set_strobe0(u32 value);

static void dump_reg(void);
static int lm3642_turn_on(work_mode mode, flash_lum_level lum);
static int lm3642_turn_off(void);
static int lm3642_camera_mode_flag = 0;
static int lm3642_init_flag = 0;
static int lm3642_read_reg8(u8 reg, u8 *val);

int lm3642_init(void);
void lm3642_exit(void);

static const struct i2c_device_id lm3642_id[] = {
	{K3_FLASH_LM3642_NAME, 0},
	{}
};

static struct i2c_driver lm3642_driver = {
	.driver = {
		   .name = K3_FLASH_LM3642_NAME,
		   },
	.probe = lm3642_probe,
	.remove = lm3642_remove,
	.id_table = lm3642_id,
};


/*
 **************************************************************************
 * FunctionName: lm3642_i2c_read;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_i2c_read(char *rxData, int length)
{
	int ret = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = lm3642_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	ret = i2c_transfer(lm3642_client->adapter, msgs, 1);
	if (ret < 0)
		print_error("%s: transfer error %d\n", __func__, ret);

	return ret;
}

/*
 **************************************************************************
 * FunctionName: lm3642_i2c_write;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_i2c_write(char *txData, int length)
{
	int ret = 0;
	struct i2c_msg msg[] = {
		{
		 .addr = lm3642_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};
	ret = i2c_transfer(lm3642_client->adapter, msg, 1);
	if (ret < 0)
		print_error("%s: transfer error %d\n", __func__, ret);

	return ret;
}

/*
 **************************************************************************
 * FunctionName: lm3642_probe;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	struct lm3642_platform_data *pdata = NULL;
	int ret = 0;
	int val = 0;
	print_debug("Enter %s\n", __FUNCTION__);

	adapter = client->adapter;
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA
				     | I2C_FUNC_SMBUS_WRITE_BYTE)) {
		return -EIO;
	}

	pdata = client->dev.platform_data;
	if (!pdata) {
		print_error("platform data is null");
		return -ENOMEM;
	}

	lm3642_client = client;

	ret = lm3642_read_reg8(FLASH_CHIP_ID_ADDR, &val);
	if ( ( ret != 0 ) && ( FLASH_CHIP_ID == (val & FLASH_CHIP_ID_MASK) ) )
	{
		printk("lm3642 read chip id ok!\n");
	}
	else
	{
		printk("lm3642 read chip id error!\n");
		
		return -ENODEV;
	}
	/*remove func call "lm3642_turn_off" to "lm3642_init" to avoid warning of gpio-unrequest*/
	register_camera_flash(&lm3642_intf);

	if (device_create_file(&client->dev, &lm3642_led))
    {
	     print_error( "%s:Unable to create interface\n", __func__, &client->dev);
	     return -ENOMEM;
    }

	strobe0_pin = pdata->strobe0;

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
        set_hw_dev_flag(DEV_I2C_TPS);
#endif

	return 0;
}

/*
 **************************************************************************
 * FunctionName: lm3642_remove;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_remove(struct i2c_client *client)
{
	print_debug("Enter %s", __FUNCTION__);

	lm3642_client->adapter = NULL;
	return 0;
}

/*
 **************************************************************************
 * FunctionName: lm3642_read_reg8;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_read_reg8(u8 reg, u8 *val)
{
	int ret;
	u8 buf[1];
	print_debug("Enter Function:%s", __FUNCTION__);

	buf[0] = reg;
	ret = lm3642_i2c_write(buf, 1);
	if (ret < 0) {
		print_error("lm3642 read reg error(%d), reg=%x", ret, reg);
	}

	ret = lm3642_i2c_read(val, 1);
	if (ret < 0) {
		print_error("lm3642 read reg error(%d), reg=%x", ret, reg);
	}

	return ret;

}

/*
 **************************************************************************
 * FunctionName: lm3642_write_reg8;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_write_reg8(u8 reg, u8 val)
{
	u8 buf[2];
	print_debug("Enter Function:%s", __FUNCTION__);

	buf[0] = reg;
	buf[1] = val;
	return lm3642_i2c_write(buf, 2);

}

int lm3642_led_torch_mode_on()
{
	int val,ret = 0;
	
	print_debug("%s Enter.\n", __func__);

	//lm3642_init();

	if (brightness_level > FLASH_TORCH_LEVEL_3)
	{
		brightness_level= FLASH_TORCH_LEVEL_3;
	}

	val = (((brightness_level - 1) << 4) & 0xf0);

	print_debug("Torch value is %d\n", val);
		
	ret = lm3642_write_reg8(0x09, val);	
	if(ret < 0)
	{
            print_error("Set torch light level error,write reg2valu=%x",val);
            return -1;
	}
	
	ret = lm3642_write_reg8(0x0A, 0x02);	
	if(ret < 0)
	{
            print_error("Set torch mode error");
            return -1;
	}

        return 0;
}

static lm3642_led_torch_mode_off()
{
	print_debug("%s Enter.\n", __func__);

	if (0 == lm3642_camera_mode_flag)
	{
		lm3642_write_reg8(0x0A, 0x00);
	}

	//lm3642_exit();

	return 0;
}
static ssize_t lm3642_led_torch_mode_get_brightness(struct device *dev, struct device_attribute *attr,char *buf)
{
        int ret;
        sprintf(buf,"brightness_level= %d\n",brightness_level);
        ret = strlen(buf)+1;
        return ret;
}


static ssize_t lm3642_led_torch_mode_set_brightness(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val,ret;
    static char status = '0';

	print_debug("Enter [%s].\n", __func__);

    if(status != buf[0])
    {
        status = buf[0];
    }
    else
    {
        return count;
    }

	switch (buf[0])
	{
		case '0':
		{
			ret = lm3642_led_torch_mode_off();
			
			if(ret != 0)
			{
				print_error("lm3642_led_torch_mode_off error");
				return -1;
			}

			brightness_level = FLASH_TORCH_LEVEL_0;
		}
		break;

		case '1':
			brightness_level = FLASH_TORCH_LEVEL_1;
			break;

		case '2':
			brightness_level = FLASH_TORCH_LEVEL_2;
			break;
			
		default:
			brightness_level = FLASH_TORCH_LEVEL_3;
			break;
	}

	if ( brightness_level > FLASH_TORCH_LEVEL_0 )
	{
		ret = lm3642_led_torch_mode_on();
		
		if(ret != 0)
		{
			print_error("lm3642_led_torch_mode_on error");
			return -1;
		}
	}

    return count;
}

static struct device_attribute lm3642_led=
    __ATTR(lm3642_led_lightness, 0664, lm3642_led_torch_mode_get_brightness,
                        lm3642_led_torch_mode_set_brightness);

/*
 **************************************************************************
 * FunctionName: lm3642_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int lm3642_init(void)
{
	int ret;

	print_debug("[%s]\n", __func__);

	if (lm3642_init_flag)
		return;


	ret = gpio_request(strobe0_pin, NULL);
	if (ret) {
		print_error("failed to request strobe0 pin of flash ic");
		return -EIO;
	}

	/* init flash status,set enable reg as 0x00, pull down strobe pin */
	lm3642_turn_off();
	
	lm3642_init_flag = 1;
	return 0;
}

/*
 **************************************************************************
 * FunctionName: lm3642_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
void lm3642_exit(void)
{
	int ret;
	print_debug("enter %s", __FUNCTION__);

    /* flash should be turned off when exit */
    printk("lm3642_exit: turn off flash\n");
    lm3642_turn_off();

	gpio_free(strobe0_pin);

	lm3642_camera_mode_flag = 0;
	lm3642_init_flag = 0;
	return;
}

/*
 **************************************************************************
 * FunctionName: dump_reg;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void dump_reg(void)
{
/*	u8 val;
	int i;
	print_debug("enter %s", __FUNCTION__);

	for (i = 0; i < 8; i++) {
		lm3642_read_reg8(i, &val);
		print_info("read reg 0x%x = 0x%x", i, val);
		msleep(100);
	}
	*/
}

/*
 **************************************************************************
 * FunctionName: set_strobe0;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void set_strobe0(u32 value)
{
	if (value == 0)
		gpio_direction_output(strobe0_pin, 0);
	else
		gpio_direction_output(strobe0_pin, 1);

	/* for debug
	 *
	 * value = gpio_get_value(strobe0_pin);
	 * print_debug("===gpio[%d] value:%d", strobe0_pin, value);
	 */

}


/*
 **************************************************************************
 * FunctionName: test_flash;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int test_flash(void)
{
	print_debug("enter %s", __FUNCTION__);

	lm3642_turn_on(TORCH_MODE, 0);
	msleep(4000);
	dump_reg();
	lm3642_turn_off();
	msleep(1000);
	dump_reg();

	lm3642_turn_on(FLASH_MODE, 1);
	msleep(1000);
	dump_reg();
	lm3642_turn_off();
	dump_reg();

	return 0;
}

/*
 **************************************************************************
 * FunctionName: lm3642_reset;
 * Description : software reset;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int lm3642_reset(void)
{
	print_error("enter %s", __FUNCTION__);

	//gpio_direction_output(reset_pin, 0);

	return 0;
}

/*
 **************************************************************************
 * FunctionName: lm3642_turn_on;
 * Description : turn on flashlight;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_turn_on(work_mode mode, flash_lum_level lum)
{
	u8 val;
	print_debug("enter %s", __FUNCTION__);

	if (mode == FLASH_MODE) 
	{
		set_strobe0(0);
		
		if ( ( lum >= LUM_LEVEL0) && ( lum < LUM_LEVEL_MAX ) )
		{
			print_debug("start FLASH_MODE\n");
			
			lm3642_write_reg8(0x09, lum);
			lm3642_write_reg8(0x08, 0x07);
			lm3642_write_reg8(0x0A, 0x63);
			
			set_strobe0(1);
		}
		else
		{
			print_error("Unsupport lum_level:%d", lum);
			return -1;
		}
		
	} 
	else if (mode == TORCH_MODE) 
	{
		print_debug("start TORCH_MODE\n");

		if (lum > 2)
			lum = 2;

		val = ((lum << 4) & 0xf0);
		print_debug("Torch value is %d\n", val);
		lm3642_write_reg8(0x09, val);
		lm3642_write_reg8(0x0A, 0x02);
	}

	lm3642_camera_mode_flag = 1;

	return 0;
}



/*
 **************************************************************************
 * FunctionName: lm3642_turn_off;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int lm3642_turn_off(void)
{
	u8 val;
	u32 value;
	print_debug("enter %s", __FUNCTION__);

	lm3642_camera_mode_flag = 0;

	lm3642_write_reg8(0x0A, 0x00);
	set_strobe0(0);

	return 0;
}

/*
 **************************************************************************
 * FunctionName: lm3642_set_default;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void lm3642_set_default(void)
{
	print_debug("enter %s", __FUNCTION__);

	lm3642_intf.init		= lm3642_init;
	lm3642_intf.exit		= lm3642_exit;
	lm3642_intf.reset		= lm3642_reset;
	lm3642_intf.turn_on		= lm3642_turn_on;
	lm3642_intf.turn_off		= lm3642_turn_off;
	lm3642_intf.test		= test_flash;
	lm3642_intf.type		= LED_FLASH;
}

/*
 **************************************************************************
 * FunctionName: lm3642_module_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int __init lm3642_module_init(void)
{
	int ret;
	print_debug("enter %s", __FUNCTION__);
	ret = i2c_add_driver(&lm3642_driver);
	if (0 != ret) {
		print_error("[%s]Fail to add flash driver.\n", __func__);
		return -1;
	}

	lm3642_set_default();
	return 0;
}

/*
 **************************************************************************
 * FunctionName: lm3642_module_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void __exit lm3642_module_exit(void)
{
	print_debug("enter %s", __FUNCTION__);
    
	i2c_del_driver(&lm3642_driver);

	unregister_camera_flash(&lm3642_intf);
	return;
}


module_init(lm3642_module_init);
module_exit(lm3642_module_exit);
MODULE_AUTHOR("z00166742");
MODULE_DESCRIPTION("lm3642 Flash Driver");
MODULE_LICENSE("GPL");

