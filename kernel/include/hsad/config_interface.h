

#ifndef CONFIG_INTERFACE_H
#define CONFIG_INTERFACE_H

#include <linux/types.h>

extern bool product_type(char *pname);
extern bool get_spk_pa(char *spk_pa);
extern int get_touchscreen_type(void);
extern int get_usbphy_tune(void);
extern int get_board_type(void);
extern int get_sd_detect_type(void);
extern int get_sensor_type(void);
extern int get_apds_type(void);
extern int get_iomux_type(void);
extern int get_sensor_timing_type();
extern int  get_rgb_sensor_value(void);
extern int get_rgb_sensor_type(void);
extern int get_gpu_dcdc_supply(void);
extern int get_sensor_tp_type(void);

typedef enum _config_touchscreen_type {
	E_TOUCHSCREEN_TYPE_PLATFORM = 0,
	E_TOUCHSCREEN_TYPE_PHONE
} config_touchscreen_type;

/* add USBPHY config for G710C */
typedef enum _config_usbphy_tune {
	E_USBPHY_TUNE_PLATFORM = 0,
	E_USBPHY_TUNE_U9510,
	E_USBPHY_TUNE_G710C,
} config_usbphy_tune;

typedef enum _config_board_type {
	E_BOARD_TYPE_PLATFORM = 0,
	E_BOARD_TYPE_U9510
} config_board_type;

typedef enum _config_sensor_type {
	E_SENSOR_TYPE_PLATFORM = 0,
	E_SENSOR_TYPE_PHONE
} config_sensor_type;

typedef enum _config_apds_type {
	E_APDS_TYPE_D2 = 0,
	E_APDS_TYPE_U9701L,
	E_APDS_TYPE_U9700L
} config_apds_type;

typedef enum _config_iomux_type {
	E_IOMUX_PALTFORM_ES = 0,
	E_IOMUX_PALTFORM_CS,
	E_IOMUX_PHONE_ES,
	E_IOMUX_PHONE_CS,
	E_IOMUX_PHONE_C9800D,
	E_IOMUX_PHONE_U9700L,
	E_IOMUX_PHONE_U9900,
	E_IOMUX_PHONE_U9701L,
    E_IOMUX_PHONE_G710C,
    E_IOMUX_PHONE_G710C_VB,
	E_IOMUX_MAX
} config_iomux_type;

typedef enum _sd_detect_type
{
        GPIO_LOW_MEAN_DETECTED =0,
        GPIO_HIGH_MEAN_DETECTED,
        SD_DETECT_TYPE_MAX
}sd_detect_type;

typedef enum _config_felica_board_type {
	E_FELICA_BOARD_TYPE_U9701 = 0,
	E_FELICA_BOARD_TYPE_U9700
} config_felica_board_type;

typedef enum _config_touchkey_key_port {
    E_TOUCHKEY_KEY_PORT_U9700S0 = 7,
    E_TOUCHKEY_KEY_PORT_U9700S1 = 14,
    E_TOUCHKEY_KEY_PORT_U9701S1 = 15,
} config_touchkey_key_port;

typedef enum _config_touchkey_regulator_vout {
    E_TOUCHKEY_REGULATOR_VOUT13 = 13,
    E_TOUCHKEY_REGULATOR_VOUT17 = 17,
} config_touchkey_regulator_vout;

typedef enum _config_touchkey_led_brightness {
    E_TOUCHKEY_KEY_BRIGHTNESS_11 = 11,
    E_TOUCHKEY_KEY_BRIGHTNESS_20 = 20,
} config_touchkey_led_brightness;

typedef enum _config_touchkey_led_gpio_control {
    E_TOUCHKEY_KEY_LED_GPIO_NONE = 0,
    E_TOUCHKEY_KEY_LED_GPIO_CONTROL= 1,
} config_touchkey_led_gpio_control;

typedef enum _config_uart_headset_type {
	E_UART_HEADSET_TYPE_NOT_SUPPORTED = 0,
	E_UART_HEADSET_TYPE_LEGACY,
	E_UART_HEADSET_TYPE_SMART
} config_uart_headset_type;

typedef enum _config_camera_sensor_flip_type {
	E_CAMERA_SENSOR_FLIP_TYPE_NONE = 0,
	E_CAMERA_SENSOR_FLIP_TYPE_H_V,
	E_CAMERA_SENSOR_FLIP_TYPE_H,
	E_CAMERA_SENSOR_FLIP_TYPE_V
} config_camera_sensor_flip_type;

typedef enum _config_u9700_ldo {
    E_U9700_LDO_CTRL = 8,
  } config_u9700_ldo;

#ifdef CONFIG_HUAWEI_GPIO_UNITE

extern struct gpio_config_type *get_gpio_config_table(void);
extern int get_gpio_num_by_name(char *name);
extern struct pm_gpio_cfg_t *get_pm_gpio_config_table(void);
extern int get_pm_gpio_num_by_name(char *name);

#endif

#ifdef CONFIG_HW_POWER_TREE
extern struct hw_config_power_tree *get_power_config_table(void);
#endif

extern int get_hsd_invert(void);
extern int get_hs_keys(void);
extern int get_modem_type(char *ptype);
extern int get_if_use_3p75M_uart_clock(void);
extern int get_I2C_devices(char *ptype);
extern int get_connectivity_devices(char *ptype);
extern int get_motor_board_type(void);
extern int get_wifi_wl18xx_dcdc_config(void);
extern bool is_felica_exist(void);
extern int get_felica_board_type(void);
extern int get_uart_headset_type(void);
extern int get_touchkey_key_port(void);
extern int get_touchkey_led_gpio_enable(void);
extern int get_touchkey_regulator_vout(void);
extern int get_touchkey_led_brightness(void);
extern int get_primary_sensor_flip_type(void);
extern int get_secondary_sensor_flip_type(void);
extern int check_suspensory_camera(char *cname);
extern int get_camera_focus_key(void);
extern int get_u9700_ldo_ctrl(void);
#endif
