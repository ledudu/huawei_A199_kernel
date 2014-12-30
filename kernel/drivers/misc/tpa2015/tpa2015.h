#ifndef TPA2015
#define TPA2015

#define TPA2015_NAME "tpa2015"

#define TPA2015_IOCTL_MAGIC 'u'

#define TPA2015_ENABLE      _IOW(TPA2015_IOCTL_MAGIC, 0xC0, unsigned)
#define TPA2015_DISABLE     _IOW(TPA2015_IOCTL_MAGIC, 0xC1, unsigned)

struct tpa2015_platform_data {
    uint32_t gpio_tpa2015_en;
};

#endif //TPA2015
