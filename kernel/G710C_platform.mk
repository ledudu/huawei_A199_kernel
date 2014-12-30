PRODUCT_NAME=G710C
PRODUCT_BRAND=Huawei
#add macro to enable command type lcd
export USE_MIPI_CMD_OTM1282A_HD := true
#NOTICE! Do not config PRODUCT_DEVICE
export USE_EDGE_CAMERA_SETTINGS := true

export USE_AUDIO_RING_VOLUME_CHANGE := true

PRODUCT_LCD_DISPLAY=HD

ifeq ($(TARGET_VERSION_MODE),normal)
PRODUCT_PACKAGES += check_root
endif
