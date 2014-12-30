/*The file is auto generated by tools, don't modify it manully.*/
#include <hsad/configdata.h>
config_pair  hw_CS_U9510_configs [] = {
    {"arch/arch_type", (unsigned int)2, E_CONFIG_DATA_TYPE_INT },
    {"audio/audience", (const unsigned int)(unsigned int*)"ES305", E_CONFIG_DATA_TYPE_STRING },
    {"audio/digital_mic", (unsigned int)0, E_CONFIG_DATA_TYPE_INT },
    {"audio/dual_mic", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"audio/hs_keys", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"audio/hs_pa", (const unsigned int)(unsigned int*)"TPA6132", E_CONFIG_DATA_TYPE_STRING },
    {"audio/hsd_invert", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"audio/spk_pa", (const unsigned int)(unsigned int*)"BOOST5V", E_CONFIG_DATA_TYPE_STRING },
    {"audio/spk_route", (const unsigned int)(unsigned int*)"LINEOUT_L", E_CONFIG_DATA_TYPE_STRING },
    {"battery/battery_type", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"board/board_type", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"camera/camera_timing_type", (unsigned int)0, E_CONFIG_DATA_TYPE_INT },
    {"camera/faceIgnoreCount", (unsigned int)9, E_CONFIG_DATA_TYPE_INT },
    {"camera/primary_sensor", (unsigned int)90, E_CONFIG_DATA_TYPE_INT },
    {"camera/secondary_sensor", (unsigned int)270, E_CONFIG_DATA_TYPE_INT },
    {"felica/board_type", (unsigned int)0, E_CONFIG_DATA_TYPE_INT },
    {"felica/felica", (unsigned int)0, E_CONFIG_DATA_TYPE_BOOL },
    {"gas_gauge/charge_current", (unsigned int)1000, E_CONFIG_DATA_TYPE_INT },
    {"gas_gauge/charge_voltage", (unsigned int)4200, E_CONFIG_DATA_TYPE_INT },
    {"gas_gauge/firmware_name", (const unsigned int)(unsigned int*)"BQ27510_GUANGYU_1670", E_CONFIG_DATA_TYPE_STRING },
    {"gas_gauge/version", (unsigned int)0x14400a4, E_CONFIG_DATA_TYPE_INT },
    {"iomux/iomux_type", (unsigned int)3, E_CONFIG_DATA_TYPE_INT },
    {"keypad/keypad_type", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"lcd/lcd_type", (unsigned int)0, E_CONFIG_DATA_TYPE_INT },
    {"product/name", (const unsigned int)(unsigned int*)"CS_U9510", E_CONFIG_DATA_TYPE_STRING },
    {"product_thermal_type/product_name", (const unsigned int)(unsigned int*)"U9510", E_CONFIG_DATA_TYPE_STRING },
    {"sensor/sensor_type", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"touchscreen/touchscreen_type", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {"usbphy/usbphy_type", (unsigned int)1, E_CONFIG_DATA_TYPE_INT },
    {0, 0, 0}
 };
struct board_id_general_struct config_common_CS_U9510= {
	.name=COMMON_MODULE_NAME,
	.board_id=BOARD_ID_CS_U9510,
	.data_array={.config_pair_ptr=hw_CS_U9510_configs},
	.list={NULL,NULL},
};
