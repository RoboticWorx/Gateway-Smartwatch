set(srcs "main.c"
    "decode_png.c"
    "pngle.c"
    "bme680.c" 
    "i2cdev.c" 
    "VL53L1X_api.c" 
    "VL53L1X_calibration.c" 
    "RdWr_Byte.c"
    "ds3231.c"
    "icm42670.c"
    "mcp342x.c"
    )

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ".")
