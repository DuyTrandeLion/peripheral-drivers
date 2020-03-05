# Peripheral Drivers Introduction
1. **Supported device list (tested on STM32/nRF52):**
    |ADS_ADC|Tested|
    |---------|---------|
    |**ADS8887**||
    |**ADS8885**||
    |**ADS8883**||
    |**ADS8881**|**x**|
    |**ADS8866**||
    |**ADS8339**||
    |**ADS8864**||
    |**ADS8319**||
    |**ADS8862**||
    |**ADS8860**||
    |**ADS8867**|*|
    |**ADS8865**|**x**|
    |**ADS8318**||
    |**ADS8863**||
    |**ADS8861**||
    |**ADS9110**||

    |DF_SAR_ADC|Tested|
    |---------|---------|
    |**LTC2500-32**|**x**|
    |**LTC2508-32**||
    |**LTC2512-24**||
    |**LTC2380-24**||    

    |EEPROM|Tested|
    |---------|---------|
    |**AT24Cxxxx**|**x**|
    |**24AAxxxx**|**x**|
    |**24LCxxxx**|**x**|
    
    |ENC624J600|Tested|
    |---------|---------|
    |**ENC624J600**||

    |LTC26x1|Tested|
    |---------|---------|
    |**LTC2601**|**x**|
    |**LTC2611**|**x**|

    |Ublox_GPS_GNSS|Tested|
    |---------|---------|
    |**SAM-M8N**|**x**|
    |**SAM-M8Q**|**x**|
    |**SAM-M9N**|**x**|

# Usage
1. Clone the repository and submodules:
    ```bash
    git clone --recurse-submodules -j8 https://github.com/DuyTrandeLion/peripheral-drivers.git
    ```
2. Add the include path in your project.
3. Add library files in your project.
4. If you use Ublox_GPS_GNSS library: 
In file **mimea.h**, comment function ```int minmea_gettime(struct timespec *ts, const struct minmea_date *date, const struct minmea_time *time_);```.
In file **mimea.c**, function ```bool minmea_check(const char *sentence, bool strict)```, comment these lines:
    ```C
    if (checksum != expected)
        return false;
    ```
# Limitations
1. Ublox GNSS:
    - Only a handful of frames is supported right now.
    - There's no support for omitting parts of the library from building. As a workaround, use the -ffunction-sections -Wl,--gc-sections linker flags (or equivalent) to remove the unused functions (parsers) from the final image.
    - Some systems lack timegm. On these systems, the recommended course of action is to build with -Dtimegm=mktime which will work correctly as long the system runs in the default UTC timezone.

# Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

# License
[MIT](https://choosealicense.com/licenses/mit/)
	
