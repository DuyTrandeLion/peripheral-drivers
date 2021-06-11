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

    |TMF8x0x (Integration example only)|Tested|
    |---------|---------|
    |**TMF8701**||
    |**TMF8801**|**x**|
    |**TMF8805**||
    |**TMF8801_Driver_BareMetalMCU_v3.0.18.0**|**x**|

    |TMF8x0x (Integration example only)|Tested|
    |---------|---------|
    |**VL53L0X**|**x**|
    |**X-CUBE-53L0A1**|**x**|
    X-CUBE-53L0A1 product page: https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube-expansion-software/stm32-ode-sense-sw/x-cube-53l0a1.html 

    |AMG88xx|Tested|
    |---------|---------|
    |**AMG8833**|**x**|
    |**AMG8834**|**x**|
    |**amg8833-grid-eye_api_v2.1**|**x**|

    ```
    Grid-EYE API Lib is divided into 3 layers; users can select API functions from these layers
    according to the requirement.

    * API Lv1: APIs from this layer implement Grid-EYE data acquisition, transformation of temperature value and data format.
    * API Lv2: APIs from this layer implement filtering of original data, and provide functions for image processing, object detection, and human body recognition.
    * API Lv3 (binary code): APIs from this layer implement functions for object detection and object tracking.
    ```

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
4. If you use Ublox_GPS_GNSS library, also include ```Ublox_GPS_GNSS\lwgps\lwgps\src\include``` in include path.
# Limitations
1. Ublox GNSS:
    - Not all functions and NMEA protocols are supported.

# Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

# License
[MIT](https://choosealicense.com/licenses/mit/)
	
