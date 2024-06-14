# BMP388 driver for STM32 HAL
Driver for BMP388, compatible with STM32 HAL libraries.

## How to use

### Creating the BMP388 structure
First of all you need to configure I2C with HAL library. After that make a variable from BMP388_Handle_TypeDef type, this type contains all variables that are necessary to use and configure BMP388. Then set I2C pointer in your variable and use BMP388_Init function:

```c
BMP388_Handle_TypeDef hbmp388;
hbmp388.hi2c = &hi2c1;
BMP388_Init(&hbmp388);
```

### Setting Measurement Settings

To set the BMP388 mode, use the functions listed below. Recommended settings for specific situation you can find in table 10 of BMP388 datasheet in paragraph 3.5 "Recommended filter settings on use cases".

As an example, I will set the settings recommended when using the sensor on a drone (I leave the name of the BMP structure the same):

```c
BMP388_SetTempOS(&hbmp388, BMP388_NO_OVERSAMPLING);
BMP388_SetPressOS(&hbmp388, BMP388_OVERSAMPLING_8X);
BMP388_SetIIRFilterCoeff(&hbmp388, BMP3_IIR_FILTER_COEFF_3);
BMP388_SetOutputDataRate(&hbmp388, BMP3_ODR_50_HZ);
```

The specified settings will be stored in the BMP388 structure and will be written to the device registers when the measurement process starts.

### Measurement process

There are two ways to obtain barometer measurements:
1. single measurment per reading;
2. FIFO mode with some measurments per reading.

#### Single measurment per reading
If you want to use forced mode, here is the recipe: after setting of BMP mode, you need to use these commands.

```c
uint32_t raw_press = 0;
uint32_t raw_temp = 0;
uint32_t sensor_time = 0;
float press = 0;    
float temp = 0;
float device_alt = 0;

BMP388_ReadRawPressTempTime(&hbmp388, &raw_press, &raw_temp, &sensor_time);
BMP388_CompensateRawPressTemp(&hbmp388, raw_press, raw_temp, &press, &temp);

/* Be sure to take measurements at ground level before attempting 
*  to determine the height.
*  Make sure to make mesurment at "ground" level before trying
*  to find altitude. You will need to take at least 2 measurements 
* as the first measurement is always corrupted.
*/
device_alt = BMP388_FindAltitude(ground_press, press);
```

#### FIFO mode

FIFO mode means that BMP will store measurments in it's buffer (up to 512 bytes of data). This approach allows you to collect many measurements using a single request.
After setting of BMP mode, you need to use these commands:

```c
uint16_t fifo_len = 0;
uint8_t fifo_buff[512]; // May have smaller size if you want

BMP388_StartNormalModeFIFO(&hbmp388);
BMP388_GetFIFOLength(&hbmp388, &fifo_len);
BMP388_GetRawDataFIFO(&hbmp388, fifo_len, fifo_buff);
```

This set of commands will give a buffer with raw data. The resulting data can be iteratively converted to pressure and temperature.