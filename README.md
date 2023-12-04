# BMP388 driver for STM32 HAL
Driver for BMP388 that uses STM32_HAL libraries

## How to use
First of all you need to configure I2C with HAL library. After that make a variable from BMP388_Handle_TypeDef type, this type contains all variables that are necessary to use and configure BMP388. Then set I2C pointer in your variable and use BMP388_Init function

```c
BMP388_Handle_TypeDef hbmp388;
hbmp388._hi2c = &hi2c1;
BMP388_Init(&hbmp388);
```

to configure BMP388 use this functions

```c
BMP388_SetTempOS(BMP388_HandleTypeDef *bmp, uint8_t oversample);
BMP388_SetPressOS(BMP388_HandleTypeDef *bmp, uint8_t oversample);
BMP388_SetIIRFilterCoeff(BMP388_HandleTypeDef *bmp, uint8_t filtercoeff);
BMP388_SetOutputDataRate(BMP388_HandleTypeDef *bmp, uint8_t odr);
```

they made to configure registers 