/*
 * BMP388.c
 *
 *  Created on: Aug 18, 2023
 *      Author: sam
 */


/* Includes ------------------------------------------------------------------*/
#include "BMP388.h"
#include "math.h"

/* ----- PRIVATE FUNCTIONS PROTOTYPES ----- */
HAL_StatusTypeDef	BMP388_SoftReset(BMP388_HandleTypeDef *bmp);
HAL_StatusTypeDef	BMP388_GetCalibData(BMP388_HandleTypeDef *bmp);
			float	BMP388_CompensateTemp(BMP388_HandleTypeDef *bmp, uint32_t raw_temp, float *temp);
			float	BMP388_CompensatePress(BMP388_HandleTypeDef *bmp, float temp, uint32_t raw_press, float *press);
HAL_StatusTypeDef	BMP388_ReadBytes(BMP388_HandleTypeDef *bmp, BMP388_regs reg_addr, uint8_t *buff, uint8_t len);
HAL_StatusTypeDef	BMP388_WriteBytes(BMP388_HandleTypeDef *bmp, BMP388_regs reg_addr, uint8_t *buff, uint8_t len);




/* ---------------------------- */
/* ----- PUBLIC FUNCTIONS ----- */
/* ---------------------------- */

/*!
 *  @brief Function to initiate BMP388 and get calibration data
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 */
HAL_StatusTypeDef BMP388_Init(BMP388_HandleTypeDef *bmp){
	HAL_StatusTypeDef rslt;
	uint8_t chip_id;

	// Read CHIP_ID byte
	rslt = BMP388_ReadBytes(bmp, CHIP_ID, &chip_id, 1);
	if(rslt == HAL_OK && chip_id == BMP388_CHIP_ID){
		// using softreset command
		rslt = BMP388_SoftReset(bmp);
		if(rslt == HAL_OK){
			// get calibration data
			rslt = BMP388_GetCalibData(bmp);
		}
		else{
			return rslt;
		}
	}
	else{
		return rslt;
	}

	return rslt;
}



/*!
 *  @brief Function to set temperature measurment oversampling
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Wrong oversampling mode
 */
HAL_StatusTypeDef BMP388_SetTempOS(BMP388_HandleTypeDef *bmp, uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	bmp->_oversampling = (bmp->_oversampling & 0b11111000) | (oversample << 3);
	return HAL_OK;
}



/*!
 *  @brief Function to set pressure measurment oversampling
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Wrong oversampling mode
 */
HAL_StatusTypeDef BMP388_SetPressOS(BMP388_HandleTypeDef *bmp, uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	bmp->_oversampling = (bmp->_oversampling & 0b11000111) | oversample;
	return HAL_OK;
}




HAL_StatusTypeDef BMP388_SetIIRFilterCoeff(BMP388_HandleTypeDef *bmp, uint8_t filtercoeff){
	if(filtercoeff > BMP3_IIR_FILTER_COEFF_127){
		return HAL_ERROR;
	}
	bmp->_filtercoeff = filtercoeff;
	return HAL_OK;
}




HAL_StatusTypeDef BMP388_SetOutputDataRate(BMP388_HandleTypeDef *bmp, uint8_t odr){
	if(odr > BMP3_ODR_0_001_HZ){
		return HAL_ERROR;
	}
	bmp->_odr = odr;
	return HAL_OK;
}



/*!
 *  @brief Function to read pressure and temperature from BMP388 in forced mode
 *
 *	@param[in]	bmp				: Pointer to BMP388 structure
 *  @param[out] raw_pressure	: Pointer to the variable that contain uncompensated pressure data.
 *	@param[out] raw_temperature	: Pointer to the variable that contain uncompensated temperature data.
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 */
HAL_StatusTypeDef BMP388_ReadRawPressTempTime(BMP388_HandleTypeDef *bmp, uint32_t *raw_pressure, uint32_t *raw_temperature, uint32_t *time){
	HAL_StatusTypeDef rslt;
	uint8_t pwr_ctrl = BMP388_PWR_CTRL_PRESS_ON | BMP388_PWR_CTRL_TEMP_ON | BMP388_PWR_CTRL_MODE_FORCED;

	uint8_t oversampling = bmp->_oversampling;
	uint8_t odr = bmp->_odr;
	uint8_t filtercoeff = bmp->_filtercoeff;



	// Set OSR register
	rslt = BMP388_WriteBytes(bmp, OSR, &oversampling, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set ODR register
	rslt = BMP388_WriteBytes(bmp, ODR, &odr, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set CONFIG register
	rslt = BMP388_WriteBytes(bmp, CONFIG, &filtercoeff, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set PWR_CTRL register
	rslt = BMP388_WriteBytes(bmp, PWR_CTRL, &pwr_ctrl, 1);
	if(rslt != HAL_OK){
		return rslt;
	}

	uint8_t raw_data[11];
	// Get raw data for pressure and temperature
	rslt = BMP388_ReadBytes(bmp, DATA_0, raw_data, 6);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Temporary variables to store the sensor data
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	// Parsing pressure data
	data_xlsb = (uint32_t)raw_data[0];
	data_lsb = (uint32_t)raw_data[1] << 8;
	data_msb = (uint32_t)raw_data[2] << 16;
	*raw_pressure = data_msb | data_lsb | data_xlsb;

	// Parsing temperature data
	data_xlsb = (uint32_t)raw_data[3];
	data_lsb = (uint32_t)raw_data[4] << 8;
	data_msb = (uint32_t)raw_data[5] << 16;
	*raw_temperature = data_msb | data_lsb | data_xlsb;

	// Parsing time bytes
	data_xlsb = (uint32_t)raw_data[8];
	data_lsb = (uint32_t)raw_data[9] << 8;
	data_msb = (uint32_t)raw_data[10] << 16;
	*time = data_msb | data_lsb | data_xlsb;


	return rslt;
}



/*!
 *  @brief Function to compensate pressure and temperature from BMP388
 *
 *	@param[in]	bmp				: Pointer to BMP388 structure
 *  @param[in] raw_pressure		: Variable that contain uncompensated pressure data.
 *	@param[in] raw_temperature	: Variable that contain uncompensated temperature data.
 *	@param[out] pressure		: Pointer to the variable that contain pressure.
 *	@param[out]	temperature		: Pointer to the variable that contain temperature.
 *
 *  @return none
 */
void BMP388_CompensateRawPressTemp(BMP388_HandleTypeDef *bmp, uint32_t raw_pressure, uint32_t raw_temperature,
									  	  	  	  	  	  	  float *pressure, float *temperature){
	float temp;
	float press;

	BMP388_CompensateTemp(bmp, raw_temperature, &temp);
	BMP388_CompensatePress(bmp, temp, raw_pressure, &press);

	*pressure = press;
	*temperature = temp;
}



/*!
 *  @brief Function to find altutede value based on pressure measurment
 *
 *	@param[in] ground_pressure	: Pressure at ground
 *  @param[in] pressure			: Pressure that measured at flight.
 *
 *  @return Altitude
 */
float BMP388_FindAltitude(float ground_pressure, float pressure){
	  // Equation taken from BMP180 datasheet (page 16):
	  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	  // Note that using the equation from wikipedia can give bad results
	  // at high altitude. See this thread for more information:
	  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	  return 44330.0 * (1.0 - pow(pressure / ground_pressure, 0.1903));
}





/* ----------------------------- */
/* ----- PRIVATE FUNCTIONS ----- */
/* ----------------------------- */

/*!
 *  @brief Function to send softreset cmnd
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 */
HAL_StatusTypeDef BMP388_SoftReset(BMP388_HandleTypeDef *bmp){
	uint8_t rst_cmnd = BMP388_SOFTRESET;
    uint8_t cmd_rdy_status;
    uint8_t cmd_err_status;

	HAL_StatusTypeDef rslt;

	// Reading STATUS reg to understand that the BMP388 is ready to receive command
	rslt = BMP388_ReadBytes(bmp, STATUS, &cmd_rdy_status, 1);
	if((rslt == HAL_OK) && (cmd_rdy_status & BMP388_CMD_RDY)){
		// Writing SOFTRESET command to CMD reg
		rslt = BMP388_WriteBytes(bmp, CMD, &rst_cmnd, 1);
		if(rslt == HAL_OK){
			// 2 ms pause then check ERR reg
			HAL_Delay(2);
			rslt = BMP388_ReadBytes(bmp, ERR_REG, &cmd_err_status, 1);
			if((cmd_err_status & CMD) || (rslt != HAL_OK)){
				return rslt;
			}
		}
		else{
			return rslt;
		}
	}

	return rslt;
}



/*!
 *  @brief Function to get calibration data
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 */
HAL_StatusTypeDef BMP388_GetCalibData(BMP388_HandleTypeDef *bmp){
	HAL_StatusTypeDef rslt;
	uint8_t calib_buff[BMP388_CALIBDATA_LEN] = {0};

	uint16_t	raw_par_t1;
	uint16_t	raw_par_t2;
	int8_t		raw_par_t3;
	int16_t		raw_par_p1;
	int16_t		raw_par_p2;
	int8_t		raw_par_p3;
	int8_t		raw_par_p4;
	uint16_t	raw_par_p5;
	uint16_t	raw_par_p6;
	int8_t		raw_par_p7;
	int8_t		raw_par_p8;
	int16_t		raw_par_p9;
	int8_t		raw_par_p10;
	int8_t		raw_par_p11;

	rslt = BMP388_ReadBytes(bmp, CALIB_DATA, calib_buff, BMP388_CALIBDATA_LEN);

	float temp_var;
	if(rslt == HAL_OK){
		// PAR_T1
		temp_var = 0.00390625f;
		raw_par_t1 = ((uint16_t)calib_buff[1] << 8) | (uint16_t)calib_buff[0];
		bmp->_calib_data.par_t1 = (float)raw_par_t1 / temp_var;
		// PAR_T2
		temp_var = 1073741824.f;
		raw_par_t2 = ((uint16_t)calib_buff[3] << 8) | (uint16_t)calib_buff[2];
		bmp->_calib_data.par_t2 = (float)raw_par_t2 / temp_var;
		// PAR_T3
		temp_var = 281474976710656.f;
		raw_par_t3 = calib_buff[4];
		bmp->_calib_data.par_t3 = (float)raw_par_t3 / temp_var;
		// PAR_P1
		temp_var = 1048576.f;
		raw_par_p1 = ((int16_t)calib_buff[6] << 8) | (int16_t)calib_buff[5];
		bmp->_calib_data.par_p1 = ((float)raw_par_p1 - 16384) / temp_var;
		// PAR_P2
		temp_var = 536870912.f;
		raw_par_p2 = ((int16_t)calib_buff[8] << 8) | (int16_t)calib_buff[7];
		bmp->_calib_data.par_p2 = ((float)raw_par_p2 - 16384) / temp_var;
		// PAR_P3
		temp_var = 4294967296.f;
		raw_par_p3 = (int8_t)calib_buff[9];
		bmp->_calib_data.par_p3 = (float)raw_par_p3 / temp_var;
		// PAR_P4
		temp_var = 137438953472.f;
		raw_par_p4 = (int8_t)calib_buff[10];
		bmp->_calib_data.par_p4 = (float)raw_par_p4 / temp_var;
		// PAR_P5
		temp_var = 0.125f;
		raw_par_p5 = ((uint16_t)calib_buff[12] << 8) | (uint16_t)calib_buff[11];
		bmp->_calib_data.par_p5 = (float)raw_par_p5 / temp_var;
		// PAR_P6
		temp_var = 64.f;
		raw_par_p6 = ((uint16_t)calib_buff[14] << 8) | (uint16_t)calib_buff[13];
		bmp->_calib_data.par_p6 = (float)raw_par_p6 / temp_var;
		// PAR_P7
		temp_var = 256.f;
		raw_par_p7 = (int8_t)calib_buff[15];
		bmp->_calib_data.par_p7 = (float)raw_par_p7 / temp_var;
		// PAR_P8
		temp_var = 32768.f;
		raw_par_p8 = (int8_t)calib_buff[16];
		bmp->_calib_data.par_p8 = (float)raw_par_p8 / temp_var;
		// PAR_P9
		temp_var = 281474976710656.f;
		raw_par_p9 = ((int16_t)calib_buff[18] << 8) | (int16_t)calib_buff[17];
		bmp->_calib_data.par_p9 = (float)raw_par_p9 / temp_var;
		// PAR_P10
		temp_var = 281474976710656.f;
		raw_par_p10 = (int8_t)calib_buff[19];
		bmp->_calib_data.par_p10 = (float)raw_par_p10 / temp_var;
		// PAR_P11
		temp_var = 36893488147419103232.f;
		raw_par_p11 = (int8_t)calib_buff[20];
		bmp->_calib_data.par_p11 = (float)raw_par_p11 / temp_var;
	}
	return rslt;
}



/*!
 *  @brief Function to compensate raw temperature data
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *	@parem[in] raw_temp		: Raw temperature data that need to be compensated
 *	@param[out] temp		: Measured temp in celsius
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 */
float BMP388_CompensateTemp(BMP388_HandleTypeDef *bmp, uint32_t raw_temp, float *temp){
    float partial_data1 = (float)(raw_temp - bmp->_calib_data.par_t1);;
    float partial_data2 = (float)(partial_data1 * bmp->_calib_data.par_t2);

    *temp = partial_data2 + (partial_data1 * partial_data1) * bmp->_calib_data.par_t3;

    return *temp;
}



/*!
 *  @brief Function to compensate raw pressure data
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *	@param[in] temp			: Temperature that assosiated with pressure measurment
 *	@param[in] raw_press	: Raw pressure data that need to be compensated
 *	@param[out] press		: Measured pressure in Pa
 *
 *  @return Status of execution
 *  @retval = press			: Compensated pressure value
 */
float BMP388_CompensatePress(BMP388_HandleTypeDef *bmp, float temp, uint32_t raw_press, float *press){
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;



    partial_data1 = bmp->_calib_data.par_p6 * temp;
    partial_data2 = bmp->_calib_data.par_p7 * (temp * temp);
    partial_data3 = bmp->_calib_data.par_p8 * (temp * temp * temp);
    partial_out1 = bmp->_calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = bmp->_calib_data.par_p2 * temp;
    partial_data2 = bmp->_calib_data.par_p3 * (temp * temp);
    partial_data3 = bmp->_calib_data.par_p4 * (temp * temp * temp);
    partial_out2 = (float)raw_press * (bmp->_calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = bmp->_calib_data.par_p9 + bmp->_calib_data.par_p10 * temp;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)raw_press * (float)raw_press * (float)raw_press) * bmp->_calib_data.par_p11;

    *press = partial_out1 + partial_out2 + partial_data4;

    return *press;
}



/*!
 *  @brief Function to read byte from BMP388 in blocking mode
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *  @param[in] reg_addr     : Register address.
 *  @param[out] buff	    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : Amount of bytes to read.
 *
 *  @return Status of execution
 *  @retval = HAL_OK 		-> Success
 *  @retval != HAL_ERROR 	-> Failure Info
 */
HAL_StatusTypeDef BMP388_ReadBytes(BMP388_HandleTypeDef *bmp, BMP388_regs reg_addr, uint8_t *buff, uint8_t len){
	return HAL_I2C_Mem_Read(bmp->_hi2c, BMP388_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, len, 100);
}



/*!
 *  @brief Function to write byte from BMP388 in blocking mode
 *
 *	@param[in] bmp			: Pointer to BMP388 structure
 *  @param[in] reg_addr     : Register address.
 *  @param[out] buff	    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : Amount of bytes to write.
 *
 *  @return Status of execution
 *  @retval = HAL_OK 		-> Success
 *  @retval != HAL_ERROR 	-> Failure Info
 */
HAL_StatusTypeDef BMP388_WriteBytes(BMP388_HandleTypeDef *bmp, BMP388_regs reg_addr, uint8_t *buff, uint8_t len){
	return HAL_I2C_Mem_Write(bmp->_hi2c, BMP388_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, len, 100);
}
