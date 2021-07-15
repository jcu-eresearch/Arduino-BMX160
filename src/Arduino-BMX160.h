/*
 * Arduino-BMX160 is an Arduino Library for the Bosch BMX160 chip.
 * Copyright (C) 2021  eResearch, James Cook University
 * Author: NigelB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Repository: https://github.com/jcu-eresearch/Arduino-BMX160
 *
 */


#ifndef ARGOSTAG_ARDUINO_BMX160_H
#define ARGOSTAG_ARDUINO_BMX160_H

#include "bmi160.h"
#include "bmm150_defs.h"
#include "SPI.h"
#include "Wire.h"

/*
 * RETURN_ON_ERROR_u_u tests a uint8_t result != BMI160_OK, and returns a uint8_t
 */
#define RETURN_ON_ERROR_u_u(exp)  {uint8_t result = (exp);if(result != BMI160_OK) {return result;}}

/*
 * RETURN_ON_ERROR_u_e tests a uint8_t result != BMI160_OK, and return a ArduinoBMX160_Status_e
 */
#define RETURN_ON_ERROR_u_e(exp)  {uint8_t result = (exp);if(result != BMI160_OK) {return BMI160_OK?ArduinoBMX160_Status_OK:ArduinoBMX160_Status_ERROR;}}

/**
 * RETURN_ON_ERROR_u_e tests a ArduinoBMX160_Status_e result != BMI160_OK, and return a ArduinoBMX160_Status_e
 */
#define RETURN_ON_ERROR_e_e(exp)  {ArduinoBMX160_Status_e result = (exp);if(result != ArduinoBMX160_Status_OK) {return result;}}

enum ArduinoBMX160_Status_e
{
    ArduinoBMX160_Status_UNDEFINED           = 0b00000000,
    ArduinoBMX160_Status_OK                  = 0b00000001,
    ArduinoBMX160_Status_ERROR               = 0b00000010,
    ArduinoBMX160_Status_INVALID_PARAM_ERROR = 0b00000110,
};


struct ArduinoBMX160SPI
{
    HardwareSPI *spi;
    PinName CS;
};

struct bmx160_mag_cfg {
    bmi160_aux_cfg mag;
    uint8_t power;
};

class ArduinoBMX160
{

public:

    enum BMX160_IFType_e{
        BMC160_I2C = BMI160_I2C_INTF,
        BMC160_SPI = BMI160_SPI_INTF,
    };

    enum BMX160_Mag_ODR_e
    {
        BMX160_Mag_ODR_25_on_32 = 0b0001,
        BMX160_Mag_ODR_25_on_16 = 0b0010,
        BMX160_Mag_ODR_3_125    = 0b0011,
        BMX160_Mag_ODR_6_25     = 0b0100,
        BMX160_Mag_ODR_12_5     = 0b0101,
        BMX160_Mag_ODR_25       = 0b0110,
        BMX160_Mag_ODR_50       = 0b0111,
        BMX160_Mag_ODR_100      = 0b1000,
        BMX160_Mag_ODR_200      = 0b1001,
        BMX160_Mag_ODR_400      = 0b1010,
        BMX160_Mag_ODR_800      = 0b1011,
    };

    enum BM_160_Chip_Id_e
    {
        UNDEFINED_Chip_Id = 0x00,
        BMI160_Chip_Id = 0xD1,
        BMX160_Chip_Id = 0xD8
    };

    explicit ArduinoBMX160(BM_160_Chip_Id_e chip_id, SPIClass *spi, PinName CS);
    explicit ArduinoBMX160(BM_160_Chip_Id_e chip_id, TwoWire *i2c);

    ArduinoBMX160_Status_e begin();

    ArduinoBMX160_Status_e readData(struct bmi160_sensor_data *mag, struct bmi160_sensor_data *gyro, struct bmi160_sensor_data *accel);

    ArduinoBMX160_Status_e getIMUConfigs(bmi160_cfg &accel, bmi160_cfg &gyro, bmx160_mag_cfg &mag);
    ArduinoBMX160_Status_e setIMUConfigs(bmi160_cfg &accel, bmi160_cfg &gyro, bmx160_mag_cfg &mag);

private:
    uint8_t setMagPowerMode(uint8_t power_mode);
    ArduinoBMX160_Status_e readMagData(struct bmi160_sensor_data *mag);
    uint8_t setDataMode();
    uint8_t setReg(uint8_t reg, uint8_t val);
    uint8_t configureMag();
    struct bmi160_dev dev = {};
    struct ArduinoBMX160SPI spiInfo = {};
    BM_160_Chip_Id_e chip_id ;
};

#endif //ARGOSTAG_ARDUINO_BMX160_H
