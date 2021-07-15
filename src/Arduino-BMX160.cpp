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

#include "Arduino-BMX160.h"


ArduinoBMX160::ArduinoBMX160(BM_160_Chip_Id_e chip_id, arduino::SPIClass *spi, PinName CS){
    dev.intf = BMC160_SPI;
    spiInfo.spi = spi;
    spiInfo.CS = CS;
    dev.ctx = &spiInfo;
    dev.expected_chip_id = chip_id;
};
ArduinoBMX160::ArduinoBMX160(BM_160_Chip_Id_e chip_id, TwoWire *i2c){
    dev.intf = BMC160_I2C;
    dev.ctx = i2c;
    dev.expected_chip_id = chip_id;
};

ArduinoBMX160_Status_e ArduinoBMX160::begin()
{
    dev.id = BMI160_I2C_ADDR;
    RETURN_ON_ERROR_u_e(bmi160_init(&dev));

    if(dev.expected_chip_id == BMX160_Chip_Id)
    {
        RETURN_ON_ERROR_u_e(configureMag());
    }

    return ArduinoBMX160_Status_OK;
}

uint8_t ArduinoBMX160::configureMag()
{

    uint8_t status;
    uint8_t mag_i2c_dev_address = 0;
    RETURN_ON_ERROR_u_u(bmi160_get_regs(BMI160_AUX_IF_0_ADDR, &mag_i2c_dev_address, 1, &dev));

    dev.aux_cfg.aux_i2c_addr = (mag_i2c_dev_address >> 1);
    dev.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_0;
    dev.aux_cfg.aux_sensor_enable = 1;
    dev.aux_cfg.manual_enable = 1;
    dev.aux_cfg.aux_odr = BMX160_Mag_ODR_12_5;
    RETURN_ON_ERROR_u_u(bmi160_aux_init(&dev));                                                                  //0x19
                                                                                                                  //0x80
    uint8_t register_data;


    register_data = BMM150_SET_BITS_POS_0(0, BMM150_PWR_CNTRL, BMM150_POWER_CNTRL_ENABLE);                //0x01
    status = bmi160_aux_write(BMM150_REG_POWER_CONTROL, &register_data, 1, &dev);                       //0x4B
    RETURN_ON_ERROR_u_u(status);

    //These values Differ from datasheet -
    register_data = BMM150_REPXY_REGULAR;                                                                         //0x04
    status = bmi160_aux_write(BMM150_REG_REP_XY, &register_data, 1, &dev);                              //0x51
    RETURN_ON_ERROR_u_u(status);
    register_data = BMM150_REPZ_REGULAR;                                                                          //0x07
    status = bmi160_aux_write(BMM150_REG_REP_Z, &register_data, 1, &dev);                               //0x52
    RETURN_ON_ERROR_u_u(status);

    //Set Data Mode
    register_data = BMM150_SET_BITS(0, BMM150_OP_MODE, BMM150_POWERMODE_FORCED);                    //0x02
    status = bmi160_aux_write(BMM150_REG_OP_MODE, &register_data, 1, &dev);                             //0x4C
    RETURN_ON_ERROR_u_u(status);

    uint8_t data_addr = BMM150_REG_DATA_X_LSB;                                                                    //0x42
    status = bmi160_set_aux_auto_mode(&data_addr, &dev);                                                         //0x05
    RETURN_ON_ERROR_u_u(status);

    dev.aux_cfg.manual_enable = 0;
    status = bmi160_config_aux_mode(&dev);                                                                  //0x00
    RETURN_ON_ERROR_u_u(status);

    setMagPowerMode(BMI160_AUX_LOWPOWER_MODE);                                                                    //0x1A

    return status;


//    setReg(0x7E, 0x19);
//    delay(1);
//
//    setReg(0x4C, 0x80);
//    setReg(0x4F, 0x01);
//    setReg(0x4E, 0x4B);
//    setReg(0x4F, 0x01);
//    setReg(0x4E, 0x51);
//    setReg(0x4F, 0x0E);
//    setReg(0x4E, 0x52);
//    setReg(0x4F, 0x02);
//    setReg(0x4E, 0x4C);
//    setReg(0x4D, 0x42);
//    setReg(0x44, 0x08);
//    return setReg(0x4C, 0x03);
//    setReg(0x, 0x);
}

uint8_t ArduinoBMX160::setReg(uint8_t reg, uint8_t val)
{
    return bmi160_write_reg(dev.id, reg, &val, 1, &dev);
//    return bmi160_set_regs(reg, &val, 1, &dev);
}

uint8_t ArduinoBMX160::setDataMode()
{
    uint8_t status;
    uint8_t register_data;
    //Set Data Mode
    register_data = BMM150_SET_BITS(0, BMM150_OP_MODE, BMM150_POWERMODE_FORCED);                    //0x02
    status |= bmi160_aux_write(BMM150_REG_OP_MODE, &register_data, 1, &dev);                             //0x4C

    uint8_t data_addr = BMM150_REG_DATA_X_LSB;
    status |= bmi160_set_regs(BMI160_AUX_IF_2_ADDR, &register_data, 1, &dev);                             //0x4C

    return status;
}

ArduinoBMX160_Status_e ArduinoBMX160::readMagData(struct bmi160_sensor_data *mag)
{
    uint8_t data[23] = {0};
    uint8_t result = bmi160_get_regs(BMI160_AUX_DATA_ADDR, data, sizeof(data), &dev);
    if(result == BMI160_OK)
    {
        mag->x = (int16_t)((data[1] << 8) | data[0]);
        mag->y = (int16_t)((data[3] << 8) | data[2]);
        mag->z = (int16_t)((data[5] << 8) | data[4]);
    }

    return result == BMI160_OK ? ArduinoBMX160_Status_OK : ArduinoBMX160_Status_ERROR;
}

ArduinoBMX160_Status_e ArduinoBMX160::readData(struct bmi160_sensor_data *mag, struct bmi160_sensor_data *gyro, struct bmi160_sensor_data *accel)
{
    RETURN_ON_ERROR_e_e(readMagData(mag));
    RETURN_ON_ERROR_u_e(bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), accel, gyro, &dev));
    return ArduinoBMX160_Status_OK;
}

ArduinoBMX160_Status_e ArduinoBMX160::getIMUConfigs(bmi160_cfg &accel, bmi160_cfg &gyro, bmx160_mag_cfg &mag) {
    int8_t result = bmi160_get_sens_conf(&dev);
    bmi160_pmu_status power_status;
    bmi160_get_power_mode(&power_status, &dev);
    if(result == BMI160_OK) {
        accel.bw = dev.accel_cfg.bw;
        accel.odr = dev.accel_cfg.odr;
        dev.accel_cfg.power = power_status.accel_pmu_status;
        accel.power = dev.accel_cfg.power;
        accel.range = dev.accel_cfg.range;

        gyro.bw = dev.gyro_cfg.bw;
        gyro.odr = dev.gyro_cfg.odr;
        dev.gyro_cfg.power = power_status.gyro_pmu_status;
        gyro.power = dev.gyro_cfg.power;
        gyro.range = dev.gyro_cfg.range;

        mag.power = power_status.aux_pmu_status;

        return ArduinoBMX160_Status_OK;
    }
    return ArduinoBMX160_Status_ERROR;
}

ArduinoBMX160_Status_e ArduinoBMX160::setIMUConfigs(bmi160_cfg &accel, bmi160_cfg &gyro, bmx160_mag_cfg &mag)
{
    dev.accel_cfg.bw = accel.bw;
    dev.accel_cfg.odr = accel.odr;
    dev.accel_cfg.power = accel.power;
    dev.accel_cfg.range = accel.range;

    dev.gyro_cfg.bw = gyro.bw;
    dev.gyro_cfg.odr = gyro.odr;
    dev.gyro_cfg.power = gyro.power;
    dev.gyro_cfg.range = gyro.range;

    dev.aux_cfg.aux_odr = mag.mag.aux_odr;
    dev.aux_cfg.manual_enable = mag.mag.manual_enable;
    dev.aux_cfg.aux_rd_burst_len = mag.mag.aux_rd_burst_len;
    dev.aux_cfg.aux_sensor_enable = mag.mag.aux_sensor_enable;

    RETURN_ON_ERROR_u_e(bmi160_set_sens_conf(&dev));
    RETURN_ON_ERROR_u_e(bmi160_set_power_mode(&dev));
    RETURN_ON_ERROR_u_e(bmi160_config_aux_mode(&dev));
//    RETURN_ON_ERROR_u_e(setDataMode());
    RETURN_ON_ERROR_u_e(setMagPowerMode(mag.power));
    return ArduinoBMX160_Status_OK;
}

uint8_t ArduinoBMX160::setMagPowerMode(uint8_t power_mode)
{
    if (power_mode == BMI160_AUX_SUSPEND_MODE || power_mode == BMI160_AUX_NORMAL_MODE || power_mode == BMI160_AUX_LOWPOWER_MODE) {
        return bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &power_mode, 1, &dev) == BMI160_OK?ArduinoBMX160_Status_OK:ArduinoBMX160_Status_ERROR;
    }
    return ArduinoBMX160_Status_INVALID_PARAM_ERROR;
}



