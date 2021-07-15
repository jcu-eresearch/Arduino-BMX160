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
#include "bmi160.h"

//All of the code that calls this has already checked that dev != NULL
int8_t bmi160_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len, const struct bmi160_dev *dev)
{
    if(dev->intf == ArduinoBMX160::BMC160_I2C)
    {
        TwoWire *i2c = (TwoWire*)dev->ctx;
        i2c->beginTransmission(dev_addr);
        i2c->write(reg_addr);
        i2c->endTransmission();

        i2c->requestFrom(dev_addr, len);
        for(size_t i = 0; i < len; i++)
        {
            read_data[i] = i2c->read();
        }

    }else if(dev->intf == ArduinoBMX160::BMC160_SPI)
    {
        ArduinoBMX160SPI *spi = (ArduinoBMX160SPI*)dev->ctx;
        //ToDo: Implement SPI bmi160_read_reg
    }
    return BMI160_OK;
}

int8_t bmi160_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len, const struct bmi160_dev *dev)
{
    if(dev->intf == ArduinoBMX160::BMC160_I2C)
    {
        TwoWire *i2c = (TwoWire*)dev->ctx;
        i2c->beginTransmission(dev_addr);
        i2c->write(reg_addr);
        i2c->write(write_data, len);
        i2c->endTransmission();

    }else if(dev->intf == ArduinoBMX160::BMC160_SPI)
    {
        ArduinoBMX160SPI *spi = (ArduinoBMX160SPI*)dev->ctx;
        //ToDo: Implement SPI bmi160_write_reg
    }
    return BMI160_OK;
}

void bmi160_delay_ms(uint32_t period)
{
    delay(period);
}


