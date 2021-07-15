# Arduino BMX160 and BMI160

This library wraps a [forked](https://github.com/jcu-eresearch/BMI160_driver) version of Bosch's [BMI160](https://github.com/BoschSensortec/BMI160_driver) library and uses some headers from Bosch's [BMM150]( https://github.com/BoschSensortec/BMM150-Sensor-API.git) library.

My understanding, from [this](https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BMX160-magnetometer-problem/m-p/7664/highlight/true#M594) post is that a BMX160 is a BMI160 with a BMM150 attached to the BMi160's secondary/auxiliary interface.

# Datasheet Confusion

When examining the BMI160 and BMX160 datasheets, be very careful as some register names have different 
addresses in the two datasheets:

|Address|BMI160 Reg Name|BMX160 Reg Address|
|-------|---------------|------------------|
| 0x4B  |  MAG_IF[0]    |                  |
| 0x4C  |  MAG_IF[1]    |  MAG_IF[0]       |
| 0x4C  |  MAG_IF[2]    |  MAG_IF[1]       |
| 0x4C  |  MAG_IF[3]    |  MAG_IF[2]       |
| 0x4C  |  MAG_IF[4]    |  MAG_IF[3]       |

