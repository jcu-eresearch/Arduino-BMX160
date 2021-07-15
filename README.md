# Arduino BMX160 and BMI160

# Datasheet Confusion

|Address|BMI160 Reg Name|BMX160 Reg Address|
|-------|---------------|------------------|
| 0x4B  |  MAG_IF[0]    |                  |
| 0x4C  |  MAG_IF[1]    |  MAG_IF[0]       |
| 0x4C  |  MAG_IF[2]    |  MAG_IF[1]       |
| 0x4C  |  MAG_IF[3]    |  MAG_IF[2]       |
| 0x4C  |  MAG_IF[4]    |  MAG_IF[3]       |

