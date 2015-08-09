#!/usr/bin/env python3

# Microchip MCP9800 Temperature Sensor (I2C)
# By Michal Ludvig <mludvig@logix.net.nz>

import smbus

__all__ = [ "MCP9800" ]

class MCP9800(object):

    REG_TEMP            = 0x00  # Temperature register
    REG_CONFIG          = 0x01  # Config register
    REG_HYSTERISIS      = 0x02  # Temperature Hysteresis register
    REG_LIMIT           = 0x03  # Temperature Limit-set register

    # Config register shifts, e.g. 12 bit resolution: CONFIG_VAL |= (0b11 << SHIFT_ADC_RES)
    SHIFT_ONE_SHOT      = 7     # One shot, 1=enabled, 0=disabled (default)
    SHIFT_ADC_RES       = 5     # ADC resolution (0b00 = 9bit , 0b01 = 10bit (0.25C), 0b10 = 11bit (0.125C), 0b11 = 12bit (0.0625C))
    SHIFT_FAULT_QUEUE   = 3     # Fault queue bits, 0b00 = 1 (default), 0b01 = 2, 0b10 = 4, 0b11 = 6
    SHIFT_ALERT_POLARITY= 2     # Alert polarity, 1= active high, 0 = active low (default)
    SHIFT_COMP_INTR     = 1     # 1 = Interrupt mode, 0 = Comparator mode (default)
    SHIFT_SHUTDOWN      = 0     # 1 = Enable shutdown, 0 = Disable shutdown (default)

    def __init__(self, i2c_id = 0, address = 0x4D):
        self.address = address
        self.bus = smbus.SMBus(i2c_id)

    def read_register(self, register, length):
        data = self.bus.read_i2c_block_data(self.address, register, length)
        return data

    def read_temperature(self):
        temp = self.read_register(self.REG_TEMP, 2)
        if len(temp) < 2:
            return None
        return float((temp[0]<<8) + temp[1])/(2**8)

    def set_resolution(self, bit_resolution):
        assert(bit_resolution >= 9 and bit_resolution <= 12)
        config = self.bus.read_byte_data(self.address, self.REG_CONFIG)
        config &= ~(0b11 << self.SHIFT_ADC_RES)
        config |= ((int(bit_resolution) - 9) << self.SHIFT_ADC_RES)
        self.bus.write_byte_data(self.address, self.REG_CONFIG, config)

if __name__ == "__main__":
    # Initialise I2C
    mcp = MCP9800(1, 0x4D)

    # 12-bit resolution
    mcp.set_resolution(12)

    data = mcp.read_register(MCP9800.REG_CONFIG, 1)
    print("CONF: 0x%02X" % (data[0]))

    print("Temperature: %.1f" % mcp.read_temperature())
