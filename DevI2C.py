#!/usr/bin/env python3

# Python wrapper for /dev/i2c-* devices
# By Michal Ludvig <mludvig@logix.net.nz>
# License GPLv3

import smbus
from pYnterface.GenericI2C import GenericI2C

__all__ = [ "DevI2C" ]

class DevI2C(GenericI2C):
    MAX_SEND_BYTES = 32

    def __init__(self, i2c_id):
        self.bus = smbus.SMBus(i2c_id)

    def close(self):
        self.bus.close()

    def send_bytes(self, address, data = []):
        self.bus.write_i2c_block_data(address, data[0], data[1:])

    def scan_bus(self, min_addr = 0x08, max_addr = 0x77):
        devices = []
        for addr in range(min_addr, max_addr + 1):
            try:
                self.bus.read_byte(addr)   # throws IOError if addr is invalid
                devices.append(addr)
            except IOError as e:
                pass
        return devices

if __name__ == "__main__":
    i2c_id = 1  # /dev/i2c-<id>
    i2c = DevI2C(i2c_id = i2c_id)
    print("Opened bus: i2c-%d" % i2c_id)
    devices = i2c.scan_bus()
    print("Found I2C devices: %s" % (devices and " ".join([hex(ch) for ch in devices]) or "None"))
    i2c.close()
