#!/usr/bin/env python3

# Maxim Integrated DS2482-100 Single-channel 1-Wire Master (I2C)

import smbus

class DS2482(object):
    REG_STATUS  = 0xF0
    REG_DATA    = 0xE1
    REG_CONFIG  = 0xC3

    CFG_BIT_1WB = 1 << 0    # 1-Wire Busy
    CFG_BIT_PPD = 1 << 1    # Presence-Pulse Detect
    CFG_BIT_SD  = 1 << 2    # Short Detected
    CFG_BIT_LL  = 1 << 3    # Logic Level
    CFG_BIT_RST = 1 << 4    # DS2482 has been reset
    CFG_BIT_SBR = 1 << 5    # Single Bit Result
    CFG_BIT_TSB = 1 << 6    # Triplet Second Bit
    CFG_BIT_DIR = 1 << 7    # Branch Direction Taken

    CMD_DEVICE_RESET    = 0xF0  # Param: None
    CMD_SET_READ_REG    = 0xE1  # Param: registed id (REG_* above)
    CMD_WRITE_CONFIG    = 0xD2  # Param: config byte
    CMD_1W_RESET        = 0xB4  # Param: None
    CMD_1W_SINGLE_BIT   = 0x87  # Param: Bit Byte ([0b1 or 0b0]<<7)
    CMD_1W_WRITE_BYTE   = 0xA5  # Param: Data byte
    CMD_1W_READ_BYTE    = 0x96  # Param: None (result in REG_DATA)
    CMD_1W_TRIPLET      = 0x78  # Param: Direction Bit Byte ([0b1 or 0b0]<<7)

    OW_SEARCH_ALL   = 0xF0
    OW_SEARCH_ALARM = 0xEC
    OW_READ_ROM     = 0x33

    def __init__(self, bus = None, i2c_id = 0, address = 0x18):
        """
        DS2482(bus = None, i2c_id = 0, address = 0x4D)

        Either call with pre-initialised smbus(-compatible) instance in 'bus'
        Or with /dev/i2c-<id> device id in 'i2c_id' to open a new bus instance

        Usual MCP980x-family addresses are 0x48 ~ 0x4D
        """
        self.address = address
        self._last_register = None
        if bus:
            self.bus = bus
        else:
            self.bus = smbus.SMBus(i2c_id)

    def read_register(self, register):
        """
        read_register(register)

        Read and return one byte from 'register'.
        """
        # Don't set register pointer if reading from the same register again
        if self._last_register != register:
            self.bus.write_byte_data(self.address, self.CMD_SET_READ_REG, register)
            self._last_register = register
        return self.bus.read_byte(self.address)

    def wait_busy(self):
        # Wait until REG_CONFIG.1WB bit clears
        busy = True
        while busy:
            busy = self.read_register(self.REG_STATUS) & self.CFG_BIT_1WB

    def reset_1wire(self, wait_busy = True):
        self.bus.write_byte(self.address, self.CMD_1W_RESET)
        if wait_busy:
            self.wait_busy()

    def write_1wire_byte(self, data, wait_busy = True):
        self.bus.write_byte_data(self.address, self.CMD_1W_WRITE_BYTE, data)
        if wait_busy:
            self.wait_busy()

    def read_1wire_byte(self):
        self.bus.write_byte(self.address, self.CMD_1W_READ_BYTE)
        self.wait_busy()
        return self.read_register(self.REG_DATA)

    def triplet(self, search_bit):
        self.bus.write_byte_data(self.address, self.CMD_1W_TRIPLET, (search_bit << 7))
        self.wait_busy()

    def search_bus(self, search_alarm = False):
        # Implementation of 1-Wire Search Algorithm
        # from Maxim Integrated Application note 187
        # http://www.maximintegrated.com/en/app-notes/index.mvp/id/187

        devices = []

        last_discrepancy = 0
        last_device = False
        last_rom_no_bin = 0     # big-endian rom number

        search_command = search_alarm and self.OW_SEARCH_ALARM or self.OW_SEARCH_ALL

        while not last_device:
            rom_no_bin = 0

            last_zero = 0
            rom_no = []
            octet = 0x00
            id_bit = cmp_id_bit = dir_taken = 0

            self.reset_1wire()
            self.write_1wire_byte(search_command)

            for id_bit_number in range(1, 64 + 1):

                # Determine search_direction
                if id_bit_number == last_discrepancy:
                    search_direction = 1
                elif id_bit_number > last_discrepancy:
                    search_direction = 0
                else:
                    search_direction = (last_rom_no_bin >> (id_bit_number-1)) & 0x01

                triplet_ret = self.triplet(search_direction)
                status = self.read_register(self.REG_STATUS)   # Read previously set REG_STATUS
                id_bit = (status & self.CFG_BIT_SBR) >> 5
                cmp_id_bit = (status & self.CFG_BIT_TSB) >> 6
                dir_taken = (status & self.CFG_BIT_DIR) >> 7

                if id_bit == cmp_id_bit == 1:
                    # Error or no devices on the bus
                    last_device = True
                    break

                #is_discrepancy = (id_bit == cmp_id_bit == 0)
                #if is_discrepancy:
                #    print("%02d> %d : %d %d %d %s ld=%d" % (id_bit_number, search_direction,
                #        id_bit, cmp_id_bit, dir_taken,
                #        (is_discrepancy and "*" or ""),
                #        last_discrepancy))

                rom_no_bin |= (dir_taken << (id_bit_number-1))
                octet |= (dir_taken << ((id_bit_number-1) % 8))
                if (id_bit_number-1) % 8 == 7:
                    #print("=> 0x%02X" % octet)
                    rom_no.append(octet)
                    octet = 0x00

                if id_bit == cmp_id_bit == dir_taken == 0:
                    last_zero = id_bit_number

            last_discrepancy = last_zero
            last_device = (last_zero == 0)
            last_rom_no_bin = rom_no_bin
            #print("**** Device: %s" % ":".join(["%02X" % x for x in rom_no]))
            devices.append(rom_no)
        return devices

if __name__ == "__main__":
    import time
    ow = DS2482(i2c_id = 1, address = 0x18)
    print("DS2482 Status: 0x%02X" % ow.read_register(DS2482.REG_STATUS))
    print("DS2482 Config: 0x%02X" % ow.read_register(DS2482.REG_CONFIG))

    devices = ow.search_bus()
    for device in devices:
            print("Device: %s" % ":".join(["%02X" % x for x in device]))
