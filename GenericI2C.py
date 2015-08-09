#!/usr/bin/env python3

# By Michal Ludvig <mluvdig@logix.net.nz>

class GenericI2C(object):
    MAX_SEND_BYTES = -1

    def close(self):
        raise NotImplementedError()

    def send_bytes(self, address, data = []):
        raise NotImplementedError()

    def scan_bus(self, min_addr = 0x08, max_addr = 0x77):
        raise NotImplementedError()
