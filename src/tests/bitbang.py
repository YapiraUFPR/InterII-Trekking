# Test using general purpose I/O pins as I2C bus

import bitbangio
import board

i2c = bitbangio.I2C(board.GPIO17, board.GPIO27, frequency=100000)
print(i2c.scan())
i2c.deinit()