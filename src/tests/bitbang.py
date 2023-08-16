# Test using general purpose I/O pins as I2C bus
import board
import adafruit_bitbangio as bitbangio

i2c = bitbangio.I2C(scl=board.GPIO17, sda=board.GPIO27, frequency=400000)

while not i2c.try_lock():
    pass

print(i2c.scan())
i2c.deinit()