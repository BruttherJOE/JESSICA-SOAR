#!/usr/bin/env python
import time
from niryo_one_python_api.niryo_one_api import *

SENSOR_PIN = GPIO_1A
LED_PIN = GPIO_2A

n = NiryoOne()
n.pin_mode(SENSOR, PIN_MODE_INPUT)
n.pin_mode(LED, PIN_MODE_OUTPUT)

if __name__ == '__main__':

    while True:
        val = n.digital_read(SENSOR_PIN)
        print(val)

        n.digital_write(LED_PIN, val)
        time.sleep(0.1)

