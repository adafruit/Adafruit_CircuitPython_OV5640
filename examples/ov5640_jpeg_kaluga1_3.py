# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Jeff Epler for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

"""
The Kaluga development kit comes in two versions (v1.2 and v1.3); this demo is
tested on v1.3.

The audio board must be mounted between the Kaluga and the LCD, it provides the
I2C pull-ups(!)

You also need to place ov5640_jpeg_kaluga1_3_boot.py at CIRCUITPY/boot.py
and reset the board to make the internal flash readable by CircuitPython.
You can make CIRCUITPY readable from your PC by booting CircuitPython in
safe mode or holding the "MODE" button on the audio daughterboard while
powering on or resetting the board.
"""

import time

import board
import busio
import adafruit_ov5640
import microcontroller

try:
    with open("/boot_out.txt", "a") as f:
        pass
except OSError as e:
    print(e)
    print(
        "A 'read-only filesystem' error occurs if you did not correctly install"
        "\nov5640_jpeg_kaluga1_3_boot.py as CIRCUITPY/boot.py and reset the"
        '\nboard while holding the "mode" button'
    )
    raise SystemExit

bus = busio.I2C(scl=board.CAMERA_SIOC, sda=board.CAMERA_SIOD)
cam = adafruit_ov5640.OV5640(
    bus,
    data_pins=board.CAMERA_DATA,
    clock=board.CAMERA_PCLK,
    vsync=board.CAMERA_VSYNC,
    href=board.CAMERA_HREF,
    mclk=board.CAMERA_XCLK,
    size=adafruit_ov5640.OV5640_SIZE_SXGA,
)

cam.colorspace = adafruit_ov5640.OV5640_COLOR_JPEG
cam.quality = 5
b = bytearray(cam.capture_buffer_size)
print(f"Capturing jpeg image of up to {len(b)} bytes")
jpeg = cam.capture(b)

print(f"Captured {len(jpeg)} bytes of jpeg data")
try:
    print("Writing to internal storage (this is SLOW)")
    with open("/cam.jpg", "wb") as f:
        f.write(jpeg)
    print("Wrote to CIRCUITPY/cam.jpg")
    print("Resetting so computer sees new content of CIRCUITPY")
    time.sleep(0.5)
    microcontroller.reset()

except OSError as e:
    print(e)
    print(
        "A 'read-only filesystem' error occurs if you did not correctly install"
        "\nov5640_jpeg_kaluga1_3_boot.py as CIRCUITPY/boot.py and reset the board"
    )
