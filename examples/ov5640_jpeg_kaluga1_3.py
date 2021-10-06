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

import board
import busio
import adafruit_ov5640


bus = busio.I2C(scl=board.CAMERA_SIOC, sda=board.CAMERA_SIOD)
cam = adafruit_ov5640.OV5640(
    bus,
    data_pins=board.CAMERA_DATA,
    clock=board.CAMERA_PCLK,
    vsync=board.CAMERA_VSYNC,
    href=board.CAMERA_HREF,
    mclk=board.CAMERA_XCLK,
    mclk_frequency=20_000_000,
    size=adafruit_ov5640.OV5640_SIZE_QVGA,
)

cam.colorspace = adafruit_ov5640.OV5640_COLOR_JPEG
b = bytearray(cam.capture_buffer_size)
jpeg = cam.capture(b)

print(f"Captured {len(jpeg)} bytes of jpeg data")
try:
    with open("/jpeg.jpg", "wb") as f:
        f.write(jpeg)
except OSError as e:
    print(e)
    print(
        "A 'read-only filesystem' error occurs if you did not correctly install"
        "\nov5640_jpeg_kaluga1_3_boot.py as CIRCUITPY/boot.py and reset the board"
    )
print("Wrote to CIRCUITPY/jpeg.jpg")
