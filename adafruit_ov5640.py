# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Jeff Epler for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ov5640`
================================================================================

CircuitPython driver for OV5640 Camera


* Author(s): Jeff Epler

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies
  based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

# imports
import time
import imagecapture
import pwmio
from adafruit_bus_device.i2c_device import I2CDevice

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ov5640.git"

from micropython import const

OV5640_COLOR_RGB = 0
OV5640_COLOR_YUV = 1
OV5640_COLOR_GRAYSCALE = 2
OV5640_COLOR_JPEG = 3

# fmt: off
_SYSTEM_CTROL0 = const(0x3008)
# Bit[7]: Software reset
# Bit[6]: Software power down
# Bit[5]: Reserved
# Bit[4]: SRB clock SYNC enable
# Bit[3]: Isolation suspend select
# Bit[2:0]: Not used

_CHIP_ID_HIGH = const(0x300A)

_DRIVE_CAPABILITY = const(0x302C)
# Bit[7:6]:
#          00: 1x
#          01: 2x
#          10: 3x
#          11: 4x

_SC_PLLS_CTRL0 = const(0x303A)
# Bit[7]: PLLS bypass
_SC_PLLS_CTRL1 = const(0x303B)
# Bit[4:0]: PLLS multiplier
_SC_PLLS_CTRL2 = const(0x303C)
# Bit[6:4]: PLLS charge pump control
# Bit[3:0]: PLLS system divider
_SC_PLLS_CTRL3 = const(0x303D)
# Bit[5:4]: PLLS pre-divider
#          00: 1
#          01: 1.5
#          10: 2
#          11: 3
# Bit[2]: PLLS root-divider - 1
# Bit[1:0]: PLLS seld5
#          00: 1
#          01: 1
#          10: 2
#          11: 2.5

# AEC/AGC control functions
_AEC_PK_MANUAL = const(0x3503)
# AEC Manual Mode Control
# Bit[7:6]: Reserved
# Bit[5]: Gain delay option
#         Valid when 0x3503[4]=1’b0
#         0: Delay one frame latch
#         1: One frame latch
# Bit[4:2]: Reserved
# Bit[1]: AGC manual
#         0: Auto enable
#         1: Manual enable
# Bit[0]: AEC manual
#         0: Auto enable
#         1: Manual enable

# gain = {0x350A[1:0], 0x350B[7:0]} / 16


_X_ADDR_ST_H = const(0x3800)
# Bit[3:0]: X address start[11:8]
_X_ADDR_ST_L = const(0x3801)
# Bit[7:0]: X address start[7:0]
_Y_ADDR_ST_H = const(0x3802)
# Bit[2:0]: Y address start[10:8]
_Y_ADDR_ST_L = const(0x3803)
# Bit[7:0]: Y address start[7:0]
_X_ADDR_END_H = const(0x3804)
# Bit[3:0]: X address end[11:8]
_X_ADDR_END_L = const(0x3805)
# Bit[7:0]:
_Y_ADDR_END_H = const(0x3806)
# Bit[2:0]: Y address end[10:8]
_Y_ADDR_END_L = const(0x3807)
# Bit[7:0]:
# Size after scaling
_X_OUTPUT_SIZE_H = const(0x3808)
# Bit[3:0]: DVP output horizontal width[11:8]
_X_OUTPUT_SIZE_L = const(0x3809)
# Bit[7:0]:
_Y_OUTPUT_SIZE_H = const(0x380A)
# Bit[2:0]: DVP output vertical height[10:8]
_Y_OUTPUT_SIZE_L = const(0x380B)
# Bit[7:0]:
_X_TOTAL_SIZE_H = const(0x380C)
# Bit[3:0]: Total horizontal size[11:8]
_X_TOTAL_SIZE_L = const(0x380D)
# Bit[7:0]:
_Y_TOTAL_SIZE_H = const(0x380E)
# Bit[7:0]: Total vertical size[15:8]
_Y_TOTAL_SIZE_L = const(0x380F)
# Bit[7:0]:
_X_OFFSET_H = const(0x3810)
# Bit[3:0]: ISP horizontal offset[11:8]
_X_OFFSET_L = const(0x3811)
# Bit[7:0]:
_Y_OFFSET_H = const(0x3812)
# Bit[2:0]: ISP vertical offset[10:8]
_Y_OFFSET_L = const(0x3813)
# Bit[7:0]:
_X_INCREMENT = const(0x3814)
# Bit[7:4]: Horizontal odd subsample increment
# Bit[3:0]: Horizontal even subsample increment
_Y_INCREMENT = const(0x3815)
# Bit[7:4]: Vertical odd subsample increment
# Bit[3:0]: Vertical even subsample increment
# Size before scaling
# X_INPUT_SIZE = const(   (X_ADDR_END - X_ADDR_ST + 1 - (2 * X_OFFSET)))
# Y_INPUT_SIZE = const(   (Y_ADDR_END - Y_ADDR_ST + 1 - (2 * Y_OFFSET)))

# mirror and flip registers
_TIMING_TC_REG20 = const(0x3820)
# Timing Control Register
# Bit[2:1]: Vertical flip enable
#         00: Normal
#         11: Vertical flip
# Bit[0]: Vertical binning enable
_TIMING_TC_REG21 = const(0x3821)
# Timing Control Register
# Bit[5]: Compression Enable
# Bit[2:1]: Horizontal mirror enable
#         00: Normal
#         11: Horizontal mirror
# Bit[0]: Horizontal binning enable

_PCLK_RATIO = const(0x3824)
# Bit[4:0]: PCLK ratio manual

# frame control registers
_FRAME_CTRL01 = const(
    0x4201
)
# Control Passed Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
# Bit[7:4]: Not used
# Bit[3:0]: Frame ON number
_FRAME_CTRL02 = const(
    0x4202
)
# Control Masked Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
# Bit[7:4]: Not used
# BIT[3:0]: Frame OFF number

# format control registers
_FORMAT_CTRL00 = const(0x4300)

_CLOCK_POL_CONTROL = const(0x4740)
# Bit[5]: PCLK polarity 0: active low
#          1: active high
# Bit[3]: Gate PCLK under VSYNC
# Bit[2]: Gate PCLK under HREF
# Bit[1]: HREF polarity
#          0: active low
#          1: active high
# Bit[0] VSYNC polarity
#          0: active low
#          1: active high

_ISP_CONTROL_01 = const(0x5001)
# Bit[5]: Scale enable
#          0: Disable
#          1: Enable

# output format control registers
_FORMAT_CTRL = const(0x501F)
# Format select
# Bit[2:0]:
#  000: YUV422
#  001: RGB
#  010: Dither
#  011: RAW after DPC
#  101: RAW after CIP

# ISP top control registers
_PRE_ISP_TEST_SETTING_1 = const(0x503D)
# Bit[7]: Test enable
#         0: Test disable
#         1: Color bar enable
# Bit[6]: Rolling
# Bit[5]: Transparent
# Bit[4]: Square black and white
# Bit[3:2]: Color bar style
#         00: Standard 8 color bar
#         01: Gradual change at vertical mode 1
#         10: Gradual change at horizontal
#         11: Gradual change at vertical mode 2
# Bit[1:0]: Test select
#         00: Color bar
#         01: Random data
#         10: Square data
#         11: Black image

# exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]} / 16 × tROW

_SCALE_CTRL_1 = const(0x5601)
# Bit[6:4]: HDIV RW
#          DCW scale times
#          000: DCW 1 time
#          001: DCW 2 times
#          010: DCW 4 times
#          100: DCW 8 times
#          101: DCW 16 times
#          Others: DCW 16 times
# Bit[2:0]: VDIV RW
#          DCW scale times
#          000: DCW 1 time
#          001: DCW 2 times
#          010: DCW 4 times
#          100: DCW 8 times
#          101: DCW 16 times
#          Others: DCW 16 times

_SCALE_CTRL_2 = const(0x5602)
# X_SCALE High Bits
_SCALE_CTRL_3 = const(0x5603)
# X_SCALE Low Bits
_SCALE_CTRL_4 = const(0x5604)
# Y_SCALE High Bits
_SCALE_CTRL_5 = const(0x5605)
# Y_SCALE Low Bits
_SCALE_CTRL_6 = const(0x5606)
# Bit[3:0]: V Offset

_VFIFO_CTRL0C = const(0x460C)
# Bit[1]: PCLK manual enable
#          0: Auto
#          1: Manual by PCLK_RATIO

_VFIFO_X_SIZE_H = const(0x4602)
_VFIFO_X_SIZE_L = const(0x4603)
_VFIFO_Y_SIZE_H = const(0x4604)
_VFIFO_Y_SIZE_L = const(0x4605)

_COMPRESSION_CTRL00 = const(0x4400)
_COMPRESSION_CTRL01 = const(0x4401)
_COMPRESSION_CTRL02 = const(0x4402)
_COMPRESSION_CTRL03 = const(0x4403)
_COMPRESSION_CTRL04 = const(0x4404)
_COMPRESSION_CTRL05 = const(0x4405)
_COMPRESSION_CTRL06 = const(0x4406)
_COMPRESSION_CTRL07 = const(0x4407)
# Bit[5:0]: QS
_COMPRESSION_ISI_CTRL = const(0x4408)
_COMPRESSION_CTRL09 = const(0x4409)
_COMPRESSION_CTRL0a = const(0x440A)
_COMPRESSION_CTRL0b = const(0x440B)
_COMPRESSION_CTRL0c = const(0x440C)
_COMPRESSION_CTRL0d = const(0x440D)
_COMPRESSION_CTRL0E = const(0x440E)

_TEST_COLOR_BAR = const(0xC0)
# Enable Color Bar roling Test

_AEC_PK_MANUAL_AGC_MANUALEN = const(0x02)
# Enable AGC Manual enable
_AEC_PK_MANUAL_AEC_MANUALEN = const(0x01)
# Enable AEC Manual enable

_TIMING_TC_REG20_VFLIP = const(0x06)
# Vertical flip enable
_TIMING_TC_REG21_HMIRROR = const(0x06)
# Horizontal mirror enable

OV5640_SIZE_96X96 = 0  # 96x96
OV5640_SIZE_QQVGA = 1  # 160x120
OV5640_SIZE_QCIF = 2  # 176x144
OV5640_SIZE_HQVGA = 3  # 240x176
OV5640_SIZE_240X240 = 4  # 240x240
OV5640_SIZE_QVGA = 5  # 320x240
OV5640_SIZE_CIF = 6  # 400x296
OV5640_SIZE_HVGA = 7  # 480x320
OV5640_SIZE_VGA = 8  # 640x480
OV5640_SIZE_SVGA = 9  # 800x600
OV5640_SIZE_XGA = 10  # 1024x768
OV5640_SIZE_HD = 11  # 1280x720
OV5640_SIZE_SXGA = 12  # 1280x1024
OV5640_SIZE_UXGA = 13  # 1600x1200

_ASPECT_RATIO_4X3 = const(0)
_ASPECT_RATIO_3X2 = const(1)
_ASPECT_RATIO_16X10 = const(2)
_ASPECT_RATIO_5X3 = const(3)
_ASPECT_RATIO_16X9 = const(4)
_ASPECT_RATIO_21X9 = const(5)
_ASPECT_RATIO_5X4 = const(6)
_ASPECT_RATIO_1X1 = const(7)
_ASPECT_RATIO_9X16 = const(8)

_resolution_info = [
    [96, 96, _ASPECT_RATIO_1X1],  # 96x96
    [160, 120, _ASPECT_RATIO_4X3],  # QQVGA
    [176, 144, _ASPECT_RATIO_5X4],  # QCIF
    [240, 176, _ASPECT_RATIO_4X3],  # HQVGA
    [240, 240, _ASPECT_RATIO_1X1],  # 240x240
    [320, 240, _ASPECT_RATIO_4X3],  # QVGA
    [400, 296, _ASPECT_RATIO_4X3],  # CIF
    [480, 320, _ASPECT_RATIO_3X2],  # HVGA
    [640, 480, _ASPECT_RATIO_4X3],  # VGA
    [800, 600, _ASPECT_RATIO_4X3],  # SVGA
    [1024, 768, _ASPECT_RATIO_4X3],  # XGA
    [1280, 720, _ASPECT_RATIO_16X9],  # HD
    [1280, 1024, _ASPECT_RATIO_5X4],  # SXGA
    [1600, 1200, _ASPECT_RATIO_4X3],  # UXGA
]


_ratio_table = [
    #  mw,   mh,  sx,  sy,   ex,   ey, ox, oy,   tx,   ty
    [2560, 1920, 0, 0, 2623, 1951, 32, 16, 2844, 1968],  # 4x3
    [2560, 1704, 0, 110, 2623, 1843, 32, 16, 2844, 1752],  # 3x2
    [2560, 1600, 0, 160, 2623, 1791, 32, 16, 2844, 1648],  # 16x10
    [2560, 1536, 0, 192, 2623, 1759, 32, 16, 2844, 1584],  # 5x3
    [2560, 1440, 0, 240, 2623, 1711, 32, 16, 2844, 1488],  # 16x9
    [2560, 1080, 0, 420, 2623, 1531, 32, 16, 2844, 1128],  # 21x9
    [2400, 1920, 80, 0, 2543, 1951, 32, 16, 2684, 1968],  # 5x4
    [1920, 1920, 320, 0, 2543, 1951, 32, 16, 2684, 1968],  # 1x1
    [1088, 1920, 736, 0, 1887, 1951, 32, 16, 1884, 1968],  # 9x16
]

_REG_DLY = const(0xFFFF)
_REGLIST_TAIL = const(0x0000)

_sensor_default_regs = [
    _SYSTEM_CTROL0, 0x82,  # software reset
    _REG_DLY, 10,  # delay 10ms
    _SYSTEM_CTROL0, 0x42,  # power down
    # enable pll
    0x3103, 0x13,
    # io direction
    0x3017, 0xFF,
    0x3018, 0xFF,
    _DRIVE_CAPABILITY, 0xC3,
    _CLOCK_POL_CONTROL, 0x21,
    0x4713, 0x02,  # jpg mode select
    _ISP_CONTROL_01, 0x83,  # turn color matrix, awb and SDE
    # sys reset
    0x3000, 0x00,
    0x3002, 0x1C,
    # clock enable
    0x3004, 0xFF,
    0x3006, 0xC3,
    # isp control
    0x5000, 0xA7,
    _ISP_CONTROL_01, 0xA3,  # +scaling?
    0x5003, 0x08,  # special_effect
    # unknown
    0x370C, 0x02,  #!!IMPORTANT
    0x3634, 0x40,  #!!IMPORTANT
    # AEC/AGC
    0x3A02, 0x03,
    0x3A03, 0xD8,
    0x3A08, 0x01,
    0x3A09, 0x27,
    0x3A0A, 0x00,
    0x3A0B, 0xF6,
    0x3A0D, 0x04,
    0x3A0E, 0x03,
    0x3A0F, 0x30,  # ae_level
    0x3A10, 0x28,  # ae_level
    0x3A11, 0x60,  # ae_level
    0x3A13, 0x43,
    0x3A14, 0x03,
    0x3A15, 0xD8,
    0x3A18, 0x00,  # gainceiling
    0x3A19, 0xF8,  # gainceiling
    0x3A1B, 0x30,  # ae_level
    0x3A1E, 0x26,  # ae_level
    0x3A1F, 0x14,  # ae_level
    # vcm debug
    0x3600, 0x08,
    0x3601, 0x33,
    # 50/60Hz
    0x3C01, 0xA4,
    0x3C04, 0x28,
    0x3C05, 0x98,
    0x3C06, 0x00,
    0x3C07, 0x08,
    0x3C08, 0x00,
    0x3C09, 0x1C,
    0x3C0A, 0x9C,
    0x3C0B, 0x40,
    0x460C, 0x22,  # disable jpeg footer
    # BLC
    0x4001, 0x02,
    0x4004, 0x02,
    # AWB
    0x5180, 0xFF,
    0x5181, 0xF2,
    0x5182, 0x00,
    0x5183, 0x14,
    0x5184, 0x25,
    0x5185, 0x24,
    0x5186, 0x09,
    0x5187, 0x09,
    0x5188, 0x09,
    0x5189, 0x75,
    0x518A, 0x54,
    0x518B, 0xE0,
    0x518C, 0xB2,
    0x518D, 0x42,
    0x518E, 0x3D,
    0x518F, 0x56,
    0x5190, 0x46,
    0x5191, 0xF8,
    0x5192, 0x04,
    0x5193, 0x70,
    0x5194, 0xF0,
    0x5195, 0xF0,
    0x5196, 0x03,
    0x5197, 0x01,
    0x5198, 0x04,
    0x5199, 0x12,
    0x519A, 0x04,
    0x519B, 0x00,
    0x519C, 0x06,
    0x519D, 0x82,
    0x519E, 0x38,
    # color matrix (Saturation)
    0x5381, 0x1E,
    0x5382, 0x5B,
    0x5383, 0x08,
    0x5384, 0x0A,
    0x5385, 0x7E,
    0x5386, 0x88,
    0x5387, 0x7C,
    0x5388, 0x6C,
    0x5389, 0x10,
    0x538A, 0x01,
    0x538B, 0x98,
    # CIP control (Sharpness)
    0x5300, 0x10,  # sharpness
    0x5301, 0x10,  # sharpness
    0x5302, 0x18,  # sharpness
    0x5303, 0x19,  # sharpness
    0x5304, 0x10,
    0x5305, 0x10,
    0x5306, 0x08,  # denoise
    0x5307, 0x16,
    0x5308, 0x40,
    0x5309, 0x10,  # sharpness
    0x530A, 0x10,  # sharpness
    0x530B, 0x04,  # sharpness
    0x530C, 0x06,  # sharpness
    # GAMMA
    0x5480, 0x01,
    0x5481, 0x00,
    0x5482, 0x1E,
    0x5483, 0x3B,
    0x5484, 0x58,
    0x5485, 0x66,
    0x5486, 0x71,
    0x5487, 0x7D,
    0x5488, 0x83,
    0x5489, 0x8F,
    0x548A, 0x98,
    0x548B, 0xA6,
    0x548C, 0xB8,
    0x548D, 0xCA,
    0x548E, 0xD7,
    0x548F, 0xE3,
    0x5490, 0x1D,
    # Special Digital Effects (SDE) (UV adjust)
    0x5580, 0x06,  # enable brightness and contrast
    0x5583, 0x40,  # special_effect
    0x5584, 0x10,  # special_effect
    0x5586, 0x20,  # contrast
    0x5587, 0x00,  # brightness
    0x5588, 0x00,  # brightness
    0x5589, 0x10,
    0x558A, 0x00,
    0x558B, 0xF8,
    0x501D, 0x40,  # enable manual offset of contrast
    # power on
    0x3008, 0x02,
    # 50Hz
    0x3C00, 0x04,
    _REG_DLY, 300,
]

_sensor_fmt_jpeg = [
    _FORMAT_CTRL, 0x00,  # YUV422
    _FORMAT_CTRL00, 0x30,  # YUYV
    0x3002, 0x00,  # 0x1c to 0x00 !!!
    0x3006, 0xFF,  # 0xc3 to 0xff !!!
    0x471C, 0x50,  # 0xd0 to 0x50 !!!
]

_sensor_fmt_raw = [
    _FORMAT_CTRL, 0x03,  # RAW (DPC)
    _FORMAT_CTRL00, 0x00,  # RAW
]

_sensor_fmt_grayscale = [
    _FORMAT_CTRL, 0x00,  # YUV422
    _FORMAT_CTRL00, 0x10,  # Y8
]

_sensor_format_yuv422 = [
    _FORMAT_CTRL, 0x00,  # YUV422
    _FORMAT_CTRL00, 0x30,  # YUYV
]

_sensor_format_rgb565 = [
    _FORMAT_CTRL, 0x01,  # RGB
    _FORMAT_CTRL00, 0x61,  # RGB565 (BGR)
]

_sensor_saturation_levels = [
    [0x1D, 0x60, 0x03, 0x07, 0x48, 0x4F, 0x4B, 0x40, 0x0B, 0x01, 0x98],  # -4
    [0x1D, 0x60, 0x03, 0x08, 0x54, 0x5C, 0x58, 0x4B, 0x0D, 0x01, 0x98],  # -3
    [0x1D, 0x60, 0x03, 0x0A, 0x60, 0x6A, 0x64, 0x56, 0x0E, 0x01, 0x98],  # -2
    [0x1D, 0x60, 0x03, 0x0B, 0x6C, 0x77, 0x70, 0x60, 0x10, 0x01, 0x98],  # -1
    [0x1D, 0x60, 0x03, 0x0C, 0x78, 0x84, 0x7D, 0x6B, 0x12, 0x01, 0x98],  # 0
    [0x1D, 0x60, 0x03, 0x0D, 0x84, 0x91, 0x8A, 0x76, 0x14, 0x01, 0x98],  # +1
    [0x1D, 0x60, 0x03, 0x0E, 0x90, 0x9E, 0x96, 0x80, 0x16, 0x01, 0x98],  # +2
    [0x1D, 0x60, 0x03, 0x10, 0x9C, 0xAC, 0xA2, 0x8B, 0x17, 0x01, 0x98],  # +3
    [0x1D, 0x60, 0x03, 0x11, 0xA8, 0xB9, 0xAF, 0x96, 0x19, 0x01, 0x98],  # +4
]

_sensor_special_effects = [
    [0x06, 0x40, 0x2C, 0x08],  # Normal
    [0x46, 0x40, 0x28, 0x08],  # Negative
    [0x1E, 0x80, 0x80, 0x08],  # Grayscale
    [0x1E, 0x80, 0xC0, 0x08],  # Red Tint
    [0x1E, 0x60, 0x60, 0x08],  # Green Tint
    [0x1E, 0xA0, 0x40, 0x08],  # Blue Tint
    [0x1E, 0x40, 0xA0, 0x08],  # Sepia
]

_sensor_regs_gamma0 = [
    0x5480, 0x01,
    0x5481, 0x08,
    0x5482, 0x14,
    0x5483, 0x28,
    0x5484, 0x51,
    0x5485, 0x65,
    0x5486, 0x71,
    0x5487, 0x7D,
    0x5488, 0x87,
    0x5489, 0x91,
    0x548A, 0x9A,
    0x548B, 0xAA,
    0x548C, 0xB8,
    0x548D, 0xCD,
    0x548E, 0xDD,
    0x548F, 0xEA,
    0x5490, 0x1D,
]

sensor_regs_gamma1 = [
    0x5480, 0x1,
    0x5481, 0x0,
    0x5482, 0x1E,
    0x5483, 0x3B,
    0x5484, 0x58,
    0x5485, 0x66,
    0x5486, 0x71,
    0x5487, 0x7D,
    0x5488, 0x83,
    0x5489, 0x8F,
    0x548A, 0x98,
    0x548B, 0xA6,
    0x548C, 0xB8,
    0x548D, 0xCA,
    0x548E, 0xD7,
    0x548F, 0xE3,
    0x5490, 0x1D,
]

sensor_regs_awb0 = [
    0x5180, 0xFF,
    0x5181, 0xF2,
    0x5182, 0x00,
    0x5183, 0x14,
    0x5184, 0x25,
    0x5185, 0x24,
    0x5186, 0x09,
    0x5187, 0x09,
    0x5188, 0x09,
    0x5189, 0x75,
    0x518A, 0x54,
    0x518B, 0xE0,
    0x518C, 0xB2,
    0x518D, 0x42,
    0x518E, 0x3D,
    0x518F, 0x56,
    0x5190, 0x46,
    0x5191, 0xF8,
    0x5192, 0x04,
    0x5193, 0x70,
    0x5194, 0xF0,
    0x5195, 0xF0,
    0x5196, 0x03,
    0x5197, 0x01,
    0x5198, 0x04,
    0x5199, 0x12,
    0x519A, 0x04,
    0x519B, 0x00,
    0x519C, 0x06,
    0x519D, 0x82,
    0x519E, 0x38,
]
# fmt: on

class _RegBits:
    def __init__(self, reg, shift, mask):
        self.reg = reg
        self.shift = shift
        self.mask = mask

    def __get__(self, obj, objtype=None):
        reg_value = obj._read_register(self.reg)
        return (reg_value >> self.shift) & self.mask

    def __set__(self, obj, value):
        if value & ~self.mask:
            raise ValueError(
                f"Value 0x{value:02x} does not fit in mask 0x{self.mask:02x}"
            )
        reg_value = obj._read_register(self.reg)
        reg_value &= ~(self.mask << self.shift)
        reg_value |= value << self.shift
        obj._write_register(self.reg, reg_value)

class _RegBits16:
    def __init__(self, reg, shift, mask):
        self.reg = reg
        self.shift = shift
        self.mask = mask

    def __get__(self, obj, objtype=None):
        reg_value = obj._read_register16(self.reg)
        return (reg_value >> self.shift) & self.mask

    def __set__(self, obj, value):
        if value & ~self.mask:
            raise ValueError(
                f"Value 0x{value:02x} does not fit in mask 0x{self.mask:02x}"
            )
        reg_value = obj._read_register16(self.reg)
        reg_value &= ~(self.mask << self.shift)
        reg_value |= value << self.shift
        obj._write_register16(self.reg, reg_value)


class _SCCB16CameraBase:  # pylint: disable=too-few-public-methods
    def __init__(self, i2c_bus, i2c_address):
        self._i2c_device = I2CDevice(i2c_bus, i2c_address)
        self._bank = None

    def _write_register(self, reg, value):
        b = bytearray(3)
        b[0] = reg >> 8
        b[1] = reg & 0xff
        b[1] = value
        with self._i2c_device as i2c:
            i2c.write(b)

    def _write_register16(self, reg, value):
        self._write_register(reg, value >> 8)
        self._write_register(reg+1, value & 0xff)

    def _read_register(self, reg):
        b = bytearray(2)
        b[0] = reg >> 8
        b[1] = reg & 0xff
        with self._i2c_device as i2c:
            i2c.write(b)
            i2c.readinto(b, end=1)
        return b[0]

    def _read_register16(self, reg):
        hi = self._read_register(reg)
        lo = self._read_register(reg+1)
        return (hi << 8) | lo

    def _write_list(self, reg_list):
        for i in range(0, len(reg_list), 2):
            register = reg_list[i]
            value = reg_list[i+1]
            if register == _REG_DLY:
                time.sleep(value / 1000)
            else:
                self._write_register(register, value)


class OV5640(_SCCB16CameraBase):  # pylint: disable=too-many-instance-attributes
    def __init__(
        self,
        i2c_bus,
        data_pins,
        clock,
        vsync,
        href,
        shutdown=None,
        reset=None,
        mclk=None,
        mclk_frequency=24_000_000,
        i2c_address=0x3c,
        size=OV5640_SIZE_QQVGA,
    ):  # pylint: disable=too-many-arguments
        """
        Args:
            i2c_bus (busio.I2C): The I2C bus used to configure the OV5640
            data_pins (List[microcontroller.Pin]): A list of 8 data pins, in order.
            clock (microcontroller.Pin): The pixel clock from the OV5640.
            vsync (microcontroller.Pin): The vsync signal from the OV5640.
            href (microcontroller.Pin): The href signal from the OV5640, \
                sometimes inaccurately called hsync.
            shutdown (Optional[microcontroller.Pin]): If not None, the shutdown
                signal to the camera, also called the powerdown or enable pin.
            reset (Optional[microcontroller.Pin]): If not None, the reset signal
                to the camera.
            mclk (Optional[microcontroller.Pin]): The pin on which to create a
                master clock signal, or None if the master clock signal is
                already being generated.
            mclk_frequency (int): The frequency of the master clock to generate, \
                ignored if mclk is None, requred if it is specified
            i2c_address (int): The I2C address of the camera.
        """

        # Initialize the master clock
        if mclk:
            self._mclk_pwm = pwmio.PWMOut(mclk, frequency=mclk_frequency)
            self._mclk_pwm.duty_cycle = 32768
        else:
            self._mclk_pwm = None

        if shutdown:
            self._shutdown = digitalio.DigitalInOut(shutdown)
            self._shutdown.switch_to_output(True)
            time.sleep(0.1)
            self._shutdown.switch_to_output(False)
            time.sleep(0.3)
        else:
            self._shutdown = None

        if reset:
            self._reset = digitalio.DigitalInOut(reset)
            self._reset.switch_to_output(False)
            time.sleep(0.1)
            self._reset.switch_to_output(True)
            time.sleep(0.1)
        else:
            self._reset = None

        # Now that the master clock is running, we can initialize i2c comms
        super().__init__(i2c_bus, i2c_address)

        self._write_list(_sensor_default_regs)

        self._imagecapture = imagecapture.ParallelImageCapture(
            data_pins=data_pins, clock=clock, vsync=vsync, href=href
        )

        return

        self._colorspace = OV5640_COLOR_RGB
        self._w = None
        self._h = None
        self._size = None
        self._test_pattern = False
        self.size = size

        self._flip_x = False
        self._flip_y = False

        self.gain_ceiling = _COM9_AGC_GAIN_2x
        self.bpc = False
        self.wpc = True
        self.lenc = True

        # self._sensor_init()

    chip_id = _RegBits16(_CHIP_ID_HIGH, 0, 0xffff)

    def capture(self, buf):
        """Capture an image into the buffer.

        Args:
            buf (Union[bytearray, memoryview]): A WritableBuffer to contain the \
                captured image.  Note that this can be a ulab array or a displayio Bitmap.
        """
        self._imagecapture.capture(buf)
        if self.colorspace == OV5640_COLOR_JPEG:
            eoi = buf.find(b"\xff\xd9")
            if eoi != -1:
                # terminate the JPEG data just after the EOI marker
                return memoryview(buf)[: eoi + 2]
        return None


    @property
    def capture_buffer_size(self):
        """Return the size of capture buffer to use with current resolution & colorspace settings"""
        if self.colorspace == OV5640_COLOR_JPEG:
            return self.width * self.height // 5
        return self.width * self.height * 2

    @property
    def mclk_frequency(self):
        """Get the actual frequency the generated mclk, or None"""
        return self._mclk_pwm.frequency if self._mclk_pwm else None

    @property
    def width(self):
        """Get the image width in pixels.  A buffer of 2*width*height bytes \
        stores a whole image."""
        return self._w

    @property
    def height(self):
        """Get the image height in pixels.  A buffer of 2*width*height bytes \
        stores a whole image."""
        return self._h

    @property
    def colorspace(self):
        """Get or set the colorspace, one of the ``OV5640_COLOR_`` constants."""
        return self._colorspace

    @colorspace.setter
    def colorspace(self, colorspace):
        self._colorspace = colorspace
        self._set_size_and_colorspace()

    def _set_colorspace(self):
        colorspace = self._colorspace
        settings = _ov5640_color_settings[colorspace]

        self._write_list(settings)
        # written twice?
        self._write_list(settings)
        time.sleep(0.01)

    def deinit(self):
        """Deinitialize the camera"""
        self._imagecapture.deinit()
        if self._mclk_pwm:
            self._mclk_pwm.deinit()
        if self._shutdown:
            self._shutdown.deinit()
        if self._reset:
            self._reset.deinit()


    @property
    def size(self):
        """Get or set the captured image size, one of the ``OV5640_SIZE_`` constants."""
        return self._size

    def _set_size_and_colorspace(self):
        size = self._size
        width, height, ratio = _resolution_info[size]
        offset_x, offset_y, max_x, max_y = _ratio_table[ratio]
        mode = _OV5640_MODE_UXGA
        if size <= OV5640_SIZE_CIF:
            mode = _OV5640_MODE_CIF
            max_x //= 4
            max_y //= 4
            offset_x //= 4
            offset_y //= 4
            if max_y > 296:
                max_y = 296

        elif size <= OV5640_SIZE_SVGA:
            mode = _OV5640_MODE_SVGA
            max_x //= 2
            max_y //= 2
            offset_x //= 2
            offset_y //= 2

        self._set_window(mode, offset_x, offset_y, max_x, max_y, width, height)

    @size.setter
    def size(self, size):
        self._size = size
        self._set_size_and_colorspace()

    def _set_flip(self):
        bits = 0
        if self._flip_x:
            bits |= _REG04_HFLIP_IMG
        if self._flip_y:
            bits |= _REG04_VFLIP_IMG | _REG04_VREF_EN
        self._write_bank_register(_BANK_SENSOR, _REG04, _REG04_SET(bits))

    @property
    def flip_x(self):
        """Get or set the X-flip flag"""
        return self._flip_x

    @flip_x.setter
    def flip_x(self, value):
        self._flip_x = bool(value)
        self._set_flip()

    @property
    def flip_y(self):
        """Get or set the Y-flip flag"""
        return self._flip_y

    @flip_y.setter
    def flip_y(self, value):
        self._flip_y = bool(value)
        self._set_flip()

    @property
    def product_id(self):
        """Get the product id (PID) register.  The expected value is 0x26."""
        return self._read_bank_register(_BANK_SENSOR, _REG_PID)

    @property
    def product_version(self):
        """Get the version (VER) register.  The expected value is 0x4x."""
        return self._read_bank_register(_BANK_SENSOR, _REG_VER)

    def _set_window(
        self, mode, offset_x, offset_y, max_x, max_y, width, height
    ):  # pylint: disable=too-many-arguments, too-many-locals
        self._w = width
        self._h = height

        max_x //= 4
        max_y //= 4
        width //= 4
        height //= 4

        win_regs = [
            _BANK_SEL,
            _BANK_DSP,
            _HSIZE,
            max_x & 0xFF,
            _VSIZE,
            max_y & 0xFF,
            _XOFFL,
            offset_x & 0xFF,
            _YOFFL,
            offset_y & 0xFF,
            _VHYX,
            ((max_y >> 1) & 0x80)
            | ((offset_y >> 4) & 0x70)
            | ((max_x >> 5) & 0x08)
            | ((offset_y >> 8) & 0x07),
            _TEST,
            (max_x >> 2) & 0x80,
            _ZMOW,
            (width) & 0xFF,
            _ZMOH,
            (height) & 0xFF,
            _ZMHH,
            ((height >> 6) & 0x04) | ((width >> 8) & 0x03),
        ]

        pclk_auto = 0
        pclk_div = 8
        clk_2x = 0
        clk_div = 0

        if self._colorspace != OV5640_COLOR_JPEG:
            pclk_auto = 1
            clk_div = 7

        if mode == _OV5640_MODE_CIF:
            regs = _ov5640_settings_to_cif
            if self._colorspace != OV5640_COLOR_JPEG:
                clk_div = 3
        elif mode == _OV5640_MODE_SVGA:
            regs = _ov5640_settings_to_svga
        else:
            regs = _ov5640_settings_to_uxga
            pclk_div = 12

        clk = clk_div | (clk_2x << 7)
        pclk = pclk_div | (pclk_auto << 7)

        self._write_bank_register(_BANK_DSP, _R_BYPASS, _R_BYPASS_DSP_BYPAS)
        self._write_list(regs)
        self._write_list(win_regs)
        self._write_bank_register(_BANK_SENSOR, _CLKRC, clk)

        self._write_list(win_regs)
        self._write_bank_register(_BANK_SENSOR, _CLKRC, clk)
        self._write_bank_register(_BANK_DSP, _R_DVP_SP, pclk)
        self._write_register(_R_BYPASS, _R_BYPASS_DSP_EN)
        time.sleep(0.01)

        # Reestablish colorspace
        self._set_colorspace()

        # Reestablish test pattern
        if self._test_pattern:
            self.test_pattern = self._test_pattern

    @property
    def exposure(self):
        """The exposure level of the sensor"""
        aec_9_2 = self._get_reg_bits(_BANK_SENSOR, _AEC, 0, 0xFF)
        aec_15_10 = self._get_reg_bits(_BANK_SENSOR, _REG45, 0, 0b111111)
        aec_1_0 = self._get_reg_bits(_BANK_SENSOR, _REG04, 0, 0b11)

        return aec_1_0 | (aec_9_2 << 2) | (aec_15_10 << 10)

    @exposure.setter
    def exposure(self, exposure):
        aec_1_0 = exposure & 0x11
        aec_9_2 = (exposure >> 2) & 0b11111111
        aec_15_10 = exposure >> 10

        self._set_reg_bits(_BANK_SENSOR, _AEC, 0, 0xFF, aec_9_2)
        self._set_reg_bits(_BANK_SENSOR, _REG45, 0, 0b111111, aec_15_10)
        self._set_reg_bits(_BANK_SENSOR, _REG04, 0, 0b11, aec_1_0)

