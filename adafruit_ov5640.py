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

* ESP32-S2 Kaluga Dev Kit featuring ESP32-S2 WROVER <https://www.adafruit.com/product/4729>

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
"""

# pylint: disable=too-many-lines

# imports
import time
import imagecapture
import pwmio
import digitalio
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

_STROBE_CTRL = const(0x3b00)
_FREX_MODE = const(0x3b07)
_FREX_REQUEST = const(0x3B08)
_PAD_OUTPUT_ENABLE00 = const(0x3016)
_PAD_OUTPUT_VALUE00 = const(0x3019)
_PAD_SELECT00 = const(0x301C)

FREX_MODE_0 = 1
FREX_MODE_1 = 2
FREX_MODE_ROLL = 3

STROBE_MODE_XENON = 0
STROBE_MODE_LED1 = 1
STROBE_MODE_LED2 = 2
STROBE_MODE_LED3 = 3

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
# Control Passed Frame Number When both ON and OFF number set to 0x00,frame
# control is in bypass mode
# Bit[7:4]: Not used
# Bit[3:0]: Frame ON number
_FRAME_CTRL02 = const(
    0x4202
)
# Control Masked Frame Number When both ON and OFF number set to 0x00,frame
# control is in bypass mode
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
_COMPRESSION_CTRL0A = const(0x440A)
_COMPRESSION_CTRL0B = const(0x440B)
_COMPRESSION_CTRL0C = const(0x440C)
_COMPRESSION_CTRL0D = const(0x440D)
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
OV5640_SIZE_QHDA = 14  # 2560x1440
OV5640_SIZE_WQXGA = 15  # 2560x1600
OV5640_SIZE_PFHD = 16  # 1088x1920
OV5640_SIZE_QSXGA = 17  # 2560x1920

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
    [2560, 1440, _ASPECT_RATIO_16X9], # QHD
    [2560, 1600, _ASPECT_RATIO_16X10], # WQXGA
    [1088, 1920, _ASPECT_RATIO_9X16], # Portrait FHD
    [2560, 1920, _ASPECT_RATIO_4X3], # QSXGA

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

_pll_pre_div2x_factors = [1, 1, 2, 3, 4, 1.5, 6, 2.5, 8]
_pll_pclk_root_div_factors = [1,2,4,8]

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

_sensor_format_jpeg = [
    _FORMAT_CTRL, 0x00,  # YUV422
    _FORMAT_CTRL00, 0x30,  # YUYV
    0x3002, 0x00,  # 0x1c to 0x00 !!!
    0x3006, 0xFF,  # 0xc3 to 0xff !!!
    0x471C, 0x50,  # 0xd0 to 0x50 !!!
]

_sensor_format_raw = [
    _FORMAT_CTRL, 0x03,  # RAW (DPC)
    _FORMAT_CTRL00, 0x00,  # RAW
]

_sensor_format_grayscale = [
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

_ov5640_color_settings = {
    OV5640_COLOR_RGB: _sensor_format_rgb565,
    OV5640_COLOR_YUV: _sensor_format_yuv422,
    OV5640_COLOR_GRAYSCALE: _sensor_format_grayscale,
    OV5640_COLOR_JPEG: _sensor_format_jpeg,
}

_contrast_settings = [
    [0x20, 0x00], #  0
    [0x24, 0x10], # +1
    [0x28, 0x18], # +2
    [0x2c, 0x1c], # +3
    [0x14, 0x14], # -3
    [0x18, 0x18], # -2
    [0x1c, 0x1c], # -1
]

_sensor_saturation_levels = [
    [0x1D, 0x60, 0x03, 0x0C, 0x78, 0x84, 0x7D, 0x6B, 0x12, 0x01, 0x98],  # 0
    [0x1D, 0x60, 0x03, 0x0D, 0x84, 0x91, 0x8A, 0x76, 0x14, 0x01, 0x98],  # +1
    [0x1D, 0x60, 0x03, 0x0E, 0x90, 0x9E, 0x96, 0x80, 0x16, 0x01, 0x98],  # +2
    [0x1D, 0x60, 0x03, 0x10, 0x9C, 0xAC, 0xA2, 0x8B, 0x17, 0x01, 0x98],  # +3
    [0x1D, 0x60, 0x03, 0x11, 0xA8, 0xB9, 0xAF, 0x96, 0x19, 0x01, 0x98],  # +4
    [0x1D, 0x60, 0x03, 0x07, 0x48, 0x4F, 0x4B, 0x40, 0x0B, 0x01, 0x98],  # -4
    [0x1D, 0x60, 0x03, 0x08, 0x54, 0x5C, 0x58, 0x4B, 0x0D, 0x01, 0x98],  # -3
    [0x1D, 0x60, 0x03, 0x0A, 0x60, 0x6A, 0x64, 0x56, 0x0E, 0x01, 0x98],  # -2
    [0x1D, 0x60, 0x03, 0x0B, 0x6C, 0x77, 0x70, 0x60, 0x10, 0x01, 0x98],  # -1
]

_sensor_ev_levels = [
    [0x38, 0x30, 0x61, 0x38, 0x30, 0x10], #  0
    [0x40, 0x38, 0x71, 0x40, 0x38, 0x10], # +1
    [0x50, 0x48, 0x90, 0x50, 0x48, 0x20], # +2
    [0x60, 0x58, 0xa0, 0x60, 0x58, 0x20], # +3
    [0x10, 0x08, 0x10, 0x08, 0x20, 0x10], # -3
    [0x20, 0x18, 0x41, 0x20, 0x18, 0x10], # -2
    [0x30, 0x28, 0x61, 0x30, 0x28, 0x10], # -1
]

OV5640_WHITE_BALANCE_AUTO = 0
OV5640_WHITE_BALANCE_SUNNY = 1
OV5640_WHITE_BALANCE_FLUORESCENT = 2
OV5640_WHITE_BALANCE_CLOUDY = 3
OV5640_WHITE_BALANCE_INCANDESCENT = 4

_light_registers = [0x3406, 0x3400, 0x3401, 0x3402, 0x3403, 0x3404, 0x3405]
_light_modes = [
    [0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00], # auto
    [0x01, 0x06, 0x1c, 0x04, 0x00, 0x04, 0xf3], # sunny
    [0x01, 0x05, 0x48, 0x04, 0x00, 0x07, 0xcf], # office / fluorescent
    [0x01, 0x06, 0x48, 0x04, 0x00, 0x04, 0xd3], # cloudy
    [0x01, 0x04, 0x10, 0x04, 0x00, 0x08, 0x40], # home / incandescent

]

OV5640_SPECIAL_EFFECT_NONE = 0
OV5640_SPECIAL_EFFECT_NEGATIVE = 1
OV5640_SPECIAL_EFFECT_GRAYSCALE = 2
OV5640_SPECIAL_EFFECT_RED_TINT = 3
OV5640_SPECIAL_EFFECT_GREEN_TINT = 4
OV5640_SPECIAL_EFFECT_BLUE_TINT = 5
OV5640_SPECIAL_EFFECT_SEPIA = 6

_sensor_special_effects = [
    [0x06, 0x40, 0x10, 0x08],  # Normal
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

_autofocus_firmware_load = (
	0x3022, 0x00,
	0x3023, 0x00,
	0x3024, 0x00,
	0x3025, 0x00,
	0x3026, 0x00,
	0x3027, 0x00,
	0x3028, 0x00,
	0x3029, 0x7f,
	0x3000, 0x00, 
)

AUTOFOCUS_STAT_FIRMWAREBAD = 0x7F
AUTOFOCUS_STAT_STARTUP = 0x7E
AUTOFOCUS_STAT_IDLE = 0x70
AUTOFOCUS_STAT_FOCUSING = 0x00
AUTOFOCUS_STAT_FOCUSED = 0x10

autofocus_firmware = b'\x02\x0f\xd6\x02\n9\xc2\x01""\x00\x02\x0f\xb2\xe5\x1fpr\xf5\x1e\xd25\xff\xef%\xe0$N\xf8\xe4\xf6\x08\xf6\x0f\xbf4\xf2\x90\x0e\x93\xe4\x93\xff\xe5K\xc3\x9fP\x04\x7f\x05\x80\x02\x7f\xfbx\xbd\xa6\x07\x12\x0f\x04@\x04\x7f\x03\x80\x02\x7f0x\xbc\xa6\x07\xe6\x18\xf6\x08\xe6x\xb9\xf6x\xbc\xe6x\xba\xf6x\xbfv3\xe4\x08\xf6x\xb8v\x01uJ\x02x\xb6\xf6\x08\xf6t\xffx\xc1\xf6\x08\xf6u\x1f\x01x\xbc\xe6u\xf0\x05\xa4\xf5K\x12\n\xff\xc27"x\xb8\xe6\xd3\x94\x00@\x02\x16"\xe5\x1f\xb4\x05#\xe4\xf5\x1f\xc2\x01x\xb6\xe6\xfe\x08\xe6\xffxN\xa6\x06\x08\xa6\x07\xa27\xe43\xf5<\x900(\xf0u\x1e\x10\xd25"\xe5Ku\xf0\x05\x84x\xbc\xf6\x90\x0e\x8c\xe4\x93\xff%\xe0$\n\xf8\xe6\xfc\x08\xe6\xfdx\xbc\xe6%\xe0$N\xf8\xa6\x04\x08\xa6\x05\xef\x12\x0f\x0b\xd3x\xb7\x96\xee\x18\x96@\rx\xbc\xe6x\xb9\xf6x\xb6\xa6\x06\x08\xa6\x07\x90\x0e\x8c\xe4\x93\x12\x0f\x0b\xc3x\xc2\x96\xee\x18\x96P\rx\xbc\xe6x\xba\xf6x\xc1\xa6\x06\x08\xa6\x07x\xb6\xe6\xfe\x08\xe6\xc3x\xc2\x96\xff\xee\x18\x96x\xc3\xf6\x08\xa6\x07\x90\x0e\x95\xe4\x18\x12\x0e\xe9@\x02\xd27x\xbc\xe6\x08&\x08\xf6\xe5\x1fd\x01pJ\xe6\xc3x\xc0\x12\x0e\xdf@\x05\x12\x0e\xda@9\x12\x0f\x02@\x04\x7f\xfe\x80\x02\x7f\x02x\xbd\xa6\x07x\xb9\xe6$\x03x\xbf\xf6x\xb9\xe6$\xfdx\xc0\xf6\x12\x0f\x02@\x06x\xc0\xe6\xff\x80\x04x\xbf\xe6\xffx\xbe\xa6\x07u\x1f\x02x\xb8v\x01\x02\x02J\xe5\x1fd\x02`\x03\x02\x02*x\xbe\xe6\xff\xc3x\xc0\x12\x0e\xe0@\x08\x12\x0e\xdaP\x03\x02\x02(\x12\x0f\x02@\x04\x7f\xff\x80\x02\x7f\x01x\xbd\xa6\x07x\xb9\xe6\x04x\xbf\xf6x\xb9\xe6\x14x\xc0\xf6\x18\x12\x0f\x04@\x04\xe6\xff\x80\x02\x7f\x00x\xbf\xa6\x07\xd3\x08\xe6d\x80\x94\x80@\x04\xe6\xff\x80\x02\x7f\x00x\xc0\xa6\x07\xc3\x18\xe6d\x80\x94\xb3P\x04\xe6\xff\x80\x02\x7f3x\xbf\xa6\x07\xc3\x08\xe6d\x80\x94\xb3P\x04\xe6\xff\x80\x02\x7f3x\xc0\xa6\x07\x12\x0f\x02@\x06x\xc0\xe6\xff\x80\x04x\xbf\xe6\xffx\xbe\xa6\x07u\x1f\x03x\xb8v\x01\x80 \xe5\x1fd\x03p&x\xbe\xe6\xff\xc3x\xc0\x12\x0e\xe0@\x05\x12\x0e\xda@\tx\xb9\xe6x\xbe\xf6u\x1f\x04x\xbe\xe6u\xf0\x05\xa4\xf5K\x02\n\xff\xe5\x1f\xb4\x04\x10\x90\x0e\x94\xe4x\xc3\x12\x0e\xe9@\x02\xd27u\x1f\x05"0\x01\x03\x02\x04\xc00\x02\x03\x02\x04\xc0\x90Q\xa5\xe0x\x93\xf6\xa3\xe0\x08\xf6\xa3\xe0\x08\xf6\xe5\x1fp<u\x1e \xd25\x12\x0czx~\xa6\x06\x08\xa6\x07x\x8b\xa6\t\x18v\x01\x12\x0c[xN\xa6\x06\x08\xa6\x07x\x8b\xe6xn\xf6u\x1f\x01x\x93\xe6x\x90\xf6x\x94\xe6x\x91\xf6x\x95\xe6x\x92\xf6"y\x90\xe7\xd3x\x93\x96@\x05\xe7\x96\xff\x80\x08\xc3y\x93\xe7x\x90\x96\xffx\x88v\x00\x08\xa6\x07y\x91\xe7\xd3x\x94\x96@\x05\xe7\x96\xff\x80\x08\xc3y\x94\xe7x\x91\x96\xff\x12\x0c\x8ey\x92\xe7\xd3x\x95\x96@\x05\xe7\x96\xff\x80\x08\xc3y\x95\xe7x\x92\x96\xff\x12\x0c\x8e\x12\x0c[x\x8a\xe6%\xe0$N\xf8\xa6\x06\x08\xa6\x07x\x8a\xe6$n\xf8\xa6\tx\x8a\xe6$\x01\xff\xe43\xfe\xd3\xef\x94\x0f\xeed\x80\x94\x80@\x04\x7f\x00\x80\x05x\x8a\xe6\x04\xffx\x8a\xa6\x07\xe5\x1f\xb4\x01\n\xe6`\x03\x02\x04\xc0u\x1f\x02"\x12\x0czx\x80\xa6\x06\x08\xa6\x07\x12\x0czx\x82\xa6\x06\x08\xa6\x07xn\xe6x\x8c\xf6xn\xe6x\x8d\xf6\x7f\x01\xef%\xe0$O\xf9\xc3x\x81\xe6\x97\x18\xe6\x19\x97P\n\x12\x0c\x82x\x80\xa6\x04\x08\xa6\x05tn/\xf9x\x8c\xe6\xc3\x97P\x08tn/\xf8\xe6x\x8c\xf6\xef%\xe0$O\xf9\xd3x\x83\xe6\x97\x18\xe6\x19\x97@\n\x12\x0c\x82x\x82\xa6\x04\x08\xa6\x05tn/\xf9x\x8d\xe6\xd3\x97@\x08tn/\xf8\xe6x\x8d\xf6\x0f\xefd\x10p\x9e\xc3y\x81\xe7x\x83\x96\xff\x19\xe7\x18\x96x\x84\xf6\x08\xa6\x07\xc3y\x8c\xe7x\x8d\x96\x08\xf6\xd3y\x81\xe7x\x7f\x96\x19\xe7\x18\x96@\x05\t\xe7\x08\x80\x06\xc3y\x7f\xe7x\x81\x96\xff\x19\xe7\x18\x96\xfex\x86\xa6\x06\x08\xa6\x07y\x8c\xe7\xd3x\x8b\x96@\x05\xe7\x96\xff\x80\x08\xc3y\x8b\xe7x\x8c\x96\xffx\x8f\xa6\x07\xe5\x1fd\x02pi\x90\x0e\x91\x93\xff\x18\xe6\xc3\x9fPr\x12\x0cJ\x12\x0c/\x90\x0e\x8e\x12\x0c8x\x80\x12\x0ck{\x04\x12\x0c\x1d\xc3\x12\x06EPV\x90\x0e\x92\xe4\x93\xffx\x8f\xe6\x9f@\x02\x80\x11\x90\x0e\x90\xe4\x93\xff\xd3x\x89\xe6\x9f\x18\xe6\x94\x00@\x03u\x1f\x05\x12\x0cJ\x12\x0c/\x90\x0e\x8f\x12\x0c8x~\x12\x0ck{@\x12\x0c\x1d\xd3\x12\x06E@\x18u\x1f\x05"\xe5\x1f\xb4\x05\x0f\xd2\x01\xc2\x02\xe4\xf5\x1f\xf5\x1e\xd25\xd23\xd26"\xef\x8d\xf0\xa4\xa8\xf0\xcf\x8c\xf0\xa4(\xce\x8d\xf0\xa4.\xfe"\xbc\x00\x0b\xbe\x00)\xef\x8d\xf0\x84\xff\xad\xf0"\xe4\xcc\xf8u\xf0\x08\xef/\xff\xee3\xfe\xec3\xfc\xee\x9d\xec\x98@\x05\xfc\xee\x9d\xfe\x0f\xd5\xf0\xe9\xe4\xce\xfd"\xed\xf8\xf5\xf0\xee\x84 \xd2\x1c\xfe\xad\xf0u\xf0\x08\xef/\xff\xed3\xfd@\x07\x98P\x06\xd5\xf0\xf2"\xc3\x98\xfd\x0f\xd5\xf0\xea"\xe8\x8f\xf0\xa4\xcc\x8b\xf0\xa4,\xfc\xe9\x8e\xf0\xa4,\xfc\x8a\xf0\xed\xa4,\xfc\xea\x8e\xf0\xa4\xcd\xa8\xf0\x8b\xf0\xa4-\xcc8%\xf0\xfd\xe9\x8f\xf0\xa4,\xcd5\xf0\xfc\xeb\x8e\xf0\xa4\xfe\xa9\xf0\xeb\x8f\xf0\xa4\xcf\xc5\xf0.\xcd9\xfe\xe4<\xfc\xea\xa4-\xce5\xf0\xfd\xe4<\xfc"u\xf0\x08u\x82\x00\xef/\xff\xee3\xfe\xcd3\xcd\xcc3\xcc\xc5\x823\xc5\x82\x9b\xed\x9a\xec\x99\xe5\x82\x98@\x0c\xf5\x82\xee\x9b\xfe\xed\x9a\xfd\xec\x99\xfc\x0f\xd5\xf0\xd6\xe4\xce\xfb\xe4\xcd\xfa\xe4\xcc\xf9\xa8\x82"\xb8\x00\xc1\xb9\x00Y\xba\x00-\xec\x8b\xf0\x84\xcf\xce\xcd\xfc\xe5\xf0\xcb\xf9x\x18\xef/\xff\xee3\xfe\xed3\xfd\xec3\xfc\xeb3\xfb\x10\xd7\x03\x99@\x04\xeb\x99\xfb\x0f\xd8\xe5\xe4\xf9\xfa"x\x18\xef/\xff\xee3\xfe\xed3\xfd\xec3\xfc\xc93\xc9\x10\xd7\x05\x9b\xe9\x9a@\x07\xec\x9b\xfc\xe9\x9a\xf9\x0f\xd8\xe0\xe4\xc9\xfa\xe4\xcc\xfb"u\xf0\x10\xef/\xff\xee3\xfe\xed3\xfd\xcc3\xcc\xc83\xc8\x10\xd7\x07\x9b\xec\x9a\xe8\x99@\n\xed\x9b\xfd\xec\x9a\xfc\xe8\x99\xf8\x0f\xd5\xf0\xda\xe4\xcd\xfb\xe4\xcc\xfa\xe4\xc8\xf9"\xeb\x9f\xf5\xf0\xea\x9eB\xf0\xe9\x9dB\xf0\xe8\x9cE\xf0"\xe8`\x0f\xec\xc3\x13\xfc\xed\x13\xfd\xee\x13\xfe\xef\x13\xff\xd8\xf1"\xe8`\x0f\xef\xc33\xff\xee3\xfe\xed3\xfd\xec3\xfc\xd8\xf1"\xe4\x93\xfct\x01\x93\xfdt\x02\x93\xfet\x03\x93\xff"\xe6\xfb\x08\xe6\xf9\x08\xe6\xfa\x08\xe6\xcb\xf8"\xec\xf6\x08\xed\xf6\x08\xee\xf6\x08\xef\xf6"\xa4%\x82\xf5\x82\xe5\xf05\x83\xf5\x83"\xd0\x83\xd0\x82\xf8\xe4\x93p\x12t\x01\x93p\r\xa3\xa3\x93\xf8t\x01\x93\xf5\x82\x88\x83\xe4st\x02\x93h`\xef\xa3\xa3\xa3\x80\xdf\x908\x04xR\x12\x0b\xfd\x908\x00\xe0\xfe\xa3\xe0\xfd\xed\xff\xc3\x12\x0b\x9e\x908\x10\x12\x0b\x92\x908\x06xT\x12\x0b\xfd\x908\x02\xe0\xfe\xa3\xe0\xfd\xed\xff\xc3\x12\x0b\x9e\x908\x12\x12\x0b\x92\xa3\xe0\xb41\x07xRyR\x12\x0c\x13\x908\x14\xe0\xb4q\x15xR\xe6\xfe\x08\xe6x\x02\xce\xc3\x13\xce\x13\xd8\xf9yS\xf7\xee\x19\xf7\x908\x15\xe0\xb41\x07xTyT\x12\x0c\x13\x908\x15\xe0\xb4q\x15xT\xe6\xfe\x08\xe6x\x02\xce\xc3\x13\xce\x13\xd8\xf9yU\xf7\xee\x19\xf7yR\x12\x0b\xd9\t\x12\x0b\xd9\xafG\x12\x0b\xb2\xe5D\xfbz\x00\xfd|\x00\x12\x04\xd3xZ\xa6\x06\x08\xa6\x07\xafE\x12\x0b\xb2\xad\x03|\x00\x12\x04\xd3xV\xa6\x06\x08\xa6\x07\xafHxT\x12\x0b\xb4\xe5C\xfb\xfd|\x00\x12\x04\xd3x\\\xa6\x06\x08\xa6\x07\xafF~\x00xT\x12\x0b\xb6\xad\x03|\x00\x12\x04\xd3xX\xa6\x06\x08\xa6\x07\xc3x[\xe6\x94\x08\x18\xe6\x94\x00P\x05v\x00\x08v\x08\xc3x]\xe6\x94\x08\x18\xe6\x94\x00P\x05v\x00\x08v\x08xZ\x12\x0b\xc6\xff\xd3xW\xe6\x9f\x18\xe6\x9e@\x0exZ\xe6\x13\xfe\x08\xe6xW\x12\x0c\x08\x80\x04~\x00\x7f\x00x^\x12\x0b\xbe\xff\xd3xY\xe6\x9f\x18\xe6\x9e@\x0ex\\\xe6\x13\xfe\x08\xe6xY\x12\x0c\x08\x80\x04~\x00\x7f\x00\xe4\xfc\xfdxb\x12\x06\x99xZ\x12\x0b\xc6xW&\xff\xee\x186\xfexf\x12\x0b\xbexY&\xff\xee\x186\xfe\xe4\xfc\xfdxj\x12\x06\x99\x12\x0b\xcexf\x12\x06\x8c\xd3\x12\x06E@\x08\x12\x0b\xcexf\x12\x06\x99xT\x12\x0b\xd0xj\x12\x06\x8c\xd3\x12\x06E@\nxT\x12\x0b\xd0xj\x12\x06\x99xa\xe6\x90`\x01\xf0xe\xe6\xa3\xf0xi\xe6\xa3\xf0xU\xe6\xa3\xf0}\x01xa\x12\x0b\xe9$\x01\x12\x0b\xa6xe\x12\x0b\xe9$\x02\x12\x0b\xa6xi\x12\x0b\xe9$\x03\x12\x0b\xa6xm\x12\x0b\xe9$\x04\x12\x0b\xa6\r\xbd\x05\xd4\xc2\x0e\xc2\x06"\x85\x08A\x900$\xe0\xf5=\xa3\xe0\xf5>\xa3\xe0\xf5?\xa3\xe0\xf5@\xa3\xe0\xf5<\xd24\xe5A\x12\x06\xb1\t1\x03\t5\x04\t;\x05\t>\x06\tA\x07\tJ\x08\t[\x12\ts\x18\t\x89\x19\t^\x1a\tj\x1b\t\xad\x80\t\xb2\x81\n\x1d\x8f\n\t\x90\n\x1d\x91\n\x1d\x92\n\x1d\x93\n\x1d\x94\n\x1d\x98\n\x17\x9f\n\x1a\xec\x00\x00\n8\x12\x0ft"\x12\x0ft\xd2\x03"\xd2\x03"\xc2\x03"\xa27\xe43\xf5<\x02\n\x1d\xc2\x01\xc2\x02\xc2\x03\x12\r\ru\x1ep\xd25\x02\n\x1d\x02\n\x04\x85@J\x85<K\x12\n\xff\x02\n\x1d\x85J@\x85K<\x02\n\x1d\xe4\xf5"\xf5#\x85@1\x85?0\x85>/\x85=.\x12\x0fF\x80\x1fu"\x00u#\x01t\xff\xf5-\xf5,\xf5+\xf5*\x12\x0fF\x85-@\x85,?\x85+>\x85*=\xe4\xf5<\x80p\x12\x0f\x16\x80k\x85=E\x85>F\xe5G\xc3\x13\xff\xe5E\xc3\x9fP\x02\x8fE\xe5H\xc3\x13\xff\xe5F\xc3\x9fP\x02\x8fF\xe5G\xc3\x13\xff\xfd\xe5E-\xfd\xe43\xfc\xe5D\x12\x0f\x90@\x05\xe5D\x9f\xf5E\xe5H\xc3\x13\xff\xfd\xe5F-\xfd\xe43\xfc\xe5C\x12\x0f\x90@\x05\xe5C\x9f\xf5F\x12\x06\xd7\x80\x14\x85@H\x85?G\x85>F\x85=E\x80\x06\x02\x06\xd7\x12\r~\x900$\xe5=\xf0\xa3\xe5>\xf0\xa3\xe5?\xf0\xa3\xe5@\xf0\xa3\xe5<\xf0\x900#\xe4\xf0"\xc0\xe0\xc0\x83\xc0\x82\xc0\xd0\x90?\x0c\xe0\xf52\xe520\xe3t06f\x90`\x19\xe0\xf5\n\xa3\xe0\xf5\x0b\x90`\x1d\xe0\xf5\x14\xa3\xe0\xf5\x15\x90`!\xe0\xf5\x0c\xa3\xe0\xf5\r\x90`)\xe0\xf5\x0e\xa3\xe0\xf5\x0f\x90`1\xe0\xf5\x10\xa3\xe0\xf5\x11\x90`9\xe0\xf5\x12\xa3\xe0\xf5\x130\x01\x0603\x03\xd3\x80\x01\xc3\x92\t0\x02\x0603\x03\xd3\x80\x01\xc3\x92\n03\x0c0\x03\t \x02\x06 \x01\x03\xd3\x80\x01\xc3\x92\x0b\x900\x01\xe0D@\xf0\xe0T\xbf\xf0\xe520\xe1\x1404\x11\x900"\xe0\xf5\x08\xe4\xf00\x00\x03\xd3\x80\x01\xc3\x92\x08\xe520\xe5\x12\x90V\xa1\xe0\xf5\t01\t0\x05\x03\xd3\x80\x01\xc3\x92\r\x90?\x0c\xe52\xf0\xd0\xd0\xd0\x82\xd0\x83\xd0\xe02\x90\x0e~\xe4\x93\xfet\x01\x93\xff\xc3\x90\x0e|t\x01\x93\x9f\xff\xe4\x93\x9e\xfe\xe4\x8f;\x8e:\xf59\xf58\xab;\xaa:\xa99\xa88\xafK\xfc\xfd\xfe\x12\x05(\x12\r\xe1\xe4{\xff\xfa\xf9\xf8\x12\x05\xb3\x12\r\xe1\x90\x0ei\xe4\x12\r\xf6\x12\r\xe1\xe4\x85J7\xf56\xf55\xf54\xaf7\xae6\xad5\xac4\xa3\x12\r\xf6\x8f7\x8e6\x8d5\x8c4\xe5;E7\xf5;\xe5:E6\xf5:\xe59E5\xf59\xe58E4\xf58\xe4\xf5"\xf5#\x85;1\x85:0\x859/\x858.\x02\x0fF\xe0\xa3\xe0u\xf0\x02\xa4\xff\xae\xf0\xc3\x08\xe6\x9f\xf6\x18\xe6\x9e\xf6"\xff\xe5\xf04`\x8f\x82\xf5\x83\xec\xf0"xR~\x00\xe6\xfc\x08\xe6\xfd\x02\x04\xc1\xe4\xfc\xfd\x12\x06\x99x\\\xe6\xc3\x13\xfe\x08\xe6\x13"xR\xe6\xfe\x08\xe6\xff\xe4\xfc\xfd"\xe7\xc4\xf8T\xf0\xc8h\xf7\t\xe7\xc4T\x0fH\xf7"\xe6\xfc\xedu\xf0\x04\xa4"\x12\x06|\x8fH\x8eG\x8dF\x8cE"\xe0\xfe\xa3\xe0\xfd\xee\xf6\xed\x08\xf6"\x13\xff\xc3\xe6\x9f\xff\x18\xe6\x9e\xfe"\xe6\xc3\x13\xf7\x08\xe6\x13\t\xf7"\xad9\xac8\xfa\xf9\xf8\x12\x05(\x8f;\x8e:\x8d9\x8c8\xab7\xaa6\xa95\xa84"\x93\xff\xe4\xfc\xfd\xfe\x12\x05(\x8f7\x8e6\x8d5\x8c4"x\x84\xe6\xfe\x08\xe6\xff\xe4\x8f7\x8e6\xf55\xf54"\x90\x0e\x8c\xe4\x93%\xe0$\n\xf8\xe6\xfe\x08\xe6\xff"\xe6\xfe\x08\xe6\xff\xe4\x8f;\x8e:\xf59\xf58"xN\xe6\xfe\x08\xe6\xff"\xef%\xe0$N\xf8\xe6\xfc\x08\xe6\xfd"x\x89\xef&\xf6\x18\xe46\xf6"u\x89\x03u\xa8\x01u\xb8\x04u4\xffu5\x0eu6\x15u7\r\x12\x0e\x9a\x12\x00\t\x12\x0f\x16\x12\x00\x06\xd2\x00\xd24\xd2\xafu4\xffu5\x0eu6Iu7\x03\x12\x0e\x9a0\x08\t\xc24\x12\x08\xcb\xc2\x08\xd240\x0b\t\xc26\x12\x02l\xc2\x0b\xd260\t\t\xc26\x12\x00\x0e\xc2\t\xd260\x0e\x03\x12\x06\xd705\xd3\x900)\xe5\x1e\xf0\xb4\x10\x05\x900#\xe4\xf0\xc25\x80\xc1\xe4\xf5K\x90\x0ez\x93\xff\xe4\x8f7\xf56\xf55\xf54\xaf7\xae6\xad5\xac4\x90\x0ej\x12\r\xf6\x8f7\x8e6\x8d5\x8c4\x90\x0er\x12\x06|\xefE7\xf57\xeeE6\xf56\xedE5\xf55\xecE4\xf54\xe4\xf5"\xf5#\x8571\x8560\x855/\x854.\x12\x0fF\xe4\xf5"\xf5#\x90\x0er\x12\r\xea\x12\x0fF\xe4\xf5"\xf5#\x90\x0en\x12\r\xea\x02\x0fF\xe5@$\xf2\xf57\xe5?4C\xf56\xe5>4\xa2\xf55\xe5=4(\xf54\xe57\xff\xe4\xfe\xfd\xfcx\x18\x12\x06i\x8f@\x8e?\x8d>\x8c=\xe57T\xa0\xff\xe56\xfe\xe4\xfd\xfcx\x07\x12\x06Vx\x10\x12\x0f\x9a\xe4\xff\xfe\xe55\xfd\xe4\xfcx\x0e\x12\x06V\x12\x0f\x9d\xe4\xff\xfe\xfd\xe54\xfcx\x18\x12\x06Vx\x08\x12\x0f\x9a"\x8f;\x8e:\x8d9\x8c8"\x12\x06|\x8f1\x8e0\x8d/\x8c."\x93\xf9\xf8\x02\x06i\x00\x00\x00\x00\x12\x01\x17\x081\x15STD     \x13\x01\x10\x01V@\x1a0)~\x000\x04 \xdf0\x05@\xbfP\x03\x00\xfdP\'\x01\xfe`\x00\x11\x00?\x050\x00?\x06"\x00?\x01*\x00?\x02\x00\x006\x06\x07\x00?\x0b\x0f\xf0\x00\x00\x00\x000\x01@\xbf0\x01\x00\xbf0)p\x00:\x00\x00\xff:\x00\x00\xff6\x036\x02ADX \x18\x10\n\x04\x04\x00\x03\xffd\x00\x00\x80\x00\x00\x00\x00\x00\x00\x02\x04\x06\x06\x00\x03Q\x00zP<(\x1e\x10\x10P-(\x16\x10\x10\x02\x00\x10\x0c\x10\x04\x0cn\x06\x05\x00\xa5Z\x00\xae5\xaf6\xe4\xfd\xed\xc3\x957P3\x12\x0f\xe2\xe4\x93\xf58t\x01\x93\xf59E8`#\x859\x82\x858\x83\xe0\xfc\x12\x0f\xe2t\x03\x93R\x04\x12\x0f\xe2t\x02\x93B\x04\x859\x82\x858\x83\xec\xf0\r\x80\xc7"x\xbe\xe6\xd3\x08\xff\xe6d\x80\xf8\xefd\x80\x98"\x93\xff~\x00\xe6\xfc\x08\xe6\xfd\x12\x04\xc1x\xc1\xe6\xfc\x08\xe6\xfd\xd3\xef\x9d\xee\x9c"x\xbd\xd3\xe6d\x80\x94\x80"%\xe0$\n\xf8\xe6\xfe\x08\xe6\xff"\xe5<\xd3\x94\x00@\x0b\x90\x0e\x88\x12\x0b\xf1\x90\x0e\x86\x80\t\x90\x0e\x82\x12\x0b\xf1\x90\x0e\x80\xe4\x93\xf5D\xa3\xe4\x93\xf5C\xd2\x060\x06\x03\xd3\x80\x01\xc3\x92\x0e"\xa2\xaf\x922\xc2\xaf\xe5#E"\x90\x0e]`\x0e\x12\x0f\xcb\xe0\xf5,\x12\x0f\xc8\xe0\xf5-\x80\x0c\x12\x0f\xcb\xe50\xf0\x12\x0f\xc8\xe51\xf0\xa22\x92\xaf"\xd2\x01\xc2\x02\xe4\xf5\x1f\xf5\x1e\xd25\xd23\xd26\xd2\x01\xc2\x02\xf5\x1f\xf5\x1e\xd25\xd23"\xfb\xd3\xed\x9bt\x80\xf8l\x98"\x12\x06i\xe5@/\xf5@\xe5?>\xf5?\xe5>=\xf5>\xe5=<\xf5="\xc0\xe0\xc0\x83\xc0\x82\x90?\r\xe0\xf53\xe53\xf0\xd0\x82\xd0\x83\xd0\xe02\x90\x0e_\xe4\x93\xfet\x01\x93\xf5\x82\x8e\x83"x\x7f\xe4\xf6\xd8\xfdu\x81\xcd\x02\x0c\x98\x8f\x82\x8e\x83u\xf0\x04\xed\x02\x06\xa5'

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
        b[1] = reg & 0xFF
        b[2] = value
        with self._i2c_device as i2c:
            i2c.write(b)

    def _write_addr_reg(self, reg, x_value, y_value):
        self._write_register16(reg, x_value)
        self._write_register16(reg + 2, y_value)

    def _write_register16(self, reg, value):
        self._write_register(reg, value >> 8)
        self._write_register(reg + 1, value & 0xFF)

    def _read_register(self, reg):
        b = bytearray(2)
        b[0] = reg >> 8
        b[1] = reg & 0xFF
        with self._i2c_device as i2c:
            i2c.write(b)
            i2c.readinto(b, end=1)
        return b[0]

    def _read_register16(self, reg):
        high = self._read_register(reg)
        low = self._read_register(reg + 1)
        return (high << 8) | low

    def _write_list(self, reg_list):
        for i in range(0, len(reg_list), 2):
            register = reg_list[i]
            value = reg_list[i + 1]
            if register == _REG_DLY:
                time.sleep(value / 1000)
            else:
                self._write_register(register, value)

    def _write_reg_bits(self, reg, mask, enable):
        val = self._read_register(reg)
        if enable:
            val |= mask
        else:
            val &= ~mask
        self._write_register(reg, val)


class OV5640(_SCCB16CameraBase):  # pylint: disable=too-many-instance-attributes
    """Control & Capture Images from an OV5640 Camera"""

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
        mclk_frequency=20_000_000,
        i2c_address=0x3C,
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
                ignored if mclk is None, requred if it is specified.
                Note that the OV5640 requires a very low jitter clock,
                so only specific (microcontroller-dependent) values may
                work reliably.  On the ESP32-S2, a 20MHz clock can be generated
                with sufficiently low jitter.
            i2c_address (int): The I2C address of the camera.
        """

        # we're supposed to go into shutdown if we can, first
        if shutdown:
            self._shutdown = digitalio.DigitalInOut(shutdown)
            self._shutdown.switch_to_output(True)
        else:
            self._shutdown = None

        if reset:
            self._reset = digitalio.DigitalInOut(reset)
            self._reset.switch_to_output(False)
        else:
            self._reset = None

        time.sleep(0.1)

        # Initialize the master clock
        if mclk:
            self._mclk_pwm = pwmio.PWMOut(mclk, frequency=mclk_frequency)
            self._mclk_pwm.duty_cycle = 32768
        else:
            self._mclk_pwm = None

        if self._shutdown:
            self._shutdown.switch_to_output(False)
            time.sleep(0.1)


        if self._reset:
            self._reset.switch_to_output(True)

        time.sleep(0.1)

        # Now that the master clock is running, we can initialize i2c comms
        super().__init__(i2c_bus, i2c_address)

        self._write_list(_sensor_default_regs)

        self._imagecapture = imagecapture.ParallelImageCapture(
            data_pins=data_pins, clock=clock, vsync=vsync, href=href
        )

        self._colorspace = OV5640_COLOR_RGB
        self._flip_x = False
        self._flip_y = False
        self._w = None
        self._h = None
        self._size = None
        self._test_pattern = False
        self._binning = False
        self._scale = False
        self._ev = 0
        self._white_balance = 0
        self.size = size
        self._strobe_enabled = False

    chip_id = _RegBits16(_CHIP_ID_HIGH, 0, 0xFFFF)

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
            return self.width * self.height // self.quality
        if self.colorspace == OV5640_COLOR_GRAYSCALE:
            return self.width * self.height
        return self.width * self.height * 2

    @property
    def mclk_frequency(self):
        """Get the actual frequency the generated mclk, or None"""
        return self._mclk_pwm.frequency if self._mclk_pwm else None

    @property
    def width(self):
        """Get the image width in pixels."""
        return self._w

    @property
    def height(self):
        """Get the image height in pixels."""
        return self._h

    @property
    def colorspace(self):
        """Get or set the colorspace, one of the ``OV5640_COLOR_`` constants."""
        return self._colorspace

    @colorspace.setter
    def colorspace(self, colorspace):
        self._colorspace = colorspace
        self._set_size_and_colorspace()

    def _set_image_options(self):  # pylint: disable=too-many-branches
        reg20 = reg21 = reg4514 = reg4514_test = 0
        if self.colorspace == OV5640_COLOR_JPEG:
            reg21 |= 0x20

        if self._binning:
            reg20 |= 1
            reg21 |= 1
            reg4514_test |= 4
        else:
            reg20 |= 0x40

        if self._flip_y:
            reg20 |= 0x06
            reg4514_test |= 1

        if self._flip_x:
            reg21 |= 0x06
            reg4514_test |= 2

        if reg4514_test == 0:
            reg4514 = 0x88
        elif reg4514_test == 1:
            reg4514 = 0x00
        elif reg4514_test == 2:
            reg4514 = 0xBB
        elif reg4514_test == 3:
            reg4514 = 0x00
        elif reg4514_test == 4:
            reg4514 = 0xAA
        elif reg4514_test == 5:
            reg4514 = 0xBB
        elif reg4514_test == 6:
            reg4514 = 0xBB
        elif reg4514_test == 7:
            reg4514 = 0xAA

        self._write_register(_TIMING_TC_REG20, reg20)
        self._write_register(_TIMING_TC_REG21, reg21)
        self._write_register(0x4514, reg4514)

        if self._binning:
            self._write_register(0x4520, 0x0B)
            self._write_register(_X_INCREMENT, 0x31)
            self._write_register(_Y_INCREMENT, 0x31)
        else:
            self._write_register(0x4520, 0x10)
            self._write_register(_X_INCREMENT, 0x11)
            self._write_register(_Y_INCREMENT, 0x11)

    def _set_colorspace(self):
        colorspace = self._colorspace
        settings = _ov5640_color_settings[colorspace]

        self._write_list(settings)

    def deinit(self):
        """Deinitialize the camera"""
        self._imagecapture.deinit()
        if self._mclk_pwm:
            self._mclk_pwm.deinit()
        if self._shutdown:
            self._shutdown.deinit()
        if self._reset:
            self._reset.deinit()
        self.powerdown = True

    @property
    def size(self):
        """Get or set the captured image size, one of the ``OV5640_SIZE_`` constants."""
        return self._size

    def _set_size_and_colorspace(self):  # pylint: disable=too-many-locals
        size = self._size
        width, height, ratio = _resolution_info[size]
        self._w = width
        self._h = height
        (
            max_width,
            max_height,
            start_x,
            start_y,
            end_x,
            end_y,
            offset_x,
            offset_y,
            total_x,
            total_y,
        ) = _ratio_table[ratio]

        self._binning = (width <= max_width // 2) and (height <= max_height // 2)
        self._scale = not (
            (width == max_width and height == max_height)
            or (width == max_width // 2 and height == max_height // 2)
        )

        self._write_addr_reg(_X_ADDR_ST_H, start_x, start_y)
        self._write_addr_reg(_X_ADDR_END_H, end_x, end_y)
        self._write_addr_reg(_X_OUTPUT_SIZE_H, width, height)

        if not self._binning:
            self._write_addr_reg(_X_TOTAL_SIZE_H, total_x, total_y)
            self._write_addr_reg(_X_OFFSET_H, offset_x, offset_y)
        else:
            if width > 920:
                self._write_addr_reg(_X_TOTAL_SIZE_H, total_x - 200, total_y // 2)
            else:
                self._write_addr_reg(_X_TOTAL_SIZE_H, 2060, total_y // 2)
            self._write_addr_reg(_X_OFFSET_H, offset_x // 2, offset_y // 2)

        self._write_reg_bits(_ISP_CONTROL_01, 0x20, self._scale)

        self._set_image_options()

        if self.colorspace == OV5640_COLOR_JPEG:
            sys_mul = 200
            if size < OV5640_SIZE_QVGA:
                sys_mul = 160
            if size < OV5640_SIZE_XGA:
                sys_mul = 180
            self._set_pll(False, sys_mul, 4, 2, False, 2, True, 4)
        else:
            self._set_pll(False, 32, 1, 1, False, 1, True, 4)

        self._set_colorspace()

    def _set_pll(  # pylint: disable=too-many-arguments
        self,
        bypass,
        multiplier,
        sys_div,
        pre_div,
        root_2x,
        pclk_root_div,
        pclk_manual,
        pclk_div,
    ):
        if (  # pylint: disable=too-many-boolean-expressions
            multiplier > 252
            or multiplier < 4
            or sys_div > 15
            or pre_div > 8
            or pclk_div > 31
            or pclk_root_div > 3
        ):
            raise ValueError("Invalid argument to internal function")

        self._write_register(0x3039, 0x80 if bypass else 0)
        self._write_register(0x3034, 0x1A)
        self._write_register(0x3035, 1 | ((sys_div & 0xF) << 4))
        self._write_register(0x3036, multiplier & 0xFF)
        self._write_register(0x3037, (pre_div & 0xF) | (0x10 if root_2x else 0))
        self._write_register(0x3108, (pclk_root_div & 3) << 4 | 0x06)
        self._write_register(0x3824, pclk_div & 0x1F)
        self._write_register(0x460C, 0x22 if pclk_manual else 0x22)
        self._write_register(0x3103, 0x13)

    @size.setter
    def size(self, size):
        self._size = size
        self._set_size_and_colorspace()

    @property
    def flip_x(self):
        """Get or set the X-flip flag"""
        return self._flip_x

    @flip_x.setter
    def flip_x(self, value):
        self._flip_x = bool(value)
        self._set_image_options()

    @property
    def flip_y(self):
        """Get or set the Y-flip flag"""
        return self._flip_y

    @flip_y.setter
    def flip_y(self, value):
        self._flip_y = bool(value)
        self._set_image_options()

    @property
    def test_pattern(self):
        """Set to True to enable a test pattern, False to enable normal image capture"""
        return self._test_pattern

    @test_pattern.setter
    def test_pattern(self, value) -> None:
        if type(value) is bool:
            self._write_register(_PRE_ISP_TEST_SETTING_1, value << 7)
        else:
            self._write_register(_PRE_ISP_TEST_SETTING_1, 1 << 7 | value)
        self._test_pattern = value

    @property
    def saturation(self):
        """Get or set the saturation value, from -4 to +4."""
        return self._saturation

    @saturation.setter
    def saturation(self, value):
        if not -4 <= value <= 4:
            raise ValueError(
                "Invalid saturation {value}, use a value from -4..4 inclusive"
            )
        for offset, reg_value in enumerate(_sensor_saturation_levels[value]):
            self._write_register(0x5381 + offset, reg_value)
        self._saturation = value

    @property
    def effect(self):
        """Get or set the special effect, one of the ``OV5640_SPECIAL_EFFECT_`` constants"""
        return self._effect

    @effect.setter
    def effect(self, value):
        for reg_addr, reg_value in zip(
            (0x5580, 0x5583, 0x5584, 0x5003), _sensor_special_effects[value]
        ):
            self._write_register(reg_addr, reg_value)
        self._effect = value

    @property
    def quality(self):
        """Controls the JPEG quality.  Valid range is from 2..55 inclusive"""
        return self._read_register(_COMPRESSION_CTRL07) & 0x3F

    @quality.setter
    def quality(self, value: int):
        if not 2 <= value < 55:
            raise ValueError(
                f"Invalid quality value {value}, use a value from 2..55 inclusive"
            )
        self._write_register(_COMPRESSION_CTRL07, value & 0x3F)

    def _write_group_3_settings(self, settings):
        self._write_register(0x3212, 0x3)  # start group 3
        self._write_list(settings)
        self._write_register(0x3212, 0x13)  # end group 3
        self._write_register(0x3212, 0xA3)  # launch group 3

    @property
    def brightness(self):
        """Sensor brightness adjustment, from -4 to 4 inclusive"""
        brightness_abs = self._read_register(0x5587) >> 4
        brightness_neg = self._read_register(0x5588) & 8
        if brightness_neg:
            return -brightness_abs
        return brightness_abs

    @brightness.setter
    def brightness(self, value):
        if not -4 <= value <= 4:
            raise ValueError(
                "Invalid brightness value {value}, use a value from -4..4 inclusive"
            )
        self._write_group_3_settings(
            [0x5587, abs(value) << 4, 0x5588, 0x9 if value < 0 else 0x1]
        )

    @property
    def contrast(self):
        """Sensor contrast adjustment, from -4 to 4 inclusive"""
        contrast_abs = self._read_register(0x5587) >> 4
        contrast_neg = self._read_register(0x5588) & 8
        if contrast_neg:
            return -contrast_abs
        return contrast_abs

    @contrast.setter
    def contrast(self, value):
        if not -3 <= value <= 3:
            raise ValueError(
                "Invalid contrast value {value}, use a value from -3..3 inclusive"
            )
        setting = _contrast_settings[value]
        self._write_group_3_settings([0x5586, setting[0], 0x5585, setting[1]])

    @property
    def exposure_value(self):
        """Sensor exposure (EV) adjustment, from -4 to 4 inclusive"""
        return self._ev

    @exposure_value.setter
    def exposure_value(self, value):
        if not -3 <= value <= 3:
            raise ValueError(
                "Invalid exposure value (EV) {value}, use a value from -4..4 inclusive"
            )
        for offset, reg_value in enumerate(_sensor_ev_levels[value]):
            self._write_register(0x5381 + offset, reg_value)

    @property
    def white_balance(self):
        """The white balance setting, one of the ``OV5640_WHITE_BALANCE_*`` constants"""
        return self._white_balance

    @white_balance.setter
    def white_balance(self, value):
        if not OV5640_WHITE_BALANCE_AUTO <= value <= OV5640_WHITE_BALANCE_INCANDESCENT:
            raise ValueError(
                "Invalid exposure value (EV) {value}, "
                "use one of the OV5640_WHITE_BALANCE_* constants"
            )
        self._write_register(0x3212, 0x3)  # start group 3
        for reg_addr, reg_value in zip(_light_registers, _light_modes[value]):
            self._write_register(reg_addr, reg_value)
        self._write_register(0x3212, 0x13)  # end group 3
        self._write_register(0x3212, 0xA3)  # launch group 3

    @property
    def night_mode(self):
        """Enable or disable the night mode setting of the sensor"""
        return bool(self._read_register(0x3A00) & 0x04)

    @night_mode.setter
    def night_mode(self, value):
        self._write_reg_bits(0x3A00, 0x04, value)

    @property
    def powerdown(self):
        return bool(self._read_register(_SYSTEM_CTROL0) & 0x40)

    @powerdown.setter
    def powerdown(self, value):
        self._write_reg_bits(_SYSTEM_CTROL0, 0x40, bool(value))

    def strobe_config(self, enabled, pulse_invert, frex_mode, strobe_mode):
        # set the FREX mode rolling
        if frex_mode not in (FREX_MODE_0, FREX_MODE_1, FREX_MODE_ROLL):
            raise ValueError("Frex mode must be 0, 1 or 2 (rolling)")
        self._write_register(_FREX_MODE, frex_mode)
        # set the output pin
        self._write_register(_PAD_OUTPUT_ENABLE00, 0x2)
        # set the pulse invert, mode and enable
        if strobe_mode not in (STROBE_MODE_XENON, STROBE_MODE_LED1, STROBE_MODE_LED2, STROBE_MODE_LED3):
            raise ValueError("Strobe mode must be 0~3")
        self._write_register(_STROBE_CTRL,
                             (bool(enabled) << 7) |
                             (bool(pulse_invert) << 6) |
                             (strobe_mode & 0x3))

        #print("strobe reg: ", hex(self._read_register(_STROBE_CTRL)))
        #print("pad00 reg: ", hex(self._read_register(_PAD_OUTPUT_ENABLE00)))
        #print("clock00 reg: ", hex(self._read_register(0x3004)))

    @property
    def strobe_request(self):
        return bool(self._read_register(_FREX_REQUEST))

    @strobe_request.setter
    def strobe_request(self, value):
        self._write_register(_FREX_REQUEST, bool(value))

    @property
    def strobe_pin(self):
        return bool(self._read_register(_PAD_OUTPUT_VALUE00))

    @strobe_pin.setter
    def strobe_pin(self, value):
        self._write_register(_PAD_OUTPUT_ENABLE00, 0x02)
        self._write_register(_PAD_SELECT00, 0x02)
        self._write_register(_PAD_OUTPUT_VALUE00, bool(value) << 1)

    def autofocus_init(self):
        self._write_register(0x3000, 0x20) # reset
        # load firmware
        for addr,val in enumerate(autofocus_firmware):
            #print(hex(addr), hex(val))
            self._write_register(0x8000+addr, val)
        self._write_list(_autofocus_firmware_load)
        for _ in range(100):
            if self.autofocus_status == AUTOFOCUS_STAT_IDLE:
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("Timed out after trying to load autofocus firmware")
        return True

    @property
    def autofocus_status(self):
        return self._read_register(0x3029)
    
    def autofocus(self):
        print("Auto focusing?")
        self._write_register(0x3023, 0x01) # clear command ack
        self._write_register(0x3022, 0x08) # release focus
        for _ in range(100):
            if self._read_register(0x3032) == 0x0: # command is finished
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("Timed out trying to run autofocus")
        
        self._write_register(0x3023, 0x01)
        self._write_register(0x3022, 0x04)
        for _ in range(100):
            if self._read_register(0x3032) == 0x0: # command is finished
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("Timed out trying to run autofocus")

        """
        for _ in range(100):
            focus_stat = self._read_register(0x3029)
            print("Focus status 0x3029:", hex(focus_stat))
            if focus_stat == 0x10: # we focused
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("Timed out trying to autofocus")
        """

        return True
