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

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ov5640.git"

from micropython import const

_SYSTEM_CTROL0 = const(  0x3008  )# Bit[7]: Software reset 
                                # Bit[6]: Software power down 
                                # Bit[5]: Reserved 
                                # Bit[4]: SRB clock SYNC enable 
                                # Bit[3]: Isolation suspend select 
                                # Bit[2:0]: Not used

_DRIVE_CAPABILITY = const(0x302c )# Bit[7:6]:
                                #          00: 1x
                                #          01: 2x
                                #          10: 3x
                                #          11: 4x

_SC_PLLS_CTRL0 = const(   0x303a )# Bit[7]: PLLS bypass
_SC_PLLS_CTRL1 = const(   0x303b )# Bit[4:0]: PLLS multiplier
_SC_PLLS_CTRL2 = const(   0x303c )# Bit[6:4]: PLLS charge pump control
                                # Bit[3:0]: PLLS system divider
_SC_PLLS_CTRL3 = const(   0x303d )# Bit[5:4]: PLLS pre-divider
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
_AEC_PK_MANUAL = const(  0x3503  )# AEC Manual Mode Control
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

#gain = {0x350A[1:0], 0x350B[7:0]} / 16


_X_ADDR_ST_H = const(    0x3800 )#Bit[3:0]: X address start[11:8]
_X_ADDR_ST_L = const(    0x3801 )#Bit[7:0]: X address start[7:0]
_Y_ADDR_ST_H = const(    0x3802 )#Bit[2:0]: Y address start[10:8]
_Y_ADDR_ST_L = const(    0x3803 )#Bit[7:0]: Y address start[7:0]
_X_ADDR_END_H = const(   0x3804 )#Bit[3:0]: X address end[11:8]
_X_ADDR_END_L = const(   0x3805 )#Bit[7:0]:
_Y_ADDR_END_H = const(   0x3806 )#Bit[2:0]: Y address end[10:8]
_Y_ADDR_END_L = const(   0x3807 )#Bit[7:0]:
# Size after scaling
_X_OUTPUT_SIZE_H = const(0x3808 )#Bit[3:0]: DVP output horizontal width[11:8]
_X_OUTPUT_SIZE_L = const(0x3809 )#Bit[7:0]:
_Y_OUTPUT_SIZE_H = const(0x380a )#Bit[2:0]: DVP output vertical height[10:8]
_Y_OUTPUT_SIZE_L = const(0x380b )#Bit[7:0]:
_X_TOTAL_SIZE_H = const( 0x380c )#Bit[3:0]: Total horizontal size[11:8]
_X_TOTAL_SIZE_L = const( 0x380d )#Bit[7:0]:
_Y_TOTAL_SIZE_H = const( 0x380e )#Bit[7:0]: Total vertical size[15:8]
_Y_TOTAL_SIZE_L = const( 0x380f )#Bit[7:0]:
_X_OFFSET_H = const(     0x3810 )#Bit[3:0]: ISP horizontal offset[11:8]
_X_OFFSET_L = const(     0x3811 )#Bit[7:0]:
_Y_OFFSET_H = const(     0x3812 )#Bit[2:0]: ISP vertical offset[10:8]
_Y_OFFSET_L = const(     0x3813 )#Bit[7:0]:
_X_INCREMENT = const(    0x3814 )#Bit[7:4]: Horizontal odd subsample increment
                               #Bit[3:0]: Horizontal even subsample increment
_Y_INCREMENT = const(    0x3815 )#Bit[7:4]: Vertical odd subsample increment
                               #Bit[3:0]: Vertical even subsample increment
# Size before scaling
#X_INPUT_SIZE = const(   (X_ADDR_END - X_ADDR_ST + 1 - (2 * X_OFFSET)))
#Y_INPUT_SIZE = const(   (Y_ADDR_END - Y_ADDR_ST + 1 - (2 * Y_OFFSET)))

 # mirror and flip registers
_TIMING_TC_REG20 = const(0x3820  )# Timing Control Register
                                # Bit[2:1]: Vertical flip enable
                                #         00: Normal
                                #         11: Vertical flip
                                # Bit[0]: Vertical binning enable
_TIMING_TC_REG21 = const(0x3821  )# Timing Control Register
                                # Bit[5]: Compression Enable
                                # Bit[2:1]: Horizontal mirror enable
                                #         00: Normal
                                #         11: Horizontal mirror
                                # Bit[0]: Horizontal binning enable

_PCLK_RATIO = const(      0x3824 )# Bit[4:0]: PCLK ratio manual

 # frame control registers
_FRAME_CTRL01 = const(   0x4201  )# Control Passed Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
                                # Bit[7:4]: Not used
                                # Bit[3:0]: Frame ON number
_FRAME_CTRL02 = const(   0x4202  )# Control Masked Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
                                # Bit[7:4]: Not used
                                # BIT[3:0]: Frame OFF number

 # format control registers
_FORMAT_CTRL00 = const(  0x4300)

_CLOCK_POL_CONTROL = const(0x4740)# Bit[5]: PCLK polarity 0: active low
                                #          1: active high
                                # Bit[3]: Gate PCLK under VSYNC
                                # Bit[2]: Gate PCLK under HREF
                                # Bit[1]: HREF polarity
                                #          0: active low
                                #          1: active high
                                # Bit[0] VSYNC polarity
                                #          0: active low
                                #          1: active high

_ISP_CONTROL_01 = const(  0x5001 )# Bit[5]: Scale enable
                                #          0: Disable
                                #          1: Enable

 # output format control registers
_FORMAT_CTRL = const(    0x501F )# Format select
                                # Bit[2:0]:
                                #  000: YUV422
                                #  001: RGB
                                #  010: Dither
                                #  011: RAW after DPC
                                #  101: RAW after CIP

 # ISP top control registers
_PRE_ISP_TEST_SETTING_1 = const( 0x503D  )# Bit[7]: Test enable
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

#exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]} / 16 × tROW

_SCALE_CTRL_1 = const(    0x5601 )# Bit[6:4]: HDIV RW
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

_SCALE_CTRL_2 = const(    0x5602 )# X_SCALE High Bits
_SCALE_CTRL_3 = const(    0x5603 )# X_SCALE Low Bits
_SCALE_CTRL_4 = const(    0x5604 )# Y_SCALE High Bits
_SCALE_CTRL_5 = const(    0x5605 )# Y_SCALE Low Bits
_SCALE_CTRL_6 = const(    0x5606 )# Bit[3:0]: V Offset

_VFIFO_CTRL0C = const(    0x460C )# Bit[1]: PCLK manual enable
                                #          0: Auto
                                #          1: Manual by PCLK_RATIO

_VFIFO_X_SIZE_H = const(  0x4602)
_VFIFO_X_SIZE_L = const(  0x4603)
_VFIFO_Y_SIZE_H = const(  0x4604)
_VFIFO_Y_SIZE_L = const(  0x4605)

_COMPRESSION_CTRL00 = const(0x4400)
_COMPRESSION_CTRL01 = const(0x4401)
_COMPRESSION_CTRL02 = const(0x4402)
_COMPRESSION_CTRL03 = const(0x4403)
_COMPRESSION_CTRL04 = const(0x4404)
_COMPRESSION_CTRL05 = const(0x4405)
_COMPRESSION_CTRL06 = const(0x4406)
_COMPRESSION_CTRL07 = const(0x4407)# Bit[5:0]: QS
_COMPRESSION_ISI_CTRL = const(0x4408)
_COMPRESSION_CTRL09 = const(0x4409)
_COMPRESSION_CTRL0a = const(0x440a)
_COMPRESSION_CTRL0b = const(0x440b)
_COMPRESSION_CTRL0c = const(0x440c)
_COMPRESSION_CTRL0d = const(0x440d)
_COMPRESSION_CTRL0E = const(0x440e)

_TEST_COLOR_BAR = const( 0xC0    ) # Enable Color Bar roling Test

_AEC_PK_MANUAL_AGC_MANUALEN = const( 0x02    ) # Enable AGC Manual enable
_AEC_PK_MANUAL_AEC_MANUALEN = const( 0x01    ) # Enable AEC Manual enable

_TIMING_TC_REG20_VFLIP = const(  0x06 ) # Vertical flip enable
_TIMING_TC_REG21_HMIRROR = const(0x06 ) # Horizontal mirror enable

_ratio_table = [
    #  mw,   mh,  sx,  sy,   ex,   ey, ox, oy,   tx,   ty
    [ 2560, 1920,   0,   0, 2623, 1951, 32, 16, 2844, 1968 ], #4x3
    [ 2560, 1704,   0, 110, 2623, 1843, 32, 16, 2844, 1752 ], #3x2
    [ 2560, 1600,   0, 160, 2623, 1791, 32, 16, 2844, 1648 ], #16x10
    [ 2560, 1536,   0, 192, 2623, 1759, 32, 16, 2844, 1584 ], #5x3
    [ 2560, 1440,   0, 240, 2623, 1711, 32, 16, 2844, 1488 ], #16x9
    [ 2560, 1080,   0, 420, 2623, 1531, 32, 16, 2844, 1128 ], #21x9
    [ 2400, 1920,  80,   0, 2543, 1951, 32, 16, 2684, 1968 ], #5x4
    [ 1920, 1920, 320,   0, 2543, 1951, 32, 16, 2684, 1968 ], #1x1
    [ 1088, 1920, 736,   0, 1887, 1951, 32, 16, 1884, 1968 ]  #9x16
];

_REG_DLY = const(0xffff)
_REGLIST_TAIL = const(0x0000)

_sensor_default_regs = [
    [_SYSTEM_CTROL0, 0x82],  # software reset
    [_REG_DLY, 10], # delay 10ms
    [_SYSTEM_CTROL0, 0x42],  # power down

    #enable pll
    [0x3103, 0x13],

    #io direction
    [0x3017, 0xff],
    [0x3018, 0xff],

    [_DRIVE_CAPABILITY, 0xc3],
    [_CLOCK_POL_CONTROL, 0x21],

    [0x4713, 0x02],#jpg mode select

    [_ISP_CONTROL_01, 0x83], # turn color matrix, awb and SDE

    #sys reset
    [0x3000, 0x00],
    [0x3002, 0x1c],

    #clock enable
    [0x3004, 0xff],
    [0x3006, 0xc3],

    #isp control
    [0x5000, 0xa7],
    [_ISP_CONTROL_01, 0xa3],#+scaling?
    [0x5003, 0x08],#special_effect

    #unknown
    [0x370c, 0x02],#!!IMPORTANT
    [0x3634, 0x40],#!!IMPORTANT

    #AEC/AGC
    [0x3a02, 0x03],
    [0x3a03, 0xd8],
    [0x3a08, 0x01],
    [0x3a09, 0x27],
    [0x3a0a, 0x00],
    [0x3a0b, 0xf6],
    [0x3a0d, 0x04],
    [0x3a0e, 0x03],
    [0x3a0f, 0x30],#ae_level
    [0x3a10, 0x28],#ae_level
    [0x3a11, 0x60],#ae_level
    [0x3a13, 0x43],
    [0x3a14, 0x03],
    [0x3a15, 0xd8],
    [0x3a18, 0x00],#gainceiling
    [0x3a19, 0xf8],#gainceiling
    [0x3a1b, 0x30],#ae_level
    [0x3a1e, 0x26],#ae_level
    [0x3a1f, 0x14],#ae_level

    #vcm debug
    [0x3600, 0x08],
    [0x3601, 0x33],

    #50/60Hz
    [0x3c01, 0xa4],
    [0x3c04, 0x28],
    [0x3c05, 0x98],
    [0x3c06, 0x00],
    [0x3c07, 0x08],
    [0x3c08, 0x00],
    [0x3c09, 0x1c],
    [0x3c0a, 0x9c],
    [0x3c0b, 0x40],

    [0x460c, 0x22],#disable jpeg footer

    #BLC
    [0x4001, 0x02],
    [0x4004, 0x02],

    #AWB
    [0x5180, 0xff],
    [0x5181, 0xf2],
    [0x5182, 0x00],
    [0x5183, 0x14],
    [0x5184, 0x25],
    [0x5185, 0x24],
    [0x5186, 0x09],
    [0x5187, 0x09],
    [0x5188, 0x09],
    [0x5189, 0x75],
    [0x518a, 0x54],
    [0x518b, 0xe0],
    [0x518c, 0xb2],
    [0x518d, 0x42],
    [0x518e, 0x3d],
    [0x518f, 0x56],
    [0x5190, 0x46],
    [0x5191, 0xf8],
    [0x5192, 0x04],
    [0x5193, 0x70],
    [0x5194, 0xf0],
    [0x5195, 0xf0],
    [0x5196, 0x03],
    [0x5197, 0x01],
    [0x5198, 0x04],
    [0x5199, 0x12],
    [0x519a, 0x04],
    [0x519b, 0x00],
    [0x519c, 0x06],
    [0x519d, 0x82],
    [0x519e, 0x38],

    #color matrix (Saturation)
    [0x5381, 0x1e],
    [0x5382, 0x5b],
    [0x5383, 0x08],
    [0x5384, 0x0a],
    [0x5385, 0x7e],
    [0x5386, 0x88],
    [0x5387, 0x7c],
    [0x5388, 0x6c],
    [0x5389, 0x10],
    [0x538a, 0x01],
    [0x538b, 0x98],

    #CIP control (Sharpness)
    [0x5300, 0x10],#sharpness
    [0x5301, 0x10],#sharpness
    [0x5302, 0x18],#sharpness
    [0x5303, 0x19],#sharpness
    [0x5304, 0x10],
    [0x5305, 0x10],
    [0x5306, 0x08],#denoise
    [0x5307, 0x16],
    [0x5308, 0x40],
    [0x5309, 0x10],#sharpness
    [0x530a, 0x10],#sharpness
    [0x530b, 0x04],#sharpness
    [0x530c, 0x06],#sharpness

    #GAMMA
    [0x5480, 0x01],
    [0x5481, 0x00],
    [0x5482, 0x1e],
    [0x5483, 0x3b],
    [0x5484, 0x58],
    [0x5485, 0x66],
    [0x5486, 0x71],
    [0x5487, 0x7d],
    [0x5488, 0x83],
    [0x5489, 0x8f],
    [0x548a, 0x98],
    [0x548b, 0xa6],
    [0x548c, 0xb8],
    [0x548d, 0xca],
    [0x548e, 0xd7],
    [0x548f, 0xe3],
    [0x5490, 0x1d],

    #Special Digital Effects (SDE) (UV adjust)
    [0x5580, 0x06],#enable brightness and contrast
    [0x5583, 0x40],#special_effect
    [0x5584, 0x10],#special_effect
    [0x5586, 0x20],#contrast
    [0x5587, 0x00],#brightness
    [0x5588, 0x00],#brightness
    [0x5589, 0x10],
    [0x558a, 0x00],
    [0x558b, 0xf8],
    [0x501d, 0x40],# enable manual offset of contrast

    #power on
    [0x3008, 0x02],

    #50Hz
    [0x3c00, 0x04],
    
    [_REG_DLY, 300],
];

_sensor_fmt_jpeg = [
    [_FORMAT_CTRL, 0x00], # YUV422
    [_FORMAT_CTRL00, 0x30], # YUYV
    [0x3002, 0x00],#0x1c to 0x00 !!!
    [0x3006, 0xff],#0xc3 to 0xff !!!
    [0x471c, 0x50],#0xd0 to 0x50 !!!
];

_sensor_fmt_raw = [
    [_FORMAT_CTRL, 0x03], # RAW (DPC)
    [_FORMAT_CTRL00, 0x00], # RAW
];

_sensor_fmt_grayscale = [
    [_FORMAT_CTRL, 0x00], # YUV422
    [_FORMAT_CTRL00, 0x10], # Y8
];

_sensor_format_yuv422 = [
    [_FORMAT_CTRL, 0x00], # YUV422
    [_FORMAT_CTRL00, 0x30], # YUYV
];

_sensor_format_rgb565 = [
    [_FORMAT_CTRL, 0x01], # RGB
    [_FORMAT_CTRL00, 0x61], # RGB565 (BGR)
];

_sensor_saturaion_levels = [
    [0x1d, 0x60, 0x03, 0x07, 0x48, 0x4f, 0x4b, 0x40, 0x0b, 0x01, 0x98],#-4
    [0x1d, 0x60, 0x03, 0x08, 0x54, 0x5c, 0x58, 0x4b, 0x0d, 0x01, 0x98],#-3
    [0x1d, 0x60, 0x03, 0x0a, 0x60, 0x6a, 0x64, 0x56, 0x0e, 0x01, 0x98],#-2
    [0x1d, 0x60, 0x03, 0x0b, 0x6c, 0x77, 0x70, 0x60, 0x10, 0x01, 0x98],#-1
    [0x1d, 0x60, 0x03, 0x0c, 0x78, 0x84, 0x7d, 0x6b, 0x12, 0x01, 0x98],#0
    [0x1d, 0x60, 0x03, 0x0d, 0x84, 0x91, 0x8a, 0x76, 0x14, 0x01, 0x98],#+1
    [0x1d, 0x60, 0x03, 0x0e, 0x90, 0x9e, 0x96, 0x80, 0x16, 0x01, 0x98],#+2
    [0x1d, 0x60, 0x03, 0x10, 0x9c, 0xac, 0xa2, 0x8b, 0x17, 0x01, 0x98],#+3
    [0x1d, 0x60, 0x03, 0x11, 0xa8, 0xb9, 0xaf, 0x96, 0x19, 0x01, 0x98],#+4
];

_sensor_special_effects = [
    [0x06, 0x40, 0x2c, 0x08],#Normal
    [0x46, 0x40, 0x28, 0x08],#Negative
    [0x1e, 0x80, 0x80, 0x08],#Grayscale
    [0x1e, 0x80, 0xc0, 0x08],#Red Tint
    [0x1e, 0x60, 0x60, 0x08],#Green Tint
    [0x1e, 0xa0, 0x40, 0x08],#Blue Tint
    [0x1e, 0x40, 0xa0, 0x08],#Sepia
];

_sensor_regs_gamma0 = [
    [0x5480, 0x01],
    [0x5481, 0x08],
    [0x5482, 0x14],
    [0x5483, 0x28],
    [0x5484, 0x51],
    [0x5485, 0x65],
    [0x5486, 0x71],
    [0x5487, 0x7d],
    [0x5488, 0x87],
    [0x5489, 0x91],
    [0x548a, 0x9a],
    [0x548b, 0xaa],
    [0x548c, 0xb8],
    [0x548d, 0xcd],
    [0x548e, 0xdd],
    [0x548f, 0xea],
    [0x5490, 0x1d]
];

sensor_regs_gamma1 = [
    [0x5480, 0x1],
    [0x5481, 0x0],
    [0x5482, 0x1e],
    [0x5483, 0x3b],
    [0x5484, 0x58],
    [0x5485, 0x66],
    [0x5486, 0x71],
    [0x5487, 0x7d],
    [0x5488, 0x83],
    [0x5489, 0x8f],
    [0x548a, 0x98],
    [0x548b, 0xa6],
    [0x548c, 0xb8],
    [0x548d, 0xca],
    [0x548e, 0xd7],
    [0x548f, 0xe3],
    [0x5490, 0x1d]
];

sensor_regs_awb0 = [
    [0x5180, 0xff],
    [0x5181, 0xf2],
    [0x5182, 0x00],
    [0x5183, 0x14],
    [0x5184, 0x25],
    [0x5185, 0x24],
    [0x5186, 0x09],
    [0x5187, 0x09],
    [0x5188, 0x09],
    [0x5189, 0x75],
    [0x518a, 0x54],
    [0x518b, 0xe0],
    [0x518c, 0xb2],
    [0x518d, 0x42],
    [0x518e, 0x3d],
    [0x518f, 0x56],
    [0x5190, 0x46],
    [0x5191, 0xf8],
    [0x5192, 0x04],
    [0x5193, 0x70],
    [0x5194, 0xf0],
    [0x5195, 0xf0],
    [0x5196, 0x03],
    [0x5197, 0x01],
    [0x5198, 0x04],
    [0x5199, 0x12],
    [0x519a, 0x04],
    [0x519b, 0x00],
    [0x519c, 0x06],
    [0x519d, 0x82],
    [0x519e, 0x38]
];

#endif
