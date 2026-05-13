# The MIT License (MIT)
#
# Copyright (c) 2024 Jaimy Juliano, NHB Systems
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

"""
CircuitPython driver for the Analog Devices AD7124-4 24-bit Delta-Sigma ADC.

The AD7124-4 is a 4-channel (4 differential or 7 single-ended), 24-bit,
low-noise analog-to-digital converter with integrated PGA and digital filter.

This driver provides:
- Simple high-level API for ADC configuration and data readout
- Support for 8 independent setup configurations
- Automatic data and status readout
- Built-in temperature sensor support
- Full bridge (load cell, pressure) sensor support

Example usage:

.. code-block:: python

    import board
    import busio
    import digitalio
    from CircuitPy_AD7124 import nhb_ad7124
    
    # Create SPI bus and chip select
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    cs = digitalio.DigitalInOut(board.A0)
    
    # Initialize ADC
    adc = nhb_ad7124.Ad7124(spi, cs)
    
    # Configure and read
    adc.setup[0].set_config(nhb_ad7124.AD7124_Ref_Internal, 
                            nhb_ad7124.AD7124_Gain_1, True)
    adc.set_channel(0, 0, nhb_ad7124.AD7124_Input_AIN0, 
                    nhb_ad7124.AD7124_Input_AIN1, True)
    
    voltage = adc.read_volts(0)
    print(f"Channel 0: {voltage:.4f}V")
"""


#from machine import Pin, SPI
import board
import busio
import digitalio
import time
import adafruit_ticks
from adafruit_bus_device.spi_device import SPIDevice


try:
    from micropython import const
    upython = True
except ImportError:
    const = lambda x : x
    upython = False

_DEFAULT_TIMEOUT_MS = const(200)

_AD7124_MAX_CHANNELS = const(16) # Still not sure if this is really necessary

_AD7124_INVALID_VAL = const(-1) # Invalid argument
_AD7124_COMM_ERR    = const(-2) # Communication error on receive 
_AD7124_TIMEOUT     = const(-3) # A timeout has occurred 


_AD7124_RW = const(1)   # Read and Write
_AD7124_R  = const(2)   # Read only
_AD7124_W  = const(3)   # Write only

# AD7124 Register Map
_AD7124_COMM_REG      = const(0x00)
_AD7124_STATUS_REG    = const(0x00)
_AD7124_ADC_CTRL_REG  = const(0x01)
_AD7124_DATA_REG      = const(0x02)
_AD7124_IO_CTRL1_REG  = const(0x03)
_AD7124_IO_CTRL2_REG  = const(0x04)
_AD7124_ID_REG        = const(0x05)
_AD7124_ERR_REG       = const(0x06)
_AD7124_ERREN_REG     = const(0x07)
_AD7124_CH0_MAP_REG   = const(0x09)
_AD7124_CH1_MAP_REG   = const(0x0A)
_AD7124_CH2_MAP_REG   = const(0x0B)
_AD7124_CH3_MAP_REG   = const(0x0C)
_AD7124_CH4_MAP_REG   = const(0x0D)
_AD7124_CH5_MAP_REG   = const(0x0E)
_AD7124_CH6_MAP_REG   = const(0x0F)
_AD7124_CH7_MAP_REG   = const(0x10)
_AD7124_CH8_MAP_REG   = const(0x11)
_AD7124_CH9_MAP_REG   = const(0x12)
_AD7124_CH10_MAP_REG  = const(0x13)
_AD7124_CH11_MAP_REG  = const(0x14)
_AD7124_CH12_MAP_REG  = const(0x15)
_AD7124_CH13_MAP_REG  = const(0x16)
_AD7124_CH14_MAP_REG  = const(0x17)
_AD7124_CH15_MAP_REG  = const(0x18)
_AD7124_CFG0_REG      = const(0x19)
_AD7124_CFG1_REG      = const(0x1A)
_AD7124_CFG2_REG      = const(0x1B)
_AD7124_CFG3_REG      = const(0x1C)
_AD7124_CFG4_REG      = const(0x1D)
_AD7124_CFG5_REG      = const(0x1E)
_AD7124_CFG6_REG      = const(0x1F)
_AD7124_CFG7_REG      = const(0x20)
_AD7124_FILT0_REG     = const(0x21)
_AD7124_FILT1_REG     = const(0x22)
_AD7124_FILT2_REG     = const(0x23)
_AD7124_FILT3_REG     = const(0x24)
_AD7124_FILT4_REG     = const(0x25)
_AD7124_FILT5_REG     = const(0x26)
_AD7124_FILT6_REG     = const(0x27)
_AD7124_FILT7_REG     = const(0x28)
_AD7124_OFFS0_REG     = const(0x29)
_AD7124_OFFS1_REG     = const(0x2A)
_AD7124_OFFS2_REG     = const(0x2B)
_AD7124_OFFS3_REG     = const(0x2C)
_AD7124_OFFS4_REG     = const(0x2D)
_AD7124_OFFS5_REG     = const(0x2E)
_AD7124_OFFS6_REG     = const(0x2F)
_AD7124_OFFS7_REG     = const(0x30)
_AD7124_GAIN0_REG     = const(0x31)
_AD7124_GAIN1_REG     = const(0x32)
_AD7124_GAIN2_REG     = const(0x33)
_AD7124_GAIN3_REG     = const(0x34)
_AD7124_GAIN4_REG     = const(0x35)
_AD7124_GAIN5_REG     = const(0x36)
_AD7124_GAIN6_REG     = const(0x37)
_AD7124_GAIN7_REG     = const(0x38)

# Communication Register bits 
_AD7124_COMM_REG_WEN  = const(0 << 7)
_AD7124_COMM_REG_WR   = const(0 << 6)
_AD7124_COMM_REG_RD   = const(1 << 6)
#_AD7124_COMM_REG_RA(x)  ((x) & 0x3F)
_AD7124_COMM_REG_RA = lambda x: ((x) & 0x3F)

# Status Register bits 
_AD7124_STATUS_REG_RDY         = const(1 << 7)
_AD7124_STATUS_REG_ERROR_FLAG  = const(1 << 6)
_AD7124_STATUS_REG_POR_FLAG    = const(1 << 4)
_AD7124_STATUS_REG_CH_ACTIVE = lambda x: ((x) & 0xF)

# ADC_Control Register bits
_AD7124_ADC_CTRL_REG_DOUT_RDY_DEL   = const(1 << 12)
_AD7124_ADC_CTRL_REG_CONT_READ      = const(1 << 11)
_AD7124_ADC_CTRL_REG_DATA_STATUS    = const(1 << 10)
_AD7124_ADC_CTRL_REG_CS_EN          = const(1 << 9)
_AD7124_ADC_CTRL_REG_REF_EN         = const(1 << 8)
_AD7124_ADC_CTRL_REG_POWER_MODE     = lambda x: (((x) & 0x3) << 6)
_AD7124_ADC_CTRL_REG_MODE           = lambda x: (((x) & 0xF) << 2)
_AD7124_ADC_CTRL_REG_CLK_SEL        = lambda x: (((x) & 0x3) << 0)

# IO_Control_1 Register bits
_AD7124_IO_CTRL1_REG_GPIO_DAT2     = const(1 << 23)
_AD7124_IO_CTRL1_REG_GPIO_DAT1     = const(1 << 22)
_AD7124_IO_CTRL1_REG_GPIO_CTRL2    = const(1 << 19)
_AD7124_IO_CTRL1_REG_GPIO_CTRL1    = const(1 << 18)
_AD7124_IO_CTRL1_REG_PDSW          = const(1 << 15)
_AD7124_IO_CTRL1_REG_IOUT1         = lambda x: (((x) & 0x7) << 11)
_AD7124_IO_CTRL1_REG_IOUT0         = lambda x: (((x) & 0x7) << 8)
_AD7124_IO_CTRL1_REG_IOUT_CH1      = lambda x: (((x) & 0xF) << 4)
_AD7124_IO_CTRL1_REG_IOUT_CH0      = lambda x: (((x) & 0xF) << 0)

# IO_Control_1 AD7124-8 specific bits
_AD7124_8_IO_CTRL1_REG_GPIO_DAT4     = const(1 << 23)
_AD7124_8_IO_CTRL1_REG_GPIO_DAT3     = const(1 << 22)
_AD7124_8_IO_CTRL1_REG_GPIO_DAT2     = const(1 << 21)
_AD7124_8_IO_CTRL1_REG_GPIO_DAT1     = const(1 << 20)
_AD7124_8_IO_CTRL1_REG_GPIO_CTRL4    = const(1 << 19)
_AD7124_8_IO_CTRL1_REG_GPIO_CTRL3    = const(1 << 18)
_AD7124_8_IO_CTRL1_REG_GPIO_CTRL2    = const(1 << 17)
_AD7124_8_IO_CTRL1_REG_GPIO_CTRL1    = const(1 << 16)

# IO_Control_2 Register bits
_AD7124_IO_CTRL2_REG_GPIO_VBIAS7   = const(1 << 15)
_AD7124_IO_CTRL2_REG_GPIO_VBIAS6   = const(1 << 14)
_AD7124_IO_CTRL2_REG_GPIO_VBIAS5   = const(1 << 11)
_AD7124_IO_CTRL2_REG_GPIO_VBIAS4   = const(1 << 10)
_AD7124_IO_CTRL2_REG_GPIO_VBIAS3   = const(1 << 5)
_AD7124_IO_CTRL2_REG_GPIO_VBIAS2   = const(1 << 4)
_AD7124_IO_CTRL2_REG_GPIO_VBIAS1   = const(1 << 1)
_AD7124_IO_CTRL2_REG_GPIO_VBIAS0   = const(1 << 0)

# IO_Control_2 AD7124-8 specific bits
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS15  = const(1 << 15)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS14  = const(1 << 14)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS13  = const(1 << 13)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS12  = const(1 << 12)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS11  = const(1 << 11)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS10  = const(1 << 10)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS9   = const(1 << 9)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS8   = const(1 << 8)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7   = const(1 << 7)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS6   = const(1 << 6)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS5   = const(1 << 5)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS4   = const(1 << 4)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS3   = const(1 << 3)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS2   = const(1 << 2)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS1   = const(1 << 1)
_AD7124_8_IO_CTRL2_REG_GPIO_VBIAS0   = const(1 << 0)

# ID Register bits
_AD7124_ID_REG_DEVICE_ID   = lambda x: (((x) & 0xF) << 4)
_AD7124_ID_REG_SILICON_REV = lambda x: (((x) & 0xF) << 0)

# Error Register bits
_AD7124_ERR_REG_LDO_CAP_ERR        = const(1 << 19)
_AD7124_ERR_REG_ADC_CAL_ERR        = const(1 << 18)
_AD7124_ERR_REG_ADC_CONV_ERR       = const(1 << 17)
_AD7124_ERR_REG_ADC_SAT_ERR        = const(1 << 16)
_AD7124_ERR_REG_AINP_OV_ERR        = const(1 << 15)
_AD7124_ERR_REG_AINP_UV_ERR        = const(1 << 14)
_AD7124_ERR_REG_AINM_OV_ERR        = const(1 << 13)
_AD7124_ERR_REG_AINM_UV_ERR        = const(1 << 12)
_AD7124_ERR_REG_REF_DET_ERR        = const(1 << 11)
_AD7124_ERR_REG_DLDO_PSM_ERR       = const(1 << 9)
_AD7124_ERR_REG_ALDO_PSM_ERR       = const(1 << 7)
_AD7124_ERR_REG_SPI_IGNORE_ERR     = const(1 << 6)
_AD7124_ERR_REG_SPI_SLCK_CNT_ERR   = const(1 << 5)
_AD7124_ERR_REG_SPI_READ_ERR       = const(1 << 4)
_AD7124_ERR_REG_SPI_WRITE_ERR      = const(1 << 3)
_AD7124_ERR_REG_SPI_CRC_ERR        = const(1 << 2)
_AD7124_ERR_REG_MM_CRC_ERR         = const(1 << 1)
_AD7124_ERR_REG_ROM_CRC_ERR        = const(1 << 0)

# Error_En Register bits
_AD7124_ERREN_REG_MCLK_CNT_EN           = const(1 << 22)
_AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN   = const(1 << 21)
_AD7124_ERREN_REG_LDO_CAP_CHK           = lambda x: (((x) & 0x3) << 19)
_AD7124_ERREN_REG_ADC_CAL_ERR_EN        = const(1 << 18)
_AD7124_ERREN_REG_ADC_CONV_ERR_EN       = const(1 << 17)
_AD7124_ERREN_REG_ADC_SAT_ERR_EN        = const(1 << 16)
_AD7124_ERREN_REG_AINP_OV_ERR_EN        = const(1 << 15)
_AD7124_ERREN_REG_AINP_UV_ERR_EN        = const(1 << 14)
_AD7124_ERREN_REG_AINM_OV_ERR_EN        = const(1 << 13)
_AD7124_ERREN_REG_AINM_UV_ERR_EN        = const(1 << 12)
_AD7124_ERREN_REG_REF_DET_ERR_EN        = const(1 << 11)
_AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN = const(1 << 10)
_AD7124_ERREN_REG_DLDO_PSM_ERR_ERR      = const(1 << 9)
_AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN = const(1 << 8)
_AD7124_ERREN_REG_ALDO_PSM_ERR_EN       = const(1 << 7)
_AD7124_ERREN_REG_SPI_IGNORE_ERR_EN     = const(1 << 6)
_AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN   = const(1 << 5)
_AD7124_ERREN_REG_SPI_READ_ERR_EN       = const(1 << 4)
_AD7124_ERREN_REG_SPI_WRITE_ERR_EN      = const(1 << 3)
_AD7124_ERREN_REG_SPI_CRC_ERR_EN        = const(1 << 2)
_AD7124_ERREN_REG_MM_CRC_ERR_EN         = const(1 << 1)
_AD7124_ERREN_REG_ROM_CRC_ERR_EN        = const(1 << 0)

# Channel Registers 0-15 bits
_AD7124_CH_MAP_REG_CH_ENABLE    = const(1 << 15)
_AD7124_CH_MAP_REG_SETUP        = lambda x: (((x) & 0x7) << 12)
_AD7124_CH_MAP_REG_AINP         = lambda x: (((x) & 0x1F) << 5)
_AD7124_CH_MAP_REG_AINM         = lambda x: (((x) & 0x1F) << 0)

# Configuration Registers 0-7 bits
_AD7124_CFG_REG_BIPOLAR     = const(1 << 11)
_AD7124_CFG_REG_BURNOUT     = lambda x: (((x) & 0x3) << 9)
_AD7124_CFG_REG_REF_BUFP    = const(1 << 8)
_AD7124_CFG_REG_REF_BUFM    = const(1 << 7)
_AD7124_CFG_REG_AIN_BUFP    = const(1 << 6)
_AD7124_CFG_REG_AINN_BUFM   = const(1 << 5)
_AD7124_CFG_REG_REF_SEL     = lambda x: ((x) & 0x3) << 3
_AD7124_CFG_REG_PGA         = lambda x: (((x) & 0x7) << 0)

# Filter Register 0-7 bits
_AD7124_FILT_REG_FILTER            = lambda x: (((x) & 0x7) << 21)
_AD7124_FILT_REG_REJ60             = const(1 << 20)
_AD7124_FILT_REG_POST_FILTER       = lambda x: (((x) & 0x7) << 17)
_AD7124_FILT_REG_SINGLE_CYCLE      = const(1 << 16)
_AD7124_FILT_REG_FS                = lambda x: (((x) & 0x7FF) << 0)


# AD7124 Constants

_AD7124_CRC8_POLYNOMIAL_REPRESENTATION = const(0x07)      # x8 + x2 + x + 1 
_AD7124_DISABLE_CRC                    = const(0)
_AD7124_USE_CRC                        = const(1)



# I would prefer these were [fake] enums, but making them classes seems like it
# would waste memory, so I'll just do more constants. 

# Operating Modes
AD7124_OpMode_Continuous =                  const(0) # Continuous conversion mode (default). In continuous conversion mode, the ADC continuously performs conversions and places the result in the data register.
AD7124_OpMode_SingleConv =                  const(1) # Single conversion mode. When single conversion mode is selected, the ADC powers up and performs a single conversion on the selected channel.
AD7124_OpMode_Standby =                     const(2) # Standby mode. In standby mode, all sections of the AD7124 can be powered down except the LDOs.
AD7124_OpMode_PowerDown =                   const(3) # Power-down mode. In power-down mode, all the AD7124 circuitry is powered down, including the current sources, power switch, burnout currents, bias voltage generator, and clock circuitry.
AD7124_OpMode_Idle =                        const(4) # Idle mode. In idle mode, the ADC filter and modulator are held in a reset state even though the modulator clocks continue to be provided.
AD7124_OpMode_InternalOffsetCalibration =   const(5) # Internal zero-scale (offset) calibration. An internal short is automatically connected to the input. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
AD7124_OpMode_InternalGainCalibration =     const(6) # Internal full-scale (gain) calibration. A full-scale input voltage is automatically connected to the selected analog input for this calibration. */
AD7124_OpMode_SystemOffsetCalibration =     const(7) # System zero-scale (offset) calibration. Connect the system zero-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
AD7124_OpMode_SystemGainCalibration =       const(8) # System full-scale (gain) calibration. Connect the system full-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete.

# Power Modes    
AD7124_LowPower = 	const(0)
AD7124_MidPower = 	const(1)
AD7124_FullPower = 	const(2)

# Clock Sources
AD7124_Clk_Internal = const(0)             # internal 614.4 kHz clock. The internal clock is not available at the CLK pin.
AD7124_Clk_InternalWithOutput = const(1)   # internal 614.4 kHz clock. This clock is available at the CLK pin.
AD7124_Clk_External = const(2)             # external 614.4 kHz clock.
AD7124_Clk_ExternalDiv4 = const(3)         # external clock. The external clock is divided by 4 within the AD7124.

# Input Selection
AD7124_Input_AIN0 = const(0)
AD7124_Input_AIN1 = const(1)
AD7124_Input_AIN2 = const(2)
AD7124_Input_AIN3 = const(3)
AD7124_Input_AIN4 = const(4)
AD7124_Input_AIN5 = const(5)
AD7124_Input_AIN6 = const(6)
AD7124_Input_AIN7 = const(7)
AD7124_Input_AIN8 = const(8)
AD7124_Input_AIN9 = const(9)
AD7124_Input_AIN10 = const(10)
AD7124_Input_AIN11 = const(11)
AD7124_Input_AIN12 = const(12)
AD7124_Input_AIN13 = const(13)
AD7124_Input_AIN14 = const(14)
AD7124_Input_AIN15 = const(15)
AD7124_Input_TEMP  = const(16)     # Temperature sensor (internal)
AD7124_Input_AVSS  = const(17)     # Connect to AVss
AD7124_Input_REF   = const(18)     # Connect to Internal reference
AD7124_Input_DGND  = const(19)     # Connect to DGND.
AD7124_Input_AVDD6P = const(20)    # (AVdd − AVss)/6+. Use in conjunction with (AVdd − AVss)/6− to monitor supply AVdd − AVss .
AD7124_Input_AVDD6M = const(21)    # (AVdd − AVss)/6−. Use in conjunction with (AVdd − AVss)/6+ to monitor supply AVdd − AVss .
AD7124_Input_IOVDD6P = const(22)   # (IOVdd − DGND)/6+. Use in conjunction with (IOVdd − DGND)/6− to monitor IOVdd − DGND.
AD7124_Input_IOVDD6M = const(23)   # (IOVdd − DGND)/6−. Use in conjunction with (IOVdd − DGND)/6+ to monitor IOVdd − DGND.
AD7124_Input_ALDO6P  = const(24)   # (ALDO − AVss)/6+. Use in conjunction with (ALDO − AVss)/6− to monitor the analog LDO.
AD7124_Input_ALDO6M  = const(25)   # (ALDO − AVss)/6−. Use in conjunction with (ALDO − AVss)/6+ to monitor the analog LDO.
AD7124_Input_DLDO6P  = const(26)   # (DLDO − DGND)/6+. Use in conjunction with (DLDO − DGND)/6− to monitor the digital LDO.
AD7124_Input_DLDO6M  = const(27)   # (DLDO − DGND)/6−. Use in conjunction with (DLDO − DGND)/6+ to monitor the digital LDO.
AD7124_Input_V20mVP  = const(28)   # V_20MV_P. Use in conjunction with V_20MV_M to apply a 20 mV p-p signal to the ADC.
AD7124_Input_V20mVM  = const(29)   # V_20MV_M. Use in conjunction with V_20MV_P to apply a 20 mV p-p signal to the ADC.

# ***** SUGGESTION FROM GHCP *****
# These two are only available on the AD7124-8
AD7124_Input_VREFP   = const(30)   # VREFP. Use in conjunction with VREFM to apply an external reference voltage to the ADC. 
AD7124_Input_VREFM   = const(31)   # VREFM. Use in conjunction with VREFP to apply an external reference voltage to the ADC.


# Gain Selection
AD7124_Gain_1 = const(0)   	# Gain 1, Input Range When VREF = 2.5 V: ±2.5 V
AD7124_Gain_2 = const(1)   	# Gain 2, Input Range When VREF = 2.5 V: ±1.25 V
AD7124_Gain_4 = const(2)   	# Gain 4, Input Range When VREF = 2.5 V: ± 625 mV
AD7124_Gain_8 = const(3)   	# Gain 8, Input Range When VREF = 2.5 V: ±312.5 mV
AD7124_Gain_16 = const(4)  	# Gain 16, Input Range When VREF = 2.5 V: ±156.25 mV
AD7124_Gain_32 = const(5)  	# Gain 32, Input Range When VREF = 2.5 V: ±78.125 mV
AD7124_Gain_64 = const(6)  	# Gain 64, Input Range When VREF = 2.5 V: ±39.06 mV
AD7124_Gain_128 = const(7) 	# Gain 128, Input Range When VREF = 2.5 V: ±19.53 mV

#NOTE: These are different for the AD7124-8
# VBias 
AD7124_VBias_AIN0 = const(0x00)
AD7124_VBias_AIN1 = const(0x01)
AD7124_VBias_AIN2 = const(0x04)
AD7124_VBias_AIN3 = const(0x05)
AD7124_VBias_AIN4 = const(0x0A)
AD7124_VBias_AIN5 = const(0x0B)
AD7124_VBias_AIN6 = const(0x0E)
AD7124_VBias_AIN7 = const(0x0F)

# Reference Sources    
AD7124_Ref_ExtRef1  = const(0x00)
AD7124_Ref_ExtRef2  = const(0x01)
AD7124_Ref_Internal = const(0x02)
AD7124_Ref_Avdd     = const(0x03)

# Filter Options
AD7124_Filter_SINC4 = const(0x00)  # SINC4 Filter - Default after reset. This filter gives excellent noise performance over the complete range of output data rates. It also gives the best 50 Hz/60 Hz rejection, but it has a long settling time.
AD7124_Filter_SINC3 = const(0x02)  # SINC3 Filter - This filter has good noise performance, moderate settling time, and moderate 50 Hz and 60 Hz (±1 Hz) rejection.
AD7124_Filter_FAST4 = const(0x04)  # Fast settling + Sinc4
AD7124_Filter_FAST3 = const(0x05)  # Fast settling + Sinc3
AD7124_Filter_POST  = const(0x07)  # Post filter enable - The post filters provide rejection of 50 Hz and 60 Hz simultaneously and allow the user to trade off settling time and rejection. These filters can operate up to 27.27 SPS or can reject up to 90 dB of 50 Hz ± 1 Hz and 60 Hz ± 1 Hz interference

# Post Filter Options
AD7124_PostFilter_NoPost = const(0) # No Post Filter (Default value)
AD7124_PostFilter_dB47   = const(2) # Rejection at 50 Hz and 60 Hz ± 1 Hz: 47 dB, Output Data Rate (SPS): 27.27 Hz
AD7124_PostFilter_dB62   = const(3) # Rejection at 50 Hz and 60 Hz ± 1 Hz: 62 dB, Output Data Rate (SPS): 25 Hz
AD7124_PostFilter_dB86   = const(5) # Rejection at 50 Hz and 60 Hz ± 1 Hz: 86 dB, Output Data Rate (SPS): 20 Hz
AD7124_PostFilter_dB92   = const(6) # Rejection at 50 Hz and 60 Hz ± 1 Hz: 92 dB, Output Data Rate (SPS): 16.7 Hz

# Burnout Current Options
AD7124_Burnout_Off    = const(0)  # burnout current source off (default).
AD7124_Burnout_500nA  = const(1)  # burnout current source on, 0.5 μA.
AD7124_Burnout_2uA    = const(2)  # burnout current source on, 2 μA.
AD7124_Burnout_4uA    = const(3)  # burnout current source on, 4 μA.


# Excitation Currents Options - Not used yet.
AD7124_ExCurrent_Off    = const(0x00)
AD7124_ExCurrent_50uA   = const(0x01)
AD7124_ExCurrent_100uA  = const(0x02)
AD7124_ExCurrent_250uA  = const(0x03)
AD7124_ExCurrent_500uA  = const(0x04)
AD7124_ExCurrent_750uA  = const(0x05)
AD7124_ExCurrent_1mA    = const(0x06)


# Device register info
class Ad7124_Register:
    """
    Internal class representation of an AD7124 register.
    
    Stores register metadata including address, current value, size, and access mode.
    This is used internally for register read/write operations.
    
    :param int addr: Register address (0x00-0x38)
    :param int value: Current register value
    :param int size: Register size in bytes (1-3)
    :param int rw: Register access mode (_AD7124_R, _AD7124_W, or _AD7124_RW)
    """
    def __init__(self, addr: int, value: int, size: int, rw: int) -> None:
        self.addr = addr
        self.value = value
        self.size = size
        self.rw = rw

class Ad7124SetupVals:
    """
    Internal class to hold configuration values for an AD7124 setup.
    
    Each setup can be independently configured with different reference sources,
    gains, filters, and calibration coefficients.
    """
    def __init__(self) -> None:
        self.ref = AD7124_Ref_ExtRef1
        self.gain = AD7124_Gain_1
        self.bipolar = True
        self.burnout = AD7124_Burnout_Off
        self.filter = AD7124_Filter_SINC4
        self.fs = 0
        self.post_filter = AD7124_PostFilter_NoPost
        self.rej60 = False
        self.single_cycle = False
        self.offset_coeff = 0
        self.gain_coeff = 0
        self.refV = 2.500

class Ad7124Setup:
    """
    Manages a single AD7124 setup configuration.
    
    The AD7124 has 8 independent setups that can be configured with different
    reference sources, gains, filters, and calibration values. Each channel can
    be assigned to use a particular setup.
    
    :param Ad7124 driver: Reference to parent Ad7124 driver instance
    :param int index: Setup number (0-7)
    """

    def __init__(self, driver: "Ad7124", index: int) -> None:
        self._setup_number = index
        self._driver = driver
        self.setup_values = Ad7124SetupVals()
    
    def set_config(self, ref_source: int, gain: int, bipolar: bool,
                   burnout: int = AD7124_Burnout_Off,
                   exRefV: float = 2.50) -> int:
        """
        Configure the setup's analog input and reference settings.
        
        :param int ref_source: Reference source (AD7124_Ref_ExtRef1, AD7124_Ref_ExtRef2,
                                AD7124_Ref_Internal, AD7124_Ref_Avdd)
        :param int gain: PGA gain setting (AD7124_Gain_1 through AD7124_Gain_128)
        :param bool bipolar: True for bipolar ±Vref/Gain, False for unipolar 0 to Vref/Gain
        :param int burnout: Burnout current source setting (default: AD7124_Burnout_Off)
        :param float exRefV: External reference voltage in volts (default: 2.50V)
        
        :return: 0 on success, error code on failure
        """

        self.setup_values.ref = ref_source
        self.setup_values.gain = gain
        self.setup_values.bipolar = bipolar
        self.setup_values.burnout #not yet supported
        self.setup_values.refV = exRefV

        #Offset to config reg group
        reg = self._setup_number + _AD7124_CFG0_REG

        self._driver.regs[reg].value = _AD7124_CFG_REG_REF_SEL(ref_source) | \
                                       _AD7124_CFG_REG_PGA(gain) | \
                                       (_AD7124_CFG_REG_BIPOLAR if bipolar else 0) | \
                                       _AD7124_CFG_REG_BURNOUT(burnout) | \
                                       _AD7124_CFG_REG_REF_BUFP | _AD7124_CFG_REG_REF_BUFM | \
                                       _AD7124_CFG_REG_AIN_BUFP | _AD7124_CFG_REG_AINN_BUFM

        #print(f"Writing {hex(self._driver.regs[reg].value)} to adc config register")
        return self._driver.write_register(self._driver.regs[reg])
        

    def set_filter(self, filter: int, fs: int,
                   post_filter: int = AD7124_PostFilter_NoPost,
                   rej60: bool = False, single_cycle: bool = False) -> int:
        """
        Configure the setup's filter type and output data rate.
        
        :param int filter: Filter type (AD7124_Filter_SINC4, AD7124_Filter_SINC3, etc.)
        :param int fs: Filter select bits for output data rate (1-2047)
        :param int post_filter: Post-filter option (default: AD7124_PostFilter_NoPost)
        :param bool rej60: Enable simultaneous 50/60 Hz rejection (default: False)
        :param bool single_cycle: Enable single cycle conversion mode (default: False)
        
        :return: 0 on success, error code on failure
        """
        
        self.setup_values.filter = filter
        self.setup_values.fs = fs
        self.setup_values.post_filter = post_filter
        self.setup_values.rej60 = rej60
        self.setup_values.single_cycle = single_cycle

        #Offset to filter reg group
        reg = self._setup_number + _AD7124_FILT0_REG

        self._driver.regs[reg].value = _AD7124_FILT_REG_FILTER(filter) | \
                                       _AD7124_FILT_REG_POST_FILTER(post_filter) | \
                                       _AD7124_FILT_REG_FS(fs) | \
                                       (_AD7124_FILT_REG_REJ60 if rej60 else 0) | \
                                       (_AD7124_FILT_REG_SINGLE_CYCLE if single_cycle else 0)

        #print(f"Writing {hex(self._driver.regs[reg].value)} to the {hex(reg)} filter register")
        return self._driver.write_register(self._driver.regs[reg])

               
    def set_offset_cal(self, value: int):
        '''
        Sets the offset calibration value for a setup
        NOT YET IMPLEMENTED
        '''

        pass
    
    def set_gain_cal(self, value: int):
        '''
        Sets the gain calibration value for a setup
        NOT YET IMPLEMENTED
        '''
        pass  
    

class Ad7124:
    def __init__(self, spi, cs_pin, baudrate=4000000):
        
        '''
        Initializes the AD7124 class and set up the SPI interface using SPIDevice.
        
        Args:
            spi: A pre-configured busio.SPI object
            cs_pin: A DigitalInOut object for chip select
            baudrate: SPI clock frequency (default = 4000000)
        '''

        # Create SPIDevice instance for bus management and locking
        self.spi_device = SPIDevice(
            spi, 
            cs_pin, 
            cs_active_value=False,
            baudrate=baudrate, 
            polarity=1, 
            phase=1
        )

        self._crc_enabled = False
        self.opmode = AD7124_OpMode_SingleConv

        self.setup = []    
        
        for i in range(8):
            self.setup.append(Ad7124Setup(self, i))
        
        
        # Pre-allocated buffers for SPI transactions
        self.spi_buffer = bytearray(8)
        self.spi_buf_mv = memoryview(self.spi_buffer)  
        self.spi_buf_2 = self.spi_buf_mv[:2]
        self.spi_buf_3 = self.spi_buf_mv[:3]
        self.spi_buf_4 = self.spi_buf_mv[:4]
        self.spi_buf_5 = self.spi_buf_mv[:5]
        self.spi_buffs = (self.spi_buf_2, self.spi_buf_3, self.spi_buf_4, self.spi_buf_5)
        
        # Temporary reg struct for data with extra byte to hold status bits
        self.reg_data_and_status = Ad7124_Register(0x02, 0x0000, 4, 2)
        
        # Initialize list of register values. Initially set to POR values
        self.regs = [
            Ad7124_Register(0x00, 0x00, 1, 2),     # Status
            Ad7124_Register(0x01, 0x0000, 2, 1),   # ADC_Control
            Ad7124_Register(0x02, 0x0000, 3, 2),   # Data
            Ad7124_Register(0x03, 0x0000, 3, 1),   # IOCon1
            Ad7124_Register(0x04, 0x0000, 2, 1),   # IOCon2
            Ad7124_Register(0x05, 0x02, 1, 2),     # ID
            Ad7124_Register(0x06, 0x0000, 3, 2),   # Error
            Ad7124_Register(0x07, 0x0044, 3, 1),   # Error_En
            Ad7124_Register(0x08, 0x00, 1, 2),     # Mclk_Count

            Ad7124_Register(0x09, 0x8001, 2, 1),   # Channel_0
            Ad7124_Register(0x0A, 0x0001, 2, 1),   # Channel_1
            Ad7124_Register(0x0B, 0x0001, 2, 1),   # Channel_2 
            Ad7124_Register(0x0C, 0x0001, 2, 1),   # Channel_3 
            Ad7124_Register(0x0D, 0x0001, 2, 1),   # Channel_4 
            Ad7124_Register(0x0E, 0x0001, 2, 1),   # Channel_5 
            Ad7124_Register(0x0F, 0x0001, 2, 1),   # Channel_6 
            Ad7124_Register(0x10, 0x0001, 2, 1),   # Channel_7 
            Ad7124_Register(0x11, 0x0001, 2, 1),   # Channel_8 
            Ad7124_Register(0x12, 0x0001, 2, 1),   # Channel_9 
            Ad7124_Register(0x13, 0x0001, 2, 1),   # Channel_10
            Ad7124_Register(0x14, 0x0001, 2, 1),   # Channel_11
            Ad7124_Register(0x15, 0x0001, 2, 1),   # Channel_12
            Ad7124_Register(0x16, 0x0001, 2, 1),   # Channel_13
            Ad7124_Register(0x17, 0x0001, 2, 1),   # Channel_14
            Ad7124_Register(0x18, 0x0001, 2, 1),   # Channel_15

            Ad7124_Register(0x19, 0x0860, 2, 1),   # Config_0 
            Ad7124_Register(0x1A, 0x0860, 2, 1),   # Config_1 
            Ad7124_Register(0x1B, 0x0860, 2, 1),   # Config_2 
            Ad7124_Register(0x1C, 0x0860, 2, 1),   # Config_3 
            Ad7124_Register(0x1D, 0x0860, 2, 1),   # Config_4 
            Ad7124_Register(0x1E, 0x0860, 2, 1),   # Config_5 
            Ad7124_Register(0x1F, 0x0860, 2, 1),   # Config_6 
            Ad7124_Register(0x20, 0x0860, 2, 1),   # Config_7 

            Ad7124_Register(0x21, 0x060180, 3, 1), # Filter_0 
            Ad7124_Register(0x22, 0x060180, 3, 1), # Filter_1 
            Ad7124_Register(0x23, 0x060180, 3, 1), # Filter_2 
            Ad7124_Register(0x24, 0x060180, 3, 1), # Filter_3 
            Ad7124_Register(0x25, 0x060180, 3, 1), # Filter_4 
            Ad7124_Register(0x26, 0x060180, 3, 1), # Filter_5 
            Ad7124_Register(0x27, 0x060180, 3, 1), # Filter_6 
            Ad7124_Register(0x28, 0x060180, 3, 1), # Filter_7 

            Ad7124_Register(0x29, 0x800000, 3, 1), # Offset_0 
            Ad7124_Register(0x2A, 0x800000, 3, 1), # Offset_1 
            Ad7124_Register(0x2B, 0x800000, 3, 1), # Offset_2 
            Ad7124_Register(0x2C, 0x800000, 3, 1), # Offset_3 
            Ad7124_Register(0x2D, 0x800000, 3, 1), # Offset_4 
            Ad7124_Register(0x2E, 0x800000, 3, 1), # Offset_5 
            Ad7124_Register(0x2F, 0x800000, 3, 1), # Offset_6 
            Ad7124_Register(0x30, 0x800000, 3, 1), # Offset_7 
            
            Ad7124_Register(0x31, 0x500000, 3, 1), # Gain_0 
            Ad7124_Register(0x32, 0x500000, 3, 1), # Gain_1 
            Ad7124_Register(0x33, 0x500000, 3, 1), # Gain_2 
            Ad7124_Register(0x34, 0x500000, 3, 1), # Gain_3 
            Ad7124_Register(0x35, 0x500000, 3, 1), # Gain_4 
            Ad7124_Register(0x36, 0x500000, 3, 1), # Gain_5 
            Ad7124_Register(0x37, 0x500000, 3, 1), # Gain_6 
            Ad7124_Register(0x38, 0x500000, 3, 1), # Gain_7
        ]
    
        self.reset()
        time.sleep(0.1)  
    
        
    def reset(self):
        '''
        Write 64 1s to reset the chip.
        This currently doesn't work as expected. When called, everything
        read after the call has its last bit set to 1. It's very strange and 
        I just can't figure out what is going on. Actually, on a fresh read of 
        the datasheet, it looks like this may not be necessary anyway. It looks
        like (1) simply bringing CS high resets the communications interface 
        anyway, and (2) writing 64 1s without asserting CS may reset the
        device, which if it works, does not seem to mess up future reads. This
        will need to be tested, but for now I am clocking it out without CS and
        am moving on
        '''        
        
        buff = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
        
        with self.spi_device as spi:
            spi.write(buff)  # Write without using standard transaction (CS stays high)
           
        return self.wait_for_power_on(100)
    
    
    def get_ID(self) -> int:
        """
        Read and return the AD7124 device ID.
        
        :return: Device ID value from the ID register
        """
        self.read_register(self.regs[_AD7124_ID_REG])
        return self.regs[_AD7124_ID_REG].value
    
    @property
    def status(self) -> int:
        """
        Read the current ADC status register.
        
        Contains information about the active channel, power-on reset status,
        and error conditions.
        
        :return: Current status register value
        """
        self.read_register(self.regs[_AD7124_STATUS_REG])
        return self.regs[_AD7124_STATUS_REG].value
    
    @property  
    def is_ready(self) -> bool:
        """
        Check if conversion data is ready.
        
        :return: True if data is ready to be read, False otherwise
        """
        self.read_register(self.regs[_AD7124_STATUS_REG])
        return not bool(self.regs[_AD7124_STATUS_REG].value & _AD7124_STATUS_REG_RDY)
        
    
    def setPWRSW(self, enabled: bool) -> int:
        """
        Control the low-side power switch (PSW pin).
        
        On NHB Systems AD7124 boards, this pin controls a 2.5V linear regulator
        for sensor excitation. Can be used in custom designs to switch external
        power loads (limited to 30mA).
        
        :param bool enabled: True to close switch, False to open switch
        
        :return: 0 on success, error code on failure
        """
        self.regs[_AD7124_IO_CTRL1_REG].value &= ~_AD7124_IO_CTRL1_REG_PDSW
        
        if enabled:
            self.regs[_AD7124_IO_CTRL1_REG].value |= _AD7124_IO_CTRL1_REG_PDSW
        
        return self.write_register(self.regs[_AD7124_IO_CTRL1_REG])
    

    def wait_for_power_on(self, timeout: int = None) -> bool:
        """
        Wait for the AD7124 power-on reset (POR) to complete.
        
        :param int timeout: Optional timeout in milliseconds (None = no timeout)
        
        :return: True if POR completed, False/error code if timeout occurred
        """
        powered_on = False
        start_time = adafruit_ticks.ticks_ms()

        while True:
           
            self.read_register(self.regs[_AD7124_STATUS_REG])

            powered_on = not (self.regs[_AD7124_STATUS_REG].value & _AD7124_STATUS_REG_POR_FLAG) #POR value = 0x00

            if powered_on:
                return True

            if timeout is None:
                # Wait indefinitely without delay
                continue

            if adafruit_ticks.ticks_diff(adafruit_ticks.ticks_ms(), start_time) >= timeout:
                return _AD7124_TIMEOUT  # Timeout

            # Delay for 1 millisecond and decrement timeout
            time.sleep(0.001) # No sleep_ms() in CircuitPy
            #timeout -= 1

        # Should never reach here
        raise RuntimeError("Unexpected error!")


    def set_vbias(self, vBiasPin: int, enabled: bool) -> int:
        """
        Enable or disable bias voltage on a pin.
        
        Bias voltage is necessary for truly bipolar measurements (e.g., thermocouples).
        
        :param int vBiasPin: VBIAS pin to control
        :param bool enabled: True to enable bias, False to disable
        
        :return: 0 on success, error code on failure
        """    

        self.regs[_AD7124_IO_CTRL2_REG].value &= ~(1 << vBiasPin)

        if enabled:        
            self.regs[_AD7124_IO_CTRL2_REG].value |= (1 << vBiasPin)        

        return self.write_register(self.regs[_AD7124_IO_CTRL2_REG])
    


    #########################################################
    # WIP

    def read_raw(self, ch: int) -> int:
        """
        Read raw ADC counts from a channel.
        
        Returns the 24-bit unsigned ADC conversion result without any scaling.
        
        :param int ch: Channel number to read (0-15)
        
        :return: Raw ADC counts (0 to 16777215), or negative error code
        """
        cur_ch = self.current_channel() # <-- Remember, this only works properly when using Data + Status mode
        
        #print(f"<read_raw> cur_ch = {cur_ch}, ch = {ch}")
        
        if ch != cur_ch:
  
            #Disable previous channel if different
            ret = self.enable_channel(cur_ch, False)
            if (ret < 0):    
                return ret    

            # Moved here so only called if channel changed
            if(self.opmode == AD7124_OpMode_SingleConv):
    
                ret = self.enable_channel(ch, True)
                if (ret < 0):      
                    return ret      

                # write the mode register again to start the conversion.
                ret = self.set_mode(AD7124_OpMode_SingleConv)
                if (ret < 0):
                
                    return ret
                             
            # If in continuous mode, just enable the channel we want to read now
            else:
            
                ret = self.enable_channel(ch, True)
                if (ret < 0):
                    return ret            
            
  
        # If no channel change, just call setMode
        else:
        
            # If we are in single conversion mode, we need to write the mode  
            # register again to start the conversion.
            if(self.opmode == AD7124_OpMode_SingleConv):
               
                ret = self.set_mode(AD7124_OpMode_SingleConv)
                
                #print(f"<read_raw> set_mode returned {ret}")
                
                if (ret < 0):                
                    return ret                
                        

        ret = self.wait_for_conv_ready(_DEFAULT_TIMEOUT_MS)
        
        if (ret < 0):        
            return ret        
    
        return self.get_data()        

    
    
    def read_volts(self, ch: int) -> float:
        """
        Read voltage from a channel.
        
        Converts raw ADC counts to voltage based on the channel's setup
        configuration (reference voltage and gain). For proper results, ensure
        the external reference voltage is correctly set via set_config().
        
        :param int ch: Channel number to read (0-15)
        
        :return: Voltage in volts (float)
        """
        return self.to_volts(self.read_raw(ch), ch)
        
    # *** Not working yet ***
    # def read_tc(self, ch: int, ref_temp: float, type: int):
    #     '''
    #     Read K Type thermocouple. The channel must first be setup properly
    #     for reading thermocouples.
    #     '''
    #     return self.thermocouple.volts_to_tempC(self.read_volts(ch), ref_temp, type)
        

    def read_fb(self, ch: int, vEx: float, scale_factor: float = 1.00) -> float:
        """
        Read a 4-wire full-bridge sensor (load cell, pressure, etc.).
        
        Calculates the bridge output as millivolts per volt of excitation.
        Optionally applies a linear scaling factor.
        
        :param int ch: Channel number (must be configured for full-bridge sensor)
        :param float vEx: Excitation voltage in volts
        :param float scale_factor: Linear scaling factor (default: 1.0 = mV/V)
        
        :return: Scaled sensor reading
        """
        return ((self.read_volts(ch) * 1000.0) / vEx) * scale_factor
    

    def set_adc_control(self, mode: int, power_mode: int, ref_en: bool, 
                        clk_sel: int = AD7124_Clk_Internal) -> int:
        """
        Configure global ADC settings.
        
        Sets the operating mode, power mode, reference enable, and clock source
        for the ADC. This method always enables Data+Status mode.
        
        :param int mode: Operating mode (AD7124_OpMode_* constants)
        :param int power_mode: Power mode (AD7124_LowPower, AD7124_MidPower, AD7124_FullPower)
        :param bool ref_en: Enable internal reference voltage
        :param int clk_sel: Clock source (default: AD7124_Clk_Internal)
        
        :return: 0 on success, error code on failure
        """
        
        #NOTE: We always uses Data + Status mode
        self.regs[_AD7124_ADC_CTRL_REG].value = _AD7124_ADC_CTRL_REG_MODE(mode) | \
                                                _AD7124_ADC_CTRL_REG_POWER_MODE(power_mode) | \
                                                _AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel) | \
                                                (_AD7124_ADC_CTRL_REG_REF_EN if ref_en else 0) | \
                                                _AD7124_ADC_CTRL_REG_DATA_STATUS | \
                                                _AD7124_ADC_CTRL_REG_CS_EN
        
        return self.write_register(self.regs[_AD7124_ADC_CTRL_REG])
    
    def read_ic_temp(self, ch: int) -> float:
        """
        Read the on-chip temperature sensor.
        
        The channel must be configured to read the internal temperature sensor
        (with AD7124_Input_TEMP as the positive input).
        
        :param int ch: Channel configured for internal temperature sensor
        
        :return: Temperature in degrees Celsius
        """
        return self.scale_ic_temp(self.read_raw(ch))


    def set_mode(self, mode: int) -> int:
        """
        Set the ADC operating mode.
        
        :param int mode: Operating mode (AD7124_OpMode_*)
        
        :return: 0 on success, error code on failure
        """

        self.opmode = mode
        
        #print(f"<set_mode> starting val {hex(self.regs[_AD7124_ADC_CTRL_REG].value)} for ADC control register")

        self.regs[_AD7124_ADC_CTRL_REG].value &= ~_AD7124_ADC_CTRL_REG_MODE(0x3C) #clear mode (was 0x0F, but I don't think thats correct
        self.regs[_AD7124_ADC_CTRL_REG].value |= _AD7124_ADC_CTRL_REG_MODE(mode)
        
        #print(f"<set_mode> Writing {hex(self.regs[_AD7124_ADC_CTRL_REG].value)} to ADC control register")
        return self.write_register(self.regs[_AD7124_ADC_CTRL_REG])


    #def mode(self): # No need for this in python, just read self.opmode

    
    def set_channel(self, ch: int, setup: int, aiPos: int,
                    aiNeg: int, enable: bool) -> int:
        """
        Configure a channel's input pins and associated setup.
        
        :param int ch: Channel number (0-15)
        :param int setup: Setup configuration to use (0-7)
        :param int aiPos: Positive input (AD7124_Input_* or AIN0-AIN15)
        :param int aiNeg: Negative input (AD7124_Input_* or AIN0-AIN15)
        :param bool enable: True to enable channel, False to disable
        
        :return: 0 on success, error code on failure
        """

        if ((ch < 16) and (setup < 8)):
        
            # Offset to channel regs
            ch += _AD7124_CH0_MAP_REG

            self.regs[ch].value = _AD7124_CH_MAP_REG_SETUP(setup) | \
                                  _AD7124_CH_MAP_REG_AINP(aiPos) | \
                                  _AD7124_CH_MAP_REG_AINM(aiNeg) | \
                                  (_AD7124_CH_MAP_REG_CH_ENABLE if enable else 0)

            return self.write_register(self.regs[ch])
        
        return -1


    def enable_channel(self, ch: int, enable: bool) -> int:
        """
        Enable or disable a channel.
        
        :param int ch: Channel number (0-15)
        :param bool enable: True to enable, False to disable
        
        :return: 0 on success, error code on failure
        """

        if (ch < 16): 
        
            # Offset to channel regs
            ch += _AD7124_CH0_MAP_REG

            ret = self.read_register(self.regs[ch]) # Is this read necessary?
            if (ret < 0):            
                return ret
            

            if (enable):
            
                self.regs[ch].value |= _AD7124_CH_MAP_REG_CH_ENABLE
            
            else:            
                self.regs[ch].value &= ~_AD7124_CH_MAP_REG_CH_ENABLE
            

            return self.write_register(self.regs[ch])
        
        return -1


    def enabled(self, ch: int) -> bool:
        """
        Check if a channel is enabled.
        
        :param int ch: Channel number (0-15)
        
        :return: True if enabled, False if disabled
        """
        ch += _AD7124_CH0_MAP_REG  
        return bool((self.regs[ch].value & _AD7124_CH_MAP_REG_CH_ENABLE) >> 15)     

    # Unused?
    # def status(self):
    #     '''Reads the status register'''
    #     return self.read_register(self.regs[_AD7124_STATUS_REG])


    
    def channel_setup(self, ch: int) -> int:
        """
        Get the setup number assigned to a channel.
        
        :param int ch: Channel number (0-15)
        
        :return: Setup number (0-7), or -1 if invalid
        """

        if (ch < _AD7124_MAX_CHANNELS):    
            
            ch += _AD7124_CH0_MAP_REG 

            setup = (self.regs[ch].value >> 12) & 0x07
            return setup
        
        return -1
    

    def current_channel(self) -> int:
        """
        Get the currently active ADC channel.
        
        Only accurate when Data+Status mode is enabled.
        
        :return: Current channel number (0-15)
        """   
        return self.regs[_AD7124_STATUS_REG].value & 0x0F
    

    def get_data(self) -> uint :
        '''
        Returns positive raw ADC counts, or negative error code
        This version assumes the data + status mode is enabled. As long as testing
        checks out, that will be how the library will operate from now on -JJ 5-18-2021
        '''    

        # Temporary reg struct for data with extra byte to hold status bits
        #Reg_DataAndStatus = Ad7124_Register(0x02, 0x0000, 4, 2)

        ret = self.no_check_read_register(self.reg_data_and_status)
        
        #print(f"<get_data> register read = {hex(Reg_DataAndStatus.value)}")
        
        if (ret < 0):        
            return ret
        

        self.regs[_AD7124_STATUS_REG].value = self.reg_data_and_status.value & 0xFF
        self.regs[_AD7124_DATA_REG].value = (self.reg_data_and_status.value >> 8) & 0x00FFFFFF;
    
        #print(f"get_data return value = {hex(self.regs[_AD7124_DATA_REG].value)}")
        return self.regs[_AD7124_DATA_REG].value
        

    
    def to_volts(self, value: int, ch: int) -> float:
        """
        Convert raw ADC counts to voltage.
        
        Applies the setup's reference voltage and gain settings to convert
        raw ADC data to volts. Handles both unipolar and bipolar modes.
        
        :param int value: Raw ADC count value
        :param int ch: Channel number (for setup lookup)
        
        :return: Voltage value in volts
        """
    
        #voltage = value
        idx = self.channel_setup(ch)
        chReg = ch + _AD7124_CH0_MAP_REG
        
         
        #Special case, if reading internal temp sensor just 
        #return the original value unchanged so that the output
        #can be easily be converted with formula from datasheet  
        ainP = (self.regs[chReg].value >> 5) & 0x1F
        ainN =  self.regs[chReg].value & 0x1F
        if((ainP == AD7124_Input_TEMP) or (ainN == AD7124_Input_TEMP)):
            return value;    
        
        #print(f"<to_volts> raw counts = {value}")
        
        if (self.setup[idx].setup_values.bipolar):        
            voltage = value / 0x7FFFFF - 1
            #print(f"<to_volts> bipolar, raw voltage = {voltage}")
        else:        
            voltage = value / 0xFFFFFF
            #print(f"<to_volts> unipolar, raw voltage = {voltage}")
        
        # .setup_values.gain holds 0 to 7 value that sets the actual gain in the register, we
        # need to left shift 1 by the gain register value to get the actual gain for use in
        # the calculation below
        voltage = (voltage * self.setup[idx].setup_values.refV) / (1 << self.setup[idx].setup_values.gain)
        return voltage
        

    def scale_tc(self, volts, refTemp, type):        
        return self.thermocouple.volts_to_tempC(volts, refTemp, type)
        

    def scale_fb(self, volts, vEx, scaleFactor):        
        return ((volts * 1000.0) / vEx) * scaleFactor
        

    
    def scale_ic_temp(self, value: int) -> float:
        """
        Convert raw IC temperature sensor value to Celsius.
        
        Uses the conversion formula from the AD7124 datasheet:
        Temp(°C) = ((RAW - 0x800000) / 13548) - 272.5
        
        :param int value: Raw temperature sensor reading
        
        :return: Temperature in degrees Celsius
        """        
        return ((value - 0x800000) / 13548.00) - 272.5
        

    
    def wait_for_conv_ready(self, timeout: int) -> int:
        """
        Wait until the ADC conversion result is ready.
        
        Polls the status register's RDY bit. Blocks until data is ready or timeout.
        
        :param int timeout: Timeout in milliseconds
        
        :return: True if ready, or negative error code on timeout
        """    
    
        start_time = adafruit_ticks.ticks_ms()

        while (1): 
            
            ret = self.no_check_read_register(self.regs[_AD7124_STATUS_REG]) 
            if (ret < 0):                
                return ret; # Problem, bail and forward the error              

            # Check the RDY bit in the Status Register 
            ready = (self.regs[_AD7124_STATUS_REG].value & _AD7124_STATUS_REG_RDY) == 0
            #print(f"<wait_for_conv_ready> reg value = {hex(self.regs[_AD7124_STATUS_REG].value)}, ready = {ready}")
            if (ready):                
                return ready
                

            if adafruit_ticks.ticks_diff(adafruit_ticks.ticks_ms(), start_time) >= timeout:            
                return _AD7124_TIMEOUT #Time out
                
    

    ##########################################################

    def no_check_read_register(self, reg: Ad7124_Register) -> int:
        """
        Read a register without checking if device is ready.
        
        Updates the register value in-place. Internal method.
        
        :param Ad7124_Register reg: Register to read
        
        :return: 0 on success, error code on failure
        """
        #print("no_check_read_register:")
        
        if (reg is None) or (reg.rw == _AD7124_W):
            return _AD7124_INVALID_VAL
        
        #TODO: This needs to be adapted to use a pre-allocated buffer
        #buffer = bytearray(reg.size + 1) #dynamic allocation = BAD in MP!
        buffer = self.spi_buffs[reg.size -1] #get the appropriate memmoryview from tuple so no allocation
        
        
        buffer[0] = (_AD7124_COMM_REG_WEN | _AD7124_COMM_REG_RD |
                   _AD7124_COMM_REG_RA(reg.addr))       

        
        #print(f"writing {buffer.hex()}")
        
        #TODO: Handle if crc is enabled (additional byte required)
        self.spi_write_and_read(buffer) #The size parameter is not currently used
        
        #self.spi.write_readinto(buffer, buffer)
        
        #print(f"read buffer is: {buffer.hex()}")
        
        reg.value = 0
        for i in range(1,reg.size+1):
            reg.value <<= 8
            reg.value += buffer[i]
 
            
        # print(f"  reg.valu is: {hex(reg.value)}")
        # print("")

        return 0 # Maybe this should be done differently? Exception?
    
    def read_register(self, reg: Ad7124_Register) -> int:
        """
        Read a register with ready-check.
        
        Waits for SPI ready before reading. Updates register value in-place.
        
        :param Ad7124_Register reg: Register to read
        
        :return: 0 on success, error code on failure
        """
        #print("read_register")
        
        # direct translation from C++ library, probably not the best way to do
        # this in MicroPython
        if reg.addr != _AD7124_ERR_REG:
            ret = self.wait_for_spi_ready()
            if ret < 0:
                return ret
            
        return self.no_check_read_register(reg)
    
    
    
    def no_check_write_register(self, reg: Ad7124_Register) -> int:
        """
        Write a register without checking if device is ready.
        
        Internal method. Use write_register() for safer operation.
        
        :param Ad7124_Register reg: Register to write
        
        :return: 0 on success, error code on failure
        """
        #print("no_check_write_register:")
        
        if (reg is None) or (reg.rw == _AD7124_R):
            return _AD7124_INVALID_VAL
        
        #TODO: This needs to be adapted to use a pre-allocated buffer        
        #buffer = bytearray(reg.size +1) # allocation = BAD in MP
        buffer = self.spi_buffs[reg.size - 1] # Get correct mem view form tupple
        
        
        buffer[0] = (_AD7124_COMM_REG_WEN | _AD7124_COMM_REG_WR |
                   _AD7124_COMM_REG_RA(reg.addr))
        
        #print(f"  reg.value {hex(reg.value)}")
        
        value = reg.value # may need to use copy here? No, works (for now). Fuck, python is annoying
        
        # Fill the write buffer
        for i in range(reg.size):
            buffer[reg.size - i] = value & 0xFF
            value >>= 8

        #print(f"  reg.value {hex(reg.value)}")
        
        #TODO: Handle if crc is enabled (additional byte required)
                 
        
        #print(f"  write buffer {buffer.hex()}")
        
        self.spi_write_and_read(buffer)      
                
        return 0        
        

    def write_register(self, reg: Ad7124_Register) -> int:
        """
        Write a register with ready-check.
        
        Waits for SPI ready before writing.
        
        :param Ad7124_Register reg: Register to write
        
        :return: 0 on success, error code on failure
        """
        #print("write_register")
        
        ret = self.wait_for_spi_ready()
        if ret < 0:
            return ret
            
        return self.no_check_write_register(reg) 


    def spi_write_and_read(self, buff: bytearray):
        '''
        Writes and reads data via SPI using SPIDevice for proper bus locking
        and chip select management.
        '''
        with self.spi_device as spi:
            spi.write_readinto(buff, buff)
    
    def wait_for_spi_ready(self) -> bool:
        """
        Wait until the SPI interface is ready.
        
        Monitors the SPI error register for ignore errors.
        
        :return: True when ready
        """
        
        #print("wait_for_spi_ready")
        
        # TODO: Figure out best way to handle timeout with MicroPython

        #reg = Ad7124_Register()
        reg = self.regs[_AD7124_ERR_REG]       
                  
        while True:
            ret = self.no_check_read_register(reg)

            #TODO: Check return value for error (< 0)

            ready = not (self.regs[_AD7124_ERR_REG].value & _AD7124_ERR_REG_SPI_IGNORE_ERR)
            
            #print(f"waiting... ready = {ready}")
            
            if ready:
                return ready            
            

    def computeCRC8(self, buffer, size):
        '''
        Computes the CRC checksum for a data buffer.
        NOT USED YET
        '''
        crc = 0
        
        while size:
            for i in range(80, 0, -1):
                if ((crc & 0x80) != 0) != ((buffer[0] & i) != 0):
                    crc <<= 1
                    crc ^= _AD7124_CRC8_POLYNOMIAL_REPRESENTATION
                else:
                    crc <<= 1
            buffer = buffer[1:]
            size -= 1

        return crc