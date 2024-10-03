# CircuitPy_AD7124
Circuit Python library for using the Analog Devices AD7124-4, 24 bit ADC

The AD7124-4 is a 4 channel, 24 bit, differential ADC (it can also be configured for up to 7 single ended channels). This library is a port of the [NHB_AD7124 Arduino library](https://github.com/NHBSystems/NHB_AD7124) to Circuit Python. The library is currently written in all Python and is a bit of a memory hog as it is, but it gets the job done. I am new to Circuit Python, so I'm sure there is plenty of room for improvement and optimization.  

Like the original Arduino library, this was originally written for use 
with the [NHB AD7124 Analog Sensor FeatherWing](https://www.tindie.com/products/24680/), but there is no reason it couldn't be used with a raw chip in your own design.

So far I have only tested this library on an ESP32-C3, but it 
really should work with any architectures that has a working SPI
implementation.   

WARNING, THIS IS A WORK IN PROGRESS


Basic API  
--------


### Initialization  

The init method takes a Chip Select pin and a previously configured SPI object  
```python
def __init__(self, csPin, spi, baud = 4000000):
```
Where
| Arg           |  Description    |
| -----------   | --------------- |
|*cs*           | Chip Select pin |
|*spi*          | Previously configured SPI object|
|*baud*         | SPI clock frequency *(default = 4000000)*


So, to create an instance...
```python
from CircuitPy_AD7124 import NHB_AD7124

adc = NHB_AD7124.Ad7124(cs,spi)
```


-------------------------

### ADC Configuration  

To set the basic global parameters of the ADC we use the `setAdcControl(..)` method

```python
def set_adc_control(self, mode: int, power_mode: int, ref_en: bool, 
                        clk_sel: int = AD7124_Clk_Internal):        
```
Where  
| Arg       | Description  | Valid Options |
|-----------|--------------|---------------|
| *mode*       | The operating mode to set device to          | `AD7124_OpMode_Continuous` <br> `AD7124_OpMode_SingleConv` <br> `AD7124_OpMode_Standby` <br> `AD7124_OpMode_PowerDown` <br> `AD7124_OpMode_Idle` <br> `AD7124_OpMode_InternalOffsetCalibration` <br> `AD7124_OpMode_InternalGainCalibration` <br> `AD7124_OpMode_SystemOffsetCalibration` <br> `AD7124_OpMode_SystemGainCalibration`|
| *power_mode* | Power mode (Low, Mid, Full)                  | `AD7124_LowPower` <br> `AD7124_MidPower` <br> `AD7124_FullPower`|
| *ref_en*     | Enable the internal reference voltage        | `True` <br> `False`|
| *clk_sel*    | Set the clock source  *(default = internal)* | `AD7124_Clk_Internal` <br> `AD7124_Clk_InternalWithOutput` <br> `AD7124_Clk_External` <br> `AD7124_Clk_ExternalDiv4`|

---------------------------
<br>

### Setup Configuration

The AD7124-4 ICs have a feature defined in the datasheet as "Setups". The Setups
allow for pre-configuring for different sensor types, independent of which 
channel they are assigned to. The setups are modeled in the library as an array 
of setup objects contained within the Ad7124 class. There are 8 individual 
setups that can be used to hold different configurations. A single setup can be
 used for multiple channels of the same type. 

The two main methods for configuring a setup are `set_config(..)` and `set_filter(..)`
  
#### set_config(..)
```python
def set_config(self, ref_source, gain, bipolar: bool, 
                   burnout = AD7124_Burnout_Off, 
                   exRefV: float = 2.50):
```
Where... 
| Arg          | Description  | Valid Options |
|--------------|--------------|---------------|
| *ref_source* | Set the reference voltage source. |`AD7124_Ref_ExtRef1` <br> `AD7124_Ref_ExtRef2` <br> `AD7124_Ref_Internal` <br> `AD7124_Ref_Avdd`|
| *gain*       | Set the PGA gain                  |`AD7124_Gain_1` <br> `AD7124_Gain_2` <br> `AD7124_Gain_4` <br> `AD7124_Gain_8` <br> `AD7124_Gain_16` <br> `AD7124_Gain_32` <br> `AD7124_Gain_64` <br> `AD7124_Gain_128`|
| *bipolar*    | Bipolar or unipolar measurement   |`True` <br> `False`|
| *burnout*    | Burnout current options. *(Optional argument, default is off. **This option is untested**)*|`AD7124_Burnout_Off` <br> `AD7124_Burnout_500nA` <br> `AD7124_Burnout_2uA` <br> `AD7124_Burnout_4uA`|
| *exRefV*     | External reference voltage (if used). This value is used in calculations, so it's important that it is set correctly | `Float value between 0 and AVDD`|

<br> 

#### set_filter(..)

```python
def set_filter(self, filter, fs, 
                   post_filter = AD7124_PostFilter_NoPost,
                   rej60: bool = False, single_cycle: bool = False):
```  
Where...
| Arg          | Description  | Valid Options |
|--------------|--------------|---------------|
| `filter`       | Select which filter type to use |`AD7124_Filter_SINC4`<br>`AD7124_Filter_SINC3`<br>`AD7124_Filter_FAST4`<br>`AD7124_Filter_FAST3`<br>`AD7124_Filter_POST`|
| `fs`           | Filter output rate select bits. can be a value from 1 to 2047. Setting to 1 will give fastest output for the selected filter. See datasheet for details |
| `post_filter`  | Selects a post filter option <br> *Optional argument, defaults to AD7124_PostFilter_NoPost* |
| `rej60`        | Enables a first order notch at 60 Hz when the first notch of the sinc filter is at 50 Hz, allowing simultaneous 50 and 60 Hz rejection. <br>*Optional argument, defaults to false* |
| `single_cycle` | Enables "single cycle conversion". Has no effect when using multiple channels or when in single conversion mode. <br> *Optional argument, defaults to false* |

<br>  

### Channel Configuration

We need need to define what inputs are used by each channel and what setup will 
be used. This is done with the `set_channel(..)` method.

```python
def set_channel(self, ch: int, setup: int, aiPos: int,
                   aiNeg: int, enable: bool):  
```
Where...
| Arg          | Description  |
|--------------|--------------|
| `ch`           | The channel we are configuring      | 
| `setup`        | Which setup will this channel use?  | 
| `aiPos`        | The pin that will be connected to the positive analog input for this channel |
| `aiNeg`        | The pin that will be connected to the negative analog input for this channel |
| `enable`       | Set if the channel is enabled or not <br> *Optional argument, defaults to false* |

<br>

### Getting Readings

There are a few different methods to get readings from the ADC. The most straightforward 
one is `read_volts(ch)`, which returns the ADC reading in Volts. *Note: If using 
an external reference voltage, the voltage must be properly set with the `set_config(..)`
method for this function to work correctly.*

```python
def read_volts(self, ch: int):
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel to read.                                      |  

<br>
  
The `read_raw(ch)` method is provided for lower level access to the raw ADC counts
for the given channel  

```python
def read_raw(self, ch: int) -> uint:
```  
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel to read.                                      |  

<br>

There are also a few sensor specific read functions provided for convenience.  

The `read_FB(ch, vEx, scale_factor)` method is for reading full bridge type 
sensors (load cells, pressure gauges, extensometers, ...). It could also be 
used for potentiometers as long as you have setup the channel properly. Returns 
mV/V if scale factor is one (default). The channel read must already be properly 
configured for reading the sensor.

```python
def read_fb(self, ch: int, vEx: float, scale_factor: float = 1.00):
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel to read                                       |
| `vEx`          | The excitation voltage being used. (2.50 for built in regulator on NHB boards) |
| `scale_factor`  | A linear scaling factor to apply to the reading. A factor of 1.00 will return the reading in mV/V. <br> *Optional argument, defaults to 1.00* |

<br>

The `read_ic_temp(ch)` method reads the temperature sensor embedded in the AD7124. 
The channel must be properly configured to read the sensor first.

```python
def read_ic_temp(self, ch: int):
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel that is configured to read the internal temperature sensor |

<br>

### Excitation Voltage 

Some sensors like wheatstone bridges, potentiometers, and thermistors, require 
an excitation voltage to read. The NHB Systems AD7124 boards include a 2.5V 
linear regulator to provide this excitation. The enable pin of the regulator 
is tied to the PSW pin of the AD7124-4 allowing it to be controlled by software. 
This allows the regulator to be powered down to save power between readings in 
long term, low speed logging applications. To enable the regulator we just need 
to call the setPWRSW() method. If you are using the AD7124-4 in your own design,
this method controls PSW pin (on-chip low side power switch), which can be used 
to control power to external sensors, but is limited to 30 mA.

The `setPWRSW(bool)` method controls the internal low side switch connected to 
the PSW pin on the AD7124-4

```python
def setPWRSW(self, enabled):
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `enabled`      | Sets if switch is enabled (closed) or not (open). *On NHB Systems AD7124 boards, this is tied to a 2.5V linear regulator to provide excitation voltage*

<br/><br/>  

Basic Example
-----------  
```python
import time 
import board
import busio
import neopixel
import gc

from CircuitPy_AD7124 import NHB_AD7124

filterSelectBits = 4 #Fast

csPin = board.A0


spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

adc = NHB_AD7124.Ad7124(csPin,spi)


print("Trying to read ID register...")
ID = adc.get_ID()
print(f"ADC chip ID: {hex(ID)}")


# Configure the ADC in full power mode
adc.set_adc_control(NHB_AD7124.AD7124_OpMode_SingleConv,
                    NHB_AD7124.AD7124_FullPower,True)

# Configure Setup 0 for load cells:
# - Use the external reference tied to the excitation voltage (2.5V reg)
# - Set a gain of 128 for a bipolar measurement of +/- 19.53 mv
adc.setup[0].set_config(NHB_AD7124.AD7124_Ref_ExtRef1, NHB_AD7124.AD7124_Gain_128, True)

# Configure Setup 1 for using the internal temperature sensor
# - Use internal reference and a gain of 1
adc.setup[1].set_config(NHB_AD7124.AD7124_Ref_Internal, NHB_AD7124.AD7124_Gain_1, True) #IC Temp



# Set filter type and data rate select bits (defined above)
#  The combination of filter type and filter select bits affects the final
#  output data rate. The SINC3 filter seems to be the best general purpose
#  option. The filter select bits are a number between 1 and 2048. A smaller
#  number will give a faster conversion, but allow more noise. Refer to the
#  datasheet for details, it can get complicated.
adc.setup[0].set_filter(NHB_AD7124.AD7124_Filter_SINC3, filterSelectBits)
adc.setup[1].set_filter(NHB_AD7124.AD7124_Filter_SINC3, filterSelectBits)



# Set channel 0 to use pins AIN0(+)/AIN1(-)
adc.set_channel(0, 0, NHB_AD7124.AD7124_Input_AIN0, NHB_AD7124.AD7124_Input_AIN1, True)

# Set channel 1 to use the internal temperature sensor for the positive input
# and AVSS for the negative
adc.set_channel(1, 1, NHB_AD7124.AD7124_Input_TEMP, NHB_AD7124.AD7124_Input_AVSS, True) #IC Temp


print("Turning ExV on")
adc.setPWRSW(True)
time.sleep(0.2) # No sleep_ms() in CircuitPy


print("Now try to get some readings...")


while True:
    
    lc_reading = adc.read_fb(0, 2.5, 5.00)  # Read full brideg sensor (Chan, Ex V, Scaling)
    #reading = adc.read_volts(0)         # Or use this to just read the voltage
    print(f"Loadcell = {lc_reading:.4f}", end=',')
    
    ic_temp = adc.read_ic_temp(1)       
    print(f" IC Temp = {ic_temp:.2f}")
        
    time.sleep(0.1) # No sleep_ms() in CircuitPy 

```