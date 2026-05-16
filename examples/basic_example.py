
import time 
import board
import busio
import digitalio
import nhb_ad7124

# Filter select bits (4 = Fast)
FILTER_SELECT_BITS = 4

# Create the chip select pin
cs_pin = digitalio.DigitalInOut(board.A0)

# Create and configure the SPI bus
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialize the ADC with SPI bus and chip select pin
adc = nhb_ad7124.Ad7124(spi, cs_pin, baudrate=4000000)

print("Trying to read ID register...")
ID = adc.get_ID()
print(f"ADC chip ID: {hex(ID)}")

# Configure the ADC in full power mode
adc.set_adc_control(nhb_ad7124.AD7124_OpMode_SingleConv,
                    nhb_ad7124.AD7124_FullPower, True)

# Configure Setup 0 for load cells:
# - Use the external reference tied to the excitation voltage (2.5V reg)
# - Set a gain of 128 for a bipolar measurement of +/- 19.53 mv
adc.setup[0].set_config(nhb_ad7124.AD7124_Ref_ExtRef1, nhb_ad7124.AD7124_Gain_128, True)

# Configure Setup 1 for using the internal temperature sensor
# - Use internal reference and a gain of 1
adc.setup[1].set_config(nhb_ad7124.AD7124_Ref_Internal, nhb_ad7124.AD7124_Gain_1, True)

# Set filter type and data rate select bits (defined above)
#  The combination of filter type and filter select bits affects the final
#  output data rate. The SINC3 filter seems to be the best general purpose
#  option. The filter select bits are a number between 1 and 2048. A smaller
#  number will give a faster conversion, but allow more noise. Refer to the
#  datasheet for details, it can get complicated.
adc.setup[0].set_filter(nhb_ad7124.AD7124_Filter_SINC3, FILTER_SELECT_BITS)
adc.setup[1].set_filter(nhb_ad7124.AD7124_Filter_SINC3, FILTER_SELECT_BITS)

# Set channel 0 to use pins AIN0(+)/AIN1(-)
adc.set_channel(0, 0, nhb_ad7124.AD7124_Input_AIN0, nhb_ad7124.AD7124_Input_AIN1, True)

# Set channel 1 to use the internal temperature sensor for the positive input
# and AVSS for the negative
adc.set_channel(1, 1, nhb_ad7124.AD7124_Input_TEMP, nhb_ad7124.AD7124_Input_AVSS, True)

print("Turning ExV on")
adc.setPWRSW(True)
time.sleep(0.2)

print("Now try to get some readings...")

while True:
    lc_reading = adc.read_fb(0, 2.5, 5.00)  # Read full bridge sensor (Chan, Ex V, Scaling)
    print(f"Loadcell = {lc_reading:.4f}", end=', ')
    
    ic_temp = adc.read_ic_temp(1)       
    print(f"IC Temp = {ic_temp:.2f}")
        
    time.sleep(0.1)
