import requests
import json
from Adafruit_IO import Client
import RPi.GPIO as GPIO
import time,sys
from gpiozero import LED
import smbus
from mfrc522 import SimpleMFRC522
from datetime import date

#FUNCTIONS
def spraylemons(amount):
    openvalve.on()
    amountwatered = 0
    while amountwatered<amount:
        start_counter = 1
        time.sleep(1)
        start_counter = 0
        flow = (count / 7.5)/60 # persec Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
        amountwatered +=flow
        print("The flow is: %.3f Liter/min" % (flow))
        #publish.single("/Garden.Pi/WaterFlow", flow, hostname=MQTT_SERVER)
        count = 0
    openvalve.off()


def getwattsa(power1,power2,power3,power4):
    bus_voltage1 = ina1.getBusVoltage_V()             # voltage on V- (load side)
    shunt_voltage1 = ina1.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
    current1 = ina1.getCurrent_mA()                   # current in mA
    power1 += ina1.getPower_W()                        # power in watts

    bus_voltage2 = ina2.getBusVoltage_V()             # voltage on V- (load side)
    shunt_voltage2 = ina2.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
    current2 = ina2.getCurrent_mA()                   # current in mA
    power2 += ina2.getPower_W()                        # power in watts

    bus_voltage3 = ina3.getBusVoltage_V()             # voltage on V- (load side)
    shunt_voltage3 = ina3.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
    current3 = ina3.getCurrent_mA()                   # current in mA
    power3 += ina3.getPower_W()                        # power in watts


    bus_voltage4 = ina4.getBusVoltage_V()             # voltage on V- (load side)
    shunt_voltage4 = ina4.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
    current4 = ina4.getCurrent_mA()                   # current in mA
    power4 += ina4.getPower_W()                        # power in watts

    current_time = time.localtime()
    is_midnight = current_time.tm_hour ==0 and current_time.tm_min ==0 and current_time.tm_sec ==0
    if is_midnight:
        kwh1=power1/DAYSECONDS
        kwh2=power2/DAYSECONDS
        kwh3=power3/DAYSECONDS
        kwh4=power4/DAYSECONDS
        kwhdata=str(kwh1)+","+str(kwh2)+","+str(kwh3)+","+str(kwh4)
        aio.send("energyuse",kwhdata)
    return power1,power1,power3,power4
#END OF FUNCTIONS

DAYSECONDS = 86400
LITREAMOUNT = 25*5

GPIO.setmode(GPIO.BCM)
#RFID DATA
reader = SimpleMFRC522()
#GPIO.setmode(GPIO.BCM)
#set ultrasonic pins
GPIO_TRIGGER = 23
GPIO_ECHO = 4
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

start_date = date.today()

light1status = False
light2status = False
fanopen = False 
someoneinside = False
names = ["vasia"]
#END OF RFID DATA


#START OF INIT OF THE HAT
DAYSECONDS = 86400
# Config Register (R/W)
_REG_CONFIG                 = 0x00
# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE           = 0x01

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE             = 0x02

# POWER REGISTER (R)
_REG_POWER                  = 0x03

# CURRENT REGISTER (R)
_REG_CURRENT                = 0x04

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION            = 0x05

class BusVoltageRange:
    """Constants for ``bus_voltage_range``"""
    RANGE_16V               = 0x00      # set bus voltage range to 16V
    RANGE_32V               = 0x01      # set bus voltage range to 32V (default)

class Gain:
    """Constants for ``gain``"""
    DIV_1_40MV              = 0x00      # shunt prog. gain set to  1, 40 mV range
    DIV_2_80MV              = 0x01      # shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV             = 0x02      # shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV             = 0x03      # shunt prog. gain set to /8, 320 mV range

class ADCResolution:
    """Constants for ``bus_adc_resolution`` or ``shunt_adc_resolution``"""
    ADCRES_9BIT_1S          = 0x00      #  9bit,   1 sample,     84us
    ADCRES_10BIT_1S         = 0x01      # 10bit,   1 sample,    148us
    ADCRES_11BIT_1S         = 0x02      # 11 bit,  1 sample,    276us
    ADCRES_12BIT_1S         = 0x03      # 12 bit,  1 sample,    532us
    ADCRES_12BIT_2S         = 0x09      # 12 bit,  2 samples,  1.06ms
    ADCRES_12BIT_4S         = 0x0A      # 12 bit,  4 samples,  2.13ms
    ADCRES_12BIT_8S         = 0x0B      # 12bit,   8 samples,  4.26ms
    ADCRES_12BIT_16S        = 0x0C      # 12bit,  16 samples,  8.51ms
    ADCRES_12BIT_32S        = 0x0D      # 12bit,  32 samples, 17.02ms
    ADCRES_12BIT_64S        = 0x0E      # 12bit,  64 samples, 34.05ms
    ADCRES_12BIT_128S       = 0x0F      # 12bit, 128 samples, 68.10ms

class Mode:
    """Constants for ``mode``"""
    POWERDOW                = 0x00      # power down
    SVOLT_TRIGGERED         = 0x01      # shunt voltage triggered
    BVOLT_TRIGGERED         = 0x02      # bus voltage triggered
    SANDBVOLT_TRIGGERED     = 0x03      # shunt and bus voltage triggered
    ADCOFF                  = 0x04      # ADC off
    SVOLT_CONTINUOUS        = 0x05      # shunt voltage continuous
    BVOLT_CONTINUOUS        = 0x06      # bus voltage continuous
    SANDBVOLT_CONTINUOUS    = 0x07      # shunt and bus voltage continuous


class INA219:
    def __init__(self, i2c_bus=1, addr=0x40):
        self.bus = smbus.SMBus(i2c_bus);
        self.addr = addr

        # Set chip to known config values to start
        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0
        self.set_calibration_32V_2A()

    def read(self,address):
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return ((data[0] * 256 ) + data[1])

    def write(self,address,data):
        temp = [0,0]
        temp[1] = data & 0xFF
        temp[0] =(data & 0xFF00) >> 8
        self.bus.write_i2c_block_data(self.addr,address,temp)

    def set_calibration_32V_2A(self):
        """Configures to INA219 to be able to measure up to 32V and 2A of current. Counter
           overflow occurs at 3.2A.
           ..note :: These calculations assume a 0.1 shunt ohm resistor is present
        """
        # By default we use a pretty huge range for the input voltage,
        # which probably isn't the most appropriate choice for system
        # that don't use a lot of power.  But all of the calculations
        # are shown below if you want to change the settings.  You will
        # also need to change any relevant register settings, such as
        # setting the VBUS_MAX to 16V instead of 32V, etc.

        # VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
        # VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
        # RSHUNT = 0.1               (Resistor value in ohms)

        # 1. Determine max possible current
        # MaxPossible_I = VSHUNT_MAX / RSHUNT
        # MaxPossible_I = 3.2A

        # 2. Determine max expected current
        # MaxExpected_I = 2.0A

        # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        # MinimumLSB = MaxExpected_I/32767
        # MinimumLSB = 0.000061              (61uA per bit)
        # MaximumLSB = MaxExpected_I/4096
        # MaximumLSB = 0,000488              (488uA per bit)

        # 4. Choose an LSB between the min and max values
        #    (Preferrably a roundish number close to MinLSB)
        # CurrentLSB = 0.0001 (100uA per bit)
        self._current_lsb = .1  # Current LSB = 100uA per bit

        # 5. Compute the calibration register
        # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        # Cal = 4096 (0x1000)

        self._cal_value = 4096

        # 6. Calculate the power LSB
        # PowerLSB = 20 * CurrentLSB
        # PowerLSB = 0.002 (2mW per bit)
        self._power_lsb = .002  # Power LSB = 2mW per bit

        # 7. Compute the maximum current and shunt voltage values before overflow
        #
        # Max_Current = Current_LSB * 32767
        # Max_Current = 3.2767A before overflow
        #
        # If Max_Current > Max_Possible_I then
        #    Max_Current_Before_Overflow = MaxPossible_I
        # Else
        #    Max_Current_Before_Overflow = Max_Current
        # End If
        #
        # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        # Max_ShuntVoltage = 0.32V
        #
        # If Max_ShuntVoltage >= VSHUNT_MAX
        #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        # Else
        #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        # End If

        # 8. Compute the Maximum Power
        # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        # MaximumPower = 3.2 * 32V
        # MaximumPower = 102.4W

        # Set Calibration register to 'Cal' calculated above
        self.write(_REG_CALIBRATION,self._cal_value)

        # Set Config register to take into account the settings above
        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        self.config = self.bus_voltage_range << 13 | \
                      self.gain << 11 | \
                      self.bus_adc_resolution << 7 | \
                      self.shunt_adc_resolution << 3 | \
                      self.mode
        self.write(_REG_CONFIG,self.config)
        
    def getShuntVoltage_mV(self):
        self.write(_REG_CALIBRATION,self._cal_value)
        value = self.read(_REG_SHUNTVOLTAGE)
        if value > 32767:
            value -= 65535
        return value * 0.01
        
    def getBusVoltage_V(self):  
        self.write(_REG_CALIBRATION,self._cal_value)
        self.read(_REG_BUSVOLTAGE)
        return (self.read(_REG_BUSVOLTAGE) >> 3) * 0.004
        
    def getCurrent_mA(self):
        value = self.read(_REG_CURRENT)
        if value > 32767:
            value -= 65535
        return value * self._current_lsb
        
    def getPower_W(self):
        value = self.read(_REG_POWER)
        if value > 32767:
            value -= 65535
        return value * self._power_lsb    
        
      
if __name__=='__main__':

    ina1 = INA219(addr=0x40)
    ina2 = INA219(addr=0x41)
    ina3 = INA219(addr=0x42)
    ina4 = INA219(addr=0x43)
    

    power1=0
    kwh1=0
    power2=0
    kwh2=0
    power3=0
    kwh3=0
    power4=0
    kwh3=0
    startingtime = time.time()
    #mistake here!!
#END OF INIT HAT



#SOLENOID AND WATER SENSOR
FLOW_SENSOR_GPIO = 11
openvalve = LED(26)
 
GPIO.setup(FLOW_SENSOR_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)
 
global count
count = 0
 
def countPulse(channel):
   global count
   if start_counter == 1:
      count = count+1
 
GPIO.add_event_detect(FLOW_SENSOR_GPIO, GPIO.FALLING, callback=countPulse)
#END OF SOLENOID AND WATER SENSOR
 

api_key = "bbea4d11baf8d6f6e1095e8f78dd6752"
lat = "36.88182094761954"
lon = "27.313360373938718"
url = "https://api.openweathermap.org/data/2.5/onecall?lat=%s&lon=%s&appid=%s&units=metric" % (lat, lon, api_key)

#api info for Adafruit IO
ADAFRUIT_USERNAME = "vgiorgara"
ADAFRUIT_IO_KEY = "aio_sxrb72BH21CDFCVhp16Hk7RuhiEm"
aio = Client(ADAFRUIT_USERNAME,ADAFRUIT_IO_KEY)

#get weather data
response = requests.get(url)
data = json.loads(response.text)
#GPIO.setmode(GPIO.BCM)

removewater = 0
averagedaytemp = data.get("daily")[0].get("temp").get("day")
clouds  = data.get("daily")[0].get("clouds")
rain  = data.get("daily")[0].get("rain")


#bin led

#room leds
led1 = LED(13)
led2 = LED(6)
binled=LED(21)

#fanpin
fanpin = 33

fan = LED(fanpin)


#function to get distance from ultrasonic
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance


# print(data)towater


#aio.send("dailytemp",3)


binfilled = False


while(1):
    #RFID SCAN\
    power1,power2,power3,power4 = getwattsa(power1,power2,power3,power4)
    print(rain,clouds,averagedaytemp)
#     id, text = reader.read()
#     print("dd")
#     if (text in names) and someoneinside==False:
#         #send to feed entered
#         someoneinside = True
#         aio.send("someone-inside",text+" IN")
#         currtemp = data.get("hourly").get("temp")
#         if day and light1status==False:
#             light1status == True
#             led1.on()
#             led2.on()
#             light2status == True       
#         if currtemp>21:
#             #openfan
#             fanopen = True       
#     elif (id not in names) and someoneinside == True:
#         someoneinside == False
#         aio.send("someone-inside","OUT")
#         if light1status==True:
#             light1status == False
#             light2status == False
#             led1.off()
#             led2.off()
#         if fanopen ==True:
#             fanopen = False    
    #the logic for the waterig
    dayspast = (date.today()-start_date).days
    if dayspast==1:
        if rain!= None:
            removewater = +rain*1000
        start_date = date.today()
        averagedaytemp = data.get("daily")[0].get("temp").get("day")
        clouds  = data.get("daily")[0].get("clouds")
        rain  = data.get("daily")[0].get("rain")
        num_of_days +=1
        start_date = date.today()  
        if averagedaytemp<10:
            if num_of_days>13:
                towater = LITREAMOUNT- removewater
                spraylemons(towater)
                removewater=0
                num_of_days = 0
                aio.send("waterused",towater)
        elif averagedaytemp<15:
            if num_of_days>9:
                towater = LITREAMOUNT - removewater
                spraylemons(towater)
                removewater=0
                num_of_days = 0
                aio.send("waterused",towater)
        elif averagedaytemp<20:
            if num_of_days>6:
                towater = LITREAMOUNT - removewater
                spraylemons(towater)
                removewater=0
                num_of_days = 0
                aio.send("waterused",towater)
        elif averagedaytemp<25:
            if num_of_days>2:
                towater = LITREAMOUNT - removewater
                spraylemons(towater)
                removewater=0
                num_of_days = 0
                aio.send("waterused",towater)
        elif averagedaytemp<30:
            if num_of_days>1:
                towater = LITREAMOUNT - removewater
                spraylemons(towater)
                removewater=0
                num_of_days = 0
                aio.send("waterused",towater)
        else:
            if num_of_days>0:
                towater = LITREAMOUNT - removewater
                spraylemons(towater)
                removewater=0
                num_of_days = 0
                aio.send("waterused",towater)
         
    dist = distance()
    print(dist)
    time.sleep(1)
    if dist<=5 and binfilled == False:
       binfilled = True
       binled.on()
       aio.send("bin",1)
    elif dist>5 and binfilled == True:
       binfilled = False
       binled.off()
       aio.send("bin",0)
       
