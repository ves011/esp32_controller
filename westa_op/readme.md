# hobby weather station controller
The components here are used to read data from 1 Bosch BMP280 pressure / temperature sensor and 1 Aosong DHT22 humidity sensor<br>
Communication and control with BMP280 sensor is done using Bosch API provided here https://github.com/BoschSensortec/BMP2-Sensor-API/ ported for ESP-IDF environment<br>
Many thanks to Bosch Sensortec division for providing such a portable code. Porting effort was close to zero.<br>
<br>
Communication with BMP280 is using ESP i2c standard driver for which i implemented read/write wrappers required by API intengration.<br>
Communication with DHT22 is implemented using RMT peripheral driver. There are several implementations on github, but i prefered to do my own, to ensure a better control of bitframe consistency.<br>
<br>
Finally you'll get values for absolute pressure, relative humidity and 2 values of temperature. 1 provided by BMP sensor and 1 provided by DHT sensor.<br>
In addition sea level reduced pressure is also provided based on normal pressure value at sea level and elevation of the measurement point.<br>
You need to introduce the parameters using the command below
<br>
>westa bmp pset [elevation in cm] [sea level pressuer in Pa]

The rate these values are sent to MQTT broker can be adjusted by modifying <b>PTH_POLL_INT</b> value in westaop.h. (currently is 30 mins).
The values are also saved in SPI in a text file "yyyy.txt" in user partition. I use a 8MB variant of ESP32-S3 on which i could reserve ~4MB for user partition.<br>
At 30 min continuos rate, 4MB should be enough for mor than 4 years. <br>
<br> 