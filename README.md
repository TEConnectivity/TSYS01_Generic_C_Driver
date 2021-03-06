# TSYS01 Generic C Driver
Generic C driver for the [TSYS01 sensor](http://www.te.com/usa-en/product-G-NICO-018.html)

![tsys01](http://www.te.com/content/dam/te-com/catalog/part/0GN/ICO/018/G-NICO-018-t1.jpg/jcr:content/renditions/product-details.png)

The TSYS01 sensor is a self-contained temperature sensor that is  fully calibrated during manufacture. The sensor can operate from 2.2V to 3.6V. The TSYS01 has a low power stand-by mode for power-sensitive applications. 

### Specifications
* Measures temperature from -40°C to 125°C
*	I2C communication
*	Fully calibrated
*	Fast response time
*	Very low power consumption


### Driver features
* Connection test
* Reset
* Aquisition resolution management
* Temperature measurement


**NB:** This driver is intended to provide an implementation example of the sensor communication protocol, in order to be usable you have to implement a proper I2C layer for your target platform.
