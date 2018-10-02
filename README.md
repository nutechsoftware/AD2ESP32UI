# Example for the AlarmDecoder.com AD2ESP32UI wireless touch screen platform.

This code provides a framework for building a wireless touch screen interface for home alarm and automation systems using the AlarmDecoder.com AD2ESP32UI touch screen platform. 

The AD2ESP32UI platform has the following specifications.
- SparkFun [ESP32 Thing](https://www.sparkfun.com/products/13907).
- A quad speed spi bus connector for a  [**NHD-7.0-800480FT-CSXV-CTP**](http://newhavendisplay.com/learnmore/EVE2_7-CSXV-CTP/) display connected to the ESP32 VSPI pins.
- uSD card socket connected to the ESP32 HSPI pins.
- A MEMS microhpone [SPH0641LU4H-1](https://www.mouser.com/datasheet/2/218/-746191.pdf) connected to GPIO pins.
- External connections for i2c, uart or GPIO

## Getting the code
- git clone --recursive https://github.com/nutechsoftware/AD2ESP32UI.git

## Configuring
- Under menuconfig in **Serial flasher config** set the **Default serial port**
```console
foo@bar:~$ make menuconfig
```

## Building
- Requires an esp-idf development environment and **IDF-PATH** environment variable set to the path of your esp-idf folder (e.g., **IDF_PATH=/opt/esp/esp-idf** or **IDF_PATH=~/esp/esp-idf**).
  - https://esp-idf.readthedocs.io/en/latest/get-started/index.html
- Compile and program the ESP32 Thing
```console
foo@bar:~$ make test; make flash
```

## Screens
- Webcam interface
 - Connects to a wifi network and connect to a [motion](https://github.com/Motion-Project/motion) MJPEG URL and display it in real time.
 
## Contributors
 - Submit issues and contribute improvements on [github/nutechsoftware](https://github.com/nutechsoftware)

## Authors
 - Sean Mathews <coder@f34r.com> - Initial skeleton and R&D

## License
 - [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
