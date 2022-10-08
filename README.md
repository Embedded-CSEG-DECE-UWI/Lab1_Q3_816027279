# ECNG3006 Lab 1 Question 3

* This example will show you how to use I2C module:
 
    * read external i2c sensor, here we use a ADS1115 sensor.

## Pin assignment

* master:
    * GPIO0 is assigned as the data signal of i2c master port
    * GPIO2 is assigned as the clock signal of i2c master port

## How to use example

### Hardware Required

* Connection:
    * connect sda/scl of sensor with GPIO0/GPIO2
    * no need to add external pull-up resistors, driver will enable internal pull-up resistors.

### Configure the project

```
make menuconfig
```

* Set serial port under Serial Flasher Options.


### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
make -j4 flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output  

```

```
