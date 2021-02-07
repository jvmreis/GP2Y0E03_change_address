# I2C Self-Test Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

## Overview

This example demonstrates basic usage of I2C driver by running two tasks on I2C bus:

1. Read external I2C sensor, here we take the GP2Y0E03 distance sensor  for an example.


## How to use example

### Hardware Required

To run this example, you should have one ESP32 dev board (e.g. ESP32-WROVER Kit) or ESP32 core board (e.g. ESP32-DevKitC). Optionally, you can also connect an external sensor, here we choose the GP2Y0E03 just for an example. BH1750 is a digital ambient light sensor, for more information about it, you can read the [PDF](http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1721fvc-e.pdf) of this sensor.

#### Pin Assignment:

**Note:** The following pin assignments are used by default, yout can change these  in the `menuconfig` .

|                  | SDA    | SCL    |
| ---------------- | ------ | ------ |
| GP2Y0E03.c Sensor    | SDA    | SCL    |


- master:
  - GPIO18 is assigned as the data signal of I2C master port
  - GPIO19 is assigned as the clock signal of I2C master port

- Connection:
  - connect SDA/SCL of GP2Y0E03 sensor with GPIO18/GPIO19

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

### Configure the project

Open the project configuration menu (`idf.py menuconfig`). Then go into `Example Configuration` menu.



### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output


## Troubleshooting

- GP2Y0E03 has two I2C address, which is decided by the voltage level of `ADDR` pin at start up. Make sure to check your schemetic before run this example.

(For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you as soon as possible.)
