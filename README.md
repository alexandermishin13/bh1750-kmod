# BH1750 Ambient Light Sensor kernel driver for FreeBSD

![BH1750](/bh1750_sensor.jpeg?raw=true "BH1750 Ambient Light sensor")

## About

There is a ```FreeBSD``` kernel driver for a ```bh1750``` sensor.
It force the sensor to measure abmient light every 5 seconds and update
a corresponding sysctl branch.

## Installation

You need a FreeBSD kernel source codes.
```shell
make depend
make
sudo make install
```
You may need to edit fdt-overlay sources for Your needs, make and
install them:
```shell
cd fdt-overlay
make
sudo make install
```
Append an overlay to "/boot/loader.conf" and reboot:
<pre><code>
fdt_overlays="sun8i-h3-sid,sun8i-h3-ths<b>,sun8i-h3-bh1750-i2c0</b>"
</code></pre>

Now You can load the module and check its work:
```shell
kldload bh1750
sysctl dev.bh1750.0
```

## Description

The driver allows you to set device measurement parameters via sysctl
variables (and fdt-overlay parameters) and obtain result and calculated
parameters from the driver.
You can set and get the following parameters of the device:

* measurement mode to H-resolution or H-resolution2, which changes
the measurment sensitivity by 2 times;
* MTReg value, which leads to recalculation of the actual sensitivity
and, accordingly, a result ready time;
* percentage of reduced quality of the chip from the ideal one, which will
lead to a proportional increase of a result ready time.
The manufacturer allows a decrease in quality of chips up to 50%.
You may need it if You suspect that Your specimen is unable to complete the
measurement on time;
* polling time up to 255 seconds (through fdt-overlay parameter only by now.
I still think if it may needed to dynamically change it);
* get raw data from the chip as so called "counts";
* get calculated illuminance value in mlx.

## Status

The driver has been tested on "Raspberry Pi 2", "Orange Pi PC" and
"Orange Pi Zero" (sun8i-h2-plus-i2c0.dtbo is also needed).

Added cdev support for devices.

Added support of poll(2) and kevent(2).

Re-confirmation required:
* During testing, two sensors with addresses 0x23 and 0x5С were used
simultaneously.
