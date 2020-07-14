# BH1750 Ambient Light Sensor kernel driver for FreeBSD

![BH1750](/bh1750_sensor.jpeg?raw=true "BH1750 Ambient Light sensor")

## About

There is a `FreeBSD` kernel driver for a `bh1750` sensor.
It force the sensor to measure abmient light every 5 seconds and update
a corresponding sysctl branch.

## Installation

You need a FreeBSD kernel source codes.
```
make depend
make
sudo make install
```
Append an overlay to "/boot/loader.conf" and reboot:
<pre><code>
fdt_overlays="sun8i-h3-sid,sun8i-h3-ths<b>,sun8i-h3-bh1750-i2c0</b>"
</code></pre>

Now You can load the module and check its work:
```
kldload bh1750
sysctl dev.bh1750.0
```

## Status

Quick but worked version that would be ubdated