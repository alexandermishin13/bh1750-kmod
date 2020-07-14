#!/bin/sh

#
# Alexander Mishin, 2020
#
# Slightly modified version of script
# from "FreeBSD_I2C" project of "Takeshi MUTOH"
# https://github.com/610t/FreeBSD_I2C

I2C=/usr/sbin/i2c
DEV=/dev/iic0
ADDR=0x23
OPECODE=$'\x10'

printf ${OPECODE} | ${I2C} -f ${DEV} -a ${ADDR} -m tr -d w -v -w 0
sleep 0.1

while true
do
    ${I2C} -f ${DEV} -a ${ADDR} -m tr -d r -c 2 -v -w 0
    sleep 1
done
