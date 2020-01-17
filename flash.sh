#!/bin/bash -xe

openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -f target/klx.cfg -f flash.cfg
