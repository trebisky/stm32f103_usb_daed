# stm32f103_usb_daed

This is a fork of the following project by "Daedalean AI" on Github:

[Daedalean AI stm32f103_usb](https://github.com/daedaleanai/stm32f103_usb)

Here is a link to the original README for the project:

[The original README](README_orig.md)

I found it, built it, ran it, and began to play with it, and as I began to
tinker with it, decided it would be beneficial to set up my own fork.

==============================

What I did as a first pass:

- minor change to main.c so it would compile
- add MAPLE ifdef for the Maple board I am working with
- change LED as per Maple
- add code for USB disconnect as per Maple
- Makefile: use openocd and stlink-v2 device for "make flash"
- Makefile: get rid of linker rwx warning
- main.c: add \r\n to all printf strings
- change serial console to 115200 baud

The code was originally written for the Blue Pill (which I have).
I have been doing USB experimentation using an original Maple board.
A nice advantage of these is that you get a USB disconnect just by
rebooting, rather than having to unplug/replug the USB cable.

For me to use this with linux, I needed to give the following commands
on my Fedora 38 system:

su
echo 0483 5722 >/sys/bus/usb-serial/drivers/generic/new_id

As soon as I did this, the board showed up as /dev/ttyUSB1 using the
linux usb-serial driver.  And it works!

The code also sets up a serial console on pins A9 and A10.
I already had a CP2102 gadget connected to those pins and was
able to see console messages (at 921600 baud!).

Later I got fed up with the crazy cbprintf scheme, ditched it
entirely and pulled in my own printf code, then reworked my
serial driver to use a queue and interrupts (otherwise polled
serial IO during enumeration broke the enumeration).

I hadn't planned to make such radical changed, but it seemed
like the only way to make progress.
