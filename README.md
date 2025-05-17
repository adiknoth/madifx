# Linux ALSA driver for RME MADI FX (WIP)

This repository contains a Linux kernel driver for RME MADI FX cards. It is
still work in progress. Once finished, it'll be submitted to the mainline
kernel.

**Until then, everything may change at any time, so don't expect the userpsace
API to be stable.**

## Disclaimer

This driver has been developed via ssh on a remote machine with a single static
MADI crossover connection to a PCI RME MADI card, so not all ports have been
tested. Most importantly, the channel mappings might be wrong.

Like always, no guarantees at all, use at your own risk. Feel free to report
problems and patches to improve the driver.

## NEWS

* 2021-08-16: Update driver source to compile on 5.12+ kernels.
* 2021-04-17: Add patch for CentOS7
* 2020-05-07: Updated driver source to compile on 5.5+ kernels.
* 2020-03-02: Updated driver source to compile on 5.1+ kernels.
* 2018-06-03: Updated driver source to compile on 4.15+ kernels.

## Installation

Clone this repository. If you're on a CentOS kernel, don't patch anything.
CentOS backports the new audio stack to their old kernels.

For all kernels before 4.15, run

```
   git checkout v3.16_or_later
```

first. Then, depending on your kernel version, run the following additional
commands:

For all kernels until 3.14.x, run

```
   patch -p1 < below315.patch
```

For all kernels until 3.7.x, run

```
    patch -p1 < below38.patch
```

For all kernels until 3.4.x, *also* run

```
    patch -p1 < oldkernel.patch
```

Then, run

    make install

as root. Use `alsamixer` or `amixer` to adjust settings.

## Status

*   PCM playback/capture working (SS and DS tested, QS untested)
*   MIDI working
*   All card settings working (e.g. TX64, SMUX, AESpro, WC-Term,
    WC-singlespeed...)
*   Slave Mode/External clock selection working
*   Mirror-MADI1-to-Out2+3 maybe working (untested)
*   Redundancy mode maybe working (untested)
*   ioctls implemented (see ioctl.c)
*   Static mixer working (fixed 1:1 mapping)
*   DSP **NOT** working. RME doesn't intend to release any information regarding
    the DSP.
*   Adjustable mixer **NOT** working (needs new userspace tools)
*   Levelmetering **NOT** working (maybe wrong, needs new userspace tools)

## Signing the driver

On modern Linux distributions, especially when SecureBoot comes into play, you might need to sign the driver prior to trying it out or to install it. [This StackExchange post](https://unix.stackexchange.com/a/751571) explains pretty well what is to be done.

## TODO

*   Cleanup code (still contains plenty of the HDSPM driver)
*   Implement new userspace tools (see above)
*   Submit to mainline

## Hacking

If you want to work on mixer/level support, compile with

`make BROKEN_WIP=1`

to include the broken mixer/level kernel code. The name is pretty obvious, it's
meant for driver/tool developers, not users.

## Acknowledgement

Thanks to [IOSONO](http://www.iosono-sound.com/) for hosting the card and
providing the remote login. Kudos to Andre Schramm for taking care of the setup.

Thanks to [RME](http://www.rme-audio.com) for temporarily providing the card.
Special thanks to Martin Björnsen for the OSX driver source.

Kudos to Marcin Pączkowski from the Center For Digital Arts And Experimental
Media ([DXARTS](http://www.dxarts.washington.edu/)) for setting up remote access
and testbeds to further improve the driver.

## Contact

In case of trouble or questions, send me an email:

[Adrian Knoth](mailto:adi@drcomp.erfurt.thur.de)
