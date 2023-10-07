[![librtlsdr version](https://img.shields.io/github/tag/librtlsdr/librtlsdr.svg?style=flat&label=librtlsdr)](https://github.com/old-dab/rtlsdr/releases)
[![GPLv2 License](http://img.shields.io/badge/license-GPLv2-brightgreen.svg)](https://tldrlegal.com/license/gnu-general-public-license-v2)

# Description

rtl-sdr turns your Realtek RTL2832 based DVB dongle into a SDR receiver


# New enhancements and features in this version

Many different developments have been taken in this release. For an overview, see [improvements](README_improvements.md)

# Build / Install (on debian/ubuntu)

## prerequisites
development tools have to be installed:
```
sudo apt-get install build-essential cmake git
```

install the libusb-1.0 development package::
```
sudo apt-get install libusb-dev libusb-1.0-0-dev
```

## retrieve the sources - right branch

```
git clone https://github.com/old-dab/rtlsdr.git
cd rtlsdr
```

## build
run cmake and start compilation. cmake will accept some options, e.g.
* `-DINSTALL_UDEV_RULES=ON`, default is `OFF`
* `-DDETACH_KERNEL_DRIVER=ON`, default is `OFF`
* `-DENABLE_ZEROCOPY=ON`, default is `OFF`

all cmake options are optional

```
mkdir build && cd build
cmake ../ -DINSTALL_UDEV_RULES=ON
make
```

## install
setup into prefix, usually will require `sudo`:
```
sudo make install
sudo ldconfig
```

# Development builds / binaries

[GitHub Releases](https://github.com/old-dab/rtlsdr/releases) is used for development builds for Windows 32/64.

# For more information see:

http://superkuh.com/rtlsdr.html

https://osmocom.org/projects/rtl-sdr/wiki/Rtl-sdr


# Setup for SDR only use - without DVB compatibility:

- for permanent blacklisting you might check/call following from the clone git directory
    ```./install-blacklist.sh```
