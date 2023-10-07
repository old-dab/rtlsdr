# Improvements, compared to the Osmocom version on git://git.osmocom.org/rtl-sdr.git

# Introduction
After buying my first USB stick in 2020 with tuner FC0012, I found that DAB (Digital Audio Broadcast) and FM reception was much better with software that used the original Realtek driver than with software that used the Osmocom driver. There were also a lot of dropouts under Windows with DAB. It quickly became apparent that the gain setting of the tuner in the Osmocom software was inadequate. I wanted to improve it. I started with Hayati's version, which had some advantages over Osmocom: https://github.com/hayguen/librtlsdr/tree/development . The source code of the Linux kernel has revealed many insights. Bernhard (DB9PP) was very helpful in investigating the tuners and he carried out most of the measurements. When it comes to tools, I focused on rtl_tcp.exe, which has been optimized specifically for QIRX but is still backwards compatible with the Osmocom version.
The main improvements are:
* Less data loss under Windows by not using LibUSB
* AGC for all tuners, control range 100 dB
* Additional filter bandwidths
* Second data channel reporting gain, overload and register contents
* More precise frequency setting and correction of the quartz frequency

# LibUSB
The Osmocom version uses LibUSB for all operating systems. This works very well on Linux. However, blocks of data are lost on slow PCs under Windows. This means that uninterrupted operation of DAB, for example, is not possible. The reason is that the LibUSB uses the WinUSB under Windows. To achieve maximum data throughput, the "RAW_IO" parameter should be set in WinUSB. Only then will the winusb.sys driver be able to buffer more than one USB bulk transfer. However, this is not yet possible with LibUSB. To solve the problem, one can either use a patched version of LibUSB that sets "RAW_IO" or omit the LibUSB altogether. I decided to leave it out. In librtlsdr.c, the WinUSB functions are called directly under Windows.

# fprintf
In librtlsdr.c, texts are output with fprintf to "stderr", whereas in rtl_tcp.c they are usually output with printf to "stdout". As a result, the different texts can overwrite each other under Windows and occasionally a jumble of characters appears. Therefore, I output all text messages uniformly using printf.

# Gain and AGC
For all tuners, the LNA gain is controlled by an internal AGC.
The IF amplifier can be set either by an AGC or by software. The RTL2832 enables AGC for IF amplification in the tuner. This is not programmed in the Osmocom version.
The R820T and R828D tuners require an analog voltage on an AGC line. This is achieved using pulse width modulation followed by an RC low pass. Unfortunately, the capacitor is missing in the "RTL-SDR.COM V4", so the IF-AGC does not work.
The tuners FC0012/13 and E4000 have a digital AGC. This requires 2 AGC lines. 00 and 11 means "Hold gain", 01 means "Increase gain", 10 means "Decrease gain". The FC2580 has an internal AGC.
The realization of the mixer gain is solved in different ways depending on the tuner. Sometimes there is its own AGC, sometimes it is coupled to the AGC of the LNA or the IF.
The total gain of all tuners can be read in steps of 1/10 dB. The accuracy is approximately ± 3 dB in the frequency range 50 MHz to 1 GHz , and ± 5 dB from 1 GHz up. The AGC achieves a control range of around 100 dB for all tuners.
A disadvantage of the IF-AGC is that the overall gain of the tuner can no longer be determined accurately. The reason for the tuners R820T and R828D is that the resistor in the RC low pass varies depending on the dongle manufacturer. This has an influence on the control voltage. The software is adapted to the "RTL-SDR.COM V3" for the R820T and to the "Astrometa DVB-T/DVB-T2" for the R828D. The tuners FC0012/13 and E4000 have two outputs (I and Q), so that the ADC has twice the voltage compared to the R820T/R828D. To ensure that nothing is overdriven, the digital AGC of the RTL2832 must be switched on. Unfortunately, the control status of the digital AGC cannot be read out.
That's why there is also a software AGC for the IF. However, in the event of adjacent channel interference, reception with hardware AGC is better.
Summary of all advantages and disadvantages: 

HW-AGC: 

    + best reception (except for ADS-B) 
    + fastest response to level changes 
    + easy operation 
    - level display unusable 

manual gain setting: 

    + accurate level display 
    - worse reception in the event of adjacent channel interference 
    - additional setting required 

SW-AGC: 

    + accurate level display 
    + easy operation 
    - worst reception in the event of adjacent channel interference 
    + delayed response to level changes

# FIR filter
There are three FIR filters in the RTL2832. The first works with a clock frequency of 28.8 MHz and is freely programmable. The second FIR filter operates at four times the sample rate of the output. The bandwidth is fixed to a value that matches the output sampling rate. A sampling rate of 2048 kHz results in a bandwidth of approximately 1.9 MHz. A third FIR filter, which works at the output sampling rate, can also be switched on. It is also freely programmable.
In the Osmocom software, the first FIR filter is set to a bandwidth of 2.4 MHz. The third FIR filter is switched off.
In order to achieve better adjacent channel attenuation, the first FIR filter can be set in my software to a bandwidth of 1.5 MHz and the third FIR filter to a bandwidth of 1.5 MHz or 300 kHz. The filters are set automatically along with the tuner bandwidth.

# Data transfer to and from the tuners
The tuners cannot be accessed directly via USB, but only via the RTL2832. The RTL2832 offers two options that differ in the USB protocol. Osmocom has opted for the more complex protocol with index register 0x600. I use the simpler protocol with index register 0x300. Less data is transferred via USB. Reading a tuner register can be done with a single USB transfer. The other protocol requires two USB transfers.

# More precise frequency adjustment
The tuner frequency can only be adjusted relatively roughly. With the R820T, for example, the smallest step size at 200 MHz is 27 Hz, and at 800 MHz it is even more than 100 Hz. In the RTL2832 this can be compensated by shifting the IF. This improves the setting accuracy to around 4 Hz. This also works when the IF is zero, such as on the E4000.

# Additional features of the Library API
* int rtlsdr_set_freq_correction_ppb(rtlsdr_dev_t *dev, int ppb): Sets the frequency correction for the tuner crystal in ppb (parts per billion). Thus, a value of 12345 corresponds to 12.345 ppm.
* int rtlsdr_get_freq_correction_ppb(): Returns the frequency correction for the tuner crystal in ppb.
* int rtlsdr_set_dithering(rtlsdr_dev_t *dev, int dither): Switches dithering on or off for the R820T/R828D tuners. 0 = off, 1 = on.
* void rtlsdr_cal_imr(const int cal_imr): Enables calibration for the R820T/R828D tuners to improve the rejection of the unwanted sideband. The command sets a global variable in the library. This must happen before a device is opened.
* int rtlsdr_set_tuner_gain_index(rtlsdr_dev_t *dev, unsigned int index): Sets the desired IF amplifier stage directly, without going through rtlsdr_set_tuner_gain.
* int rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t *dev, int mode): The parameter 'mode' is expanded. 0 = hardware AGC, 1 = manual gain mode, 2 = software AGC.
The following functions were adopted by or were created together with hayguen/librtlsdr:
* int rtlsdr_ir_query(rtlsdr_dev_t *dev, uint8_t *buf, size_t buf_len): Reads remote control data from the infrared sensor. buf = buffer for infrared data, buf_len = length of the data buffer. Returns 0 if no signal, >0 for number of bytes received, <0 for error.
* int rtlsdr_set_opt_string(rtlsdr_dev_t *dev, const char *opts, int verbose): Sets multiple command line options in a single string.
* const char * rtlsdr_get_opt_help(int longInfo): Lists the possibilities of "rtlsdr_set_opt_string". LongInfo = 0: in short version, LongInfo = 1: detailed.
* int rtlsdr_set_and_get_tuner_bandwidth(rtlsdr_dev_t *dev, uint32_t bw, uint32_t *applied_bw, int apply_bw ): Sets and and gets the tuner's bandwidth.
* int rtlsdr_set_tuner_sideband(rtlsdr_dev_t *dev, int sideband): Switches to the desired sideband in the tuner R820T/R828D. 0 = lower sideband, 1 = upper sideband.
* int rtlsdr_get_tuner_i2c_register(rtlsdr_dev_t *dev, unsigned char *data, int *len, int *strength): Allows reading all tuner registers. data = data buffer, len = provides number of data bytes, strength = provides absolute gain of the tuner in 1/10 dB.
* int rtlsdr_set_tuner_i2c_register(rtlsdr_dev_t *dev, unsigned i2c_register, unsigned mask, unsigned data): Sets a register in the tuner. i2c_register = number of the register, mask = bits to be overwritten in the register, data = data byte.

# Tuner
## R820T, R828D
* At Osmocom the IF amplifier is permanently set. LNA and mixer are adjustable, either with AGC or manually. In this way only about half of the possible dynamic range was available. In my version, the LNA and mixer always run with AGC, the IF amplifier can be set either with AGC or manually.
* Appropriate bandwidths for DAB and ADS-B were added and the center frequencies of all filters were corrected. "Filter extension under weak signal" is deactivated for a stable filter characteristic.
* The tuner has a register to optimize the rejection of the image frequency. With optimal settings, the suppression of the image frequency can be increased from around 50 dB to 60 dB. Because the optimal setting depends on the mixer gain, this must be measured for each level of mixer gain. For this purpose, the tuner contains its own calibration generator with which internal measurements are possible. A single call to rtl_tcp with the "-k" parameter takes about a second and saves the results in EEPROM. This means that the calibration does not have to be repeated every time the program starts.
* If that's still not sufficient, the sidebands can additionally be switched.
* The frequency setting had a small systematic error. The dithering offset was not taken into account. The exact VCO frequency is:
2 * pll_ref * (nint + (offset + sdm) / 65536).
offset = 0 if sdm = 0, offset = 0.25 if dithering is on, offset = 0.5 if dithering is off.
* Since the gain of the LNA is frequency dependent, it was measured for many frequencies. The measurements are contained in a table in the program code. The gain at other frequencies is determined by interpolation.
* To improve the 3rd harmonic suppression, register settings in the frequency range 100 to 588 MHz were changed (bits 0 and 1 in register 26).
* A description of all registers was added to the source code "tuner_r82xx.c", as far as known from the data sheets.

## FC0012 and FC0013
* The FC0012/13 tuners were hardly usable in the Osmocom version because the IF amplifier was not adjustable and the gain was random. The received signal was either overloaded or too weak. Only the LNA could be adjusted in coarse steps. Now the IF amplifier together with the mixer can be set manually or controlled by the AGC. The LNA has its own AGC output which is connected to a level detector in the RTL2832. This makes it possible to automatically control the LNA gain.
* The smallest adjustable bandwidth could be reduced from around 6.6 MHz to 5.0 MHz. There is a register for fine-tuning the bandwidth. To further improve the selection, the FIR filter in the RTL2832 can be switched to a smaller bandwidth.

## E4000
* An AGC can be switched on in the IF amplifier. The LNA is governed by an internal AGC. The mixer does not have its own AGC. It is either set manually together with the IF amplifier or controlled by the LNA-AGC.
* In the Osmocom version high gains in the IF amplifier could not be used because the automatic DC offset compensation was not switched on.
* Since the gain of the LNA is frequency dependent, it was measured for many frequencies. The measurements are contained in a table in the program code. The gain at other frequencies is determined by interpolation. There were significant deviations from the data sheet for the LNA, mixer and IF amplifier.
* In the Osmocom version the tuner bandwidth was incorrectly interpreted. The information in the data sheet refers to the bandwidth in the baseband. The RF bandwidth is twice as large. To improve selection, the FIR filter in the RTL2832 can be switched to a smaller bandwidth.
* The tuner outputs I and Q differ significantly in their amplitudes. This creates image frequencies and a worse SNR for DAB. Some programs like "SDRSharp" can compensate for this (check "Correct IQ"). For programs that lack this feature, you can set it in rtl_tcp with the "-c" option.

## FC2580
* Because you couldn't adjust the IF gain of this tuner, it couldn't be used meaningfully with the Osmocom software. Now the IF amplifier can be adjusted manually or controlled by an internal AGC. LNA and mixer are adjusted by internal AGCs.
* The frequency could only be set accurately to 1 kHz. Now the frequency can be adjusted to 1 Hz at 200 MHz.
* The smallest adjustable bandwidth was 5.5 MHz. Any bandwidth between 1.1 and 2.1 MHz can now be set.

# rtl_tcp.exe
In "rtl_tcp" a second TCP/IP connection can be activated, one port number higher than the normal connection. Controlled by a timer, this second connection sends the contents of the tuner registers, an overload message and the overall gain of the tuner in 1/10 dB to the PC every 500 ms. The periodic query of the tuner registers is also used to optimize the image frequency attenuation on the R820T and R828D and to set the LNA gain on the FC0012/13.
Additional command line parameters not included in the Osmocom version:

    -c (correct I/Q ratio)
    -k (calibrate image rejection for R820T/R828D and store the results in EEPROM)
    -r (response port)
    -u (upper sideband for R820T/R828D)
Additional TCP/IP features not included in the Osmocom version:
SET_TUNER_BANDWIDTH, SET_I2C_TUNER_REGISTER, SET_SIDEBAND, REPORT_I2C_REGS, SET_DITHERING, SET_FREQUENCY_CORRECTION_PPB.

## rtl2_tcp.exe
rtl2_tcp is a special version of rtl_tcp for QIRX with a different protocol. This makes it possible to select the USB device in a menu within QIRX when operating several sticks at the same time.

