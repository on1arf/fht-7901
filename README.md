# fht-7901

This is proof-of-concept code to show the ability of the sx1276 (AKA RFM95, AKA LORA32) to emulate the transmission of a remote control of a FHT-7901 power-socket. 

Datasheet:
  [SX1276 | 137MHz to 1020MHz Long Range Low Power Transceiver | Semtech](https://www.semtech.com/products/wireless-rf/lora-core/sx1276#download-resources)

  
I did not find any official documentation of the FHT-7901, but manuals of the device can be found in a number of places on the internet.
e.g. here:
  [Manual REV-Ritter FHT-7901 (page 1 of 2) (English, German, Spanish)](https://www.libble.eu/rev-ritter-fht-7901/online-manual-801705/)
  [Manual REV-Ritter FHT-7901 (page 1 of 2) (English, German, Spanish)](https://web.archive.org/web/20220320135742/https://www.libble.eu/rev-ritter-fht-7901/online-manual-801705/) (on archive.org)

  
The wireless remote-control of the FHT-7901 uses OOK (On-Off keying) modulation at 433.790 MHz at a speed of 6666 symbols per second.
  Data is encoded using 4 symbols/bit, according the following table:

```
0: 1000
1: 1110
```


For every keypress, the remote-control unit transmits up to 30 packets.
  The first packets of the transmission is the 25 bit "command-packet" this is repeated.
  The last three packets of the transmission is a fix 25 bit "end-marker" packet that has a fixed format. 
  
The format of the two different types of packets can be found in the source-code.
 
  
  
This program uses code from the "RadioHead" project, converted from c++ to python
  https://github.com/hallard/RadioHead/blob/master/RH_RF95.cpp
  https://github.com/hallard/RadioHead/blob/master/RH_RF95.h

  
 The original code of the RadioHead library was developed by Mike McCauley
 [RadioHead: RadioHead Packet Radio library for embedded microprocessors](http://www.airspayce.com/mikem/arduino/RadioHead/)
 
  
  
This code was tested on a TTGO T-beam, which uses a ESP32 and a "LORA32" radiochip.
  [GitHub - LilyGO/TTGO-T-Beam](https://github.com/LilyGO/TTGO-T-Beam)
  The LORA32 is identical to the semtech sx1278.
 
 
 
 The code is written in micropython for fast and easy development and experimentation. More info about about running micropython on a ESP32 can be found here:
  [Overview &mdash; MicroPython 1.18 documentation](https://docs.micropython.org/en/latest/index.html)
