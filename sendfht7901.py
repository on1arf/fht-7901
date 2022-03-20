# proof of concept code to use a sx1278 radio-module in OOK mode to
# 	control a FHT7901 powersplug


"""
This is proof-of-concept code to show the ability of the sx1276 (AKA RFM95, AKA LORA32) to emulate the transmission of a remote control of a FHT-7901 power-socket.
Datasheet : SX1276 | 137MHz to 1020MHz Long Range Low Power Transceiver | Semtech

I did not find any official documentation of the FHT-7901, but manuals of the device can be found in a number of places on the internet. e.g. here:

- https://www.libble.eu/rev-ritter-fht-7901/online-manual-801705/
- https://web.archive.org/web/20220320135742/https://www.libble.eu/rev-ritter-fht-7901/online-manual-801705/ (on archive.org)


The wireless remote-control of the FHT-7901 uses OOK (On-Off keying) modulation at 433.790 MHz at a speed of 6666 symbols per second.

Data is encoded using 4 symbols/bit, according the following table:
0: 1000
1: 1110

_: 0000 (no transmission, is used inbetween packets)


For every keypress, the remote-control unit transmits up to 30 packets.

1/ The first packets of the transmission is the 25 bit "command-packet" this is repeated.

The format of the command-packet is as follows:
N4 1 - N3 1 - N2 1 - N1 1 - N0 1 - 0 - D0 0 - D1 0 - D2 0 - D3 0 - D4 0 - B0 0 - B1.0

- bits 0 to 9: network address
N4 down to N0: Address bits. This corresponds to the 5-bit "network-address" that needs to be set in the remove-control and every power-scoket using the DIP switches.
(see user-manual for more info)

In the code-example below, network-address '11111' is used. Note that after every address-bit a bit '1' is sent.

- bit 10:
This bit has the fixed value of '0'

- bits 11 to 19: device address 
D0 up to D4: device selector. This corresponds to the buttons "A' up to "E" on the remote control and the "B" group of dip-switches on the power-socket.
The encoding is as follows:
A: 00 10 10 10 10
B: 10 00 10 10 10
C: 10 10 00 10 10
D; 10 10 10 00 10
E: 10 10 10 10 00


- bits 20 to 24: "button" on/off selector
These 4 bits are encoded as follows
ON : 10 00 (left button on remote control)
OFF: 00 10 (right button on remote control)


2/ The last three packets of the transmission is a fix 25 bit "end-marker" packet that has a fixed format.

N4 1 - N3 1 = N2 1 - N1 1 - N0 1 - 0 - 10 10 10 10 11 11 10 

Multiple packets are sent for one button-press on the remote control.
A packet has a length of 150 uS (hence, the 6666.7 baud symbolrate), In between packets, 28 symbol-time (4.2 ms) of pause (no transmission) is added.






This program uses code from the "RadioHead" project, converted from c++ to python
https://github.com/hallard/RadioHead/blob/master/RH_RF95.cpp
https://github.com/hallard/RadioHead/blob/master/RH_RF95.h


The original code of the RadioHead library was developed by Mike McCauley [RadioHead: RadioHead Packet Radio library for embedded microprocessors]
http://www.airspayce.com/mikem/arduino/RadioHead/

This code was tested on a TTGO T-beam, which uses a ESP32 and a "LORA32" radiochip.
https://github.com/LilyGO/TTGO-T-Beam
The LORA32 is identical to the semtech sx1278.

The code is written in micropython for fast and easy development and experimentation. More info about about running micropython on a ESP32 can be found here:
https://docs.micropython.org/en/latest/index.html

"""

from machine import SPI
from machine import Pin
import array
import time

XMITFREQ=433.7900 		# frequency used by the FHT7901 remote control


##### lora32 specific information for SPI
spi = SPI(2, baudrate=100000, polarity=0, bits=8, phase=0, sck=Pin(5), mosi=Pin(27), miso=Pin(19))
nss = Pin(18, Pin.OUT) # GPIO5 = D5 = 5 = VSPI_CS0
resetpin = Pin(23, Pin.OUT) # GPIO22 = D22 = 22 = resetpin


##### global data

# 'FSK/OOK' registers of the sx1278
RH_RF95_REG_00_FIFO    = const(0x00)	# RegFifo
RH_RF95_REG_01_OP_MODE = const(0x01)	# RegOpMode
RH_RF95_REG_02_BRT_MSB = const(0x02)	# RegBitrateMsb
RH_RF95_REG_03_BRT_LSB = const(0x03)	# RegBitrateLsb
RH_RF95_REG_06_FRF_MSB = const(0x06)	# RegFrfMsb
RH_RF95_REG_07_FRF_MID = const(0x07)	# RegFrfMid
RH_RF95_REG_08_FRF_LSB = const(0x08)	# RegFrfLsb
RH_RF95_REG_09_PA_CNFG = const(0x09)	# RegPaConfig
RH_RF95_REG_0A_PA_RAMP = const(0x0a)	# RegPaRamp


RH_RF95_REG_25_PRA_MSB	= const(0x25)	# RegPreambleMsb
RH_RF95_REG_26_PRA_LSB	= const(0x26)	# RegPreambleLsb

RH_RF95_REG_27_SYN_CFG = const(0x27)	# RegSyncConfig
RH_RF95_REG_30_PKT_CF1	= const(0x30)	# RegPacketConfig1
RH_RF95_REG_31_PKT_CF2	= const(0x31)	# RegPacketConfig2
RH_RF95_REG_32_PAY_LEN	= const(0x32)	# RegPayloadLength

RH_RF95_REG_4D_PA_DAC	= const(0x4d)	# RegPaDac



# Mode of operations
RH_RF95_MODE_SLEEP = const(0x00)
RH_RF95_MODE_STDBY = const(0x01)
RH_RF95_MODE_TX    = const(0x03)
RH_RF95_MODE_OOK   = const(0x20)


# OOK modulation filters
RH_RF95_OOK_NO_SHAPING = const(0x00)	# OOK modulation filter, No Filter
RH_RF95_OOK_FILTER_BR  = const(0x20)	# OOK modulation filter, f_cutoff = BR
RH_RF95_OOK_FILTER_2BR = const(0x40)	#  OOK modulation filter, f_cutoff = 2*BR

# some other stuff
RH_RF95_FXOSC = 32000000.0
RH_RF95_FSTEP = (RH_RF95_FXOSC / 524288)

RH_SPI_WRITE_MASK = const(0x80)

RH_RF95_PA_DAC_DISABLE = const(0x04)
RH_RF95_PA_DAC_ENABLE  = const(0x07)
RH_RF95_PA_SELECT      = const(0x80)




#### Support functions

# Do Reset
def DoReset():
	resetpin.off()
	time.sleep_ms(100)
	resetpin.on()

#end doreset


# SPI write (with different datatypes)
def spi_write(register,data, nodrop=False):
	def d2bytes(d):
		if type(d) == int: return bytes([d])
		elif type(d) == str: return bytes([ord(c) for c in d])
		elif type(d) == bytes: return d
		elif type(d) == list: return [d2bytes(c) for c in d]
		else:
			print("unknown datatype in spi_write"+str(type(d)))
		
	#end def
	
	nss.off()
	spi.write(d2bytes(register | RH_SPI_WRITE_MASK))

	towrite=d2bytes(data)
	
	if (type(towrite) == bytes):
		spi.write(towrite)
	elif (type(towrite) == list):
		for c in towrite:
			spi.write(c)
	else:
		print("unknown datatype in spi_write"+str(type(data)))
		print(type(data))

	if nodrop == False:
		nss.on()

# end "spi_write"

# SPI read
# spi_read returns bytes!
def spi_read(register):
	data=array.array('H')

	nss.off()
	spi.write(bytes([register & ~RH_SPI_WRITE_MASK]))
	data=spi.read(1)
	nss.on()
	return(data)

# end "spi_read"


# Set Frequency
def setFrequency(centre):
	frf = (centre * 1000000.0) / RH_RF95_FSTEP
	spi_write(RH_RF95_REG_06_FRF_MSB, (int(frf) >> 16) & 0xff)
	spi_write(RH_RF95_REG_07_FRF_MID, (int(frf) >> 8) & 0xff)
	spi_write(RH_RF95_REG_08_FRF_LSB, int(frf) & 0xff)
	_usingHFport = (centre >= 779.0)

# end "setFrequency"




def setTxPower(power):
	if (power > 23):
		power = 23
	if (power < 5):
		power = 5

	if (power > 20):
		spi_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE)
		power -= 3
	else:
		spi_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE)

	spi_write(RH_RF95_REG_09_PA_CNFG, RH_RF95_PA_SELECT | (power-5))

# end "setTxPower"



def initchip():
	DoReset()
	time.sleep(1)

	# set chip in OOK and SLEEP mode
	spi_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP) 
	

	#set baudrate
	# 6666.7 baud: 0x12,0xc0  hex(round(RH_RF95_FXOSC/6666.7))
	spi_write(RH_RF95_REG_02_BRT_MSB, 0x12) 
	spi_write(RH_RF95_REG_03_BRT_LSB, 0xc0) 


	# set frequency
	setFrequency(XMITFREQ) # 433.790 in this case

	# set tx power
	setTxPower(5) # minimal power level

	# go to mode idle
	spi_write(RH_RF95_REG_01_OP_MODE,RH_RF95_MODE_STDBY)
	

	# set length of preamble to 0
	# this disabled the preamble
	spi_write(RH_RF95_REG_25_PRA_MSB, 0)
	spi_write(RH_RF95_REG_26_PRA_LSB, 0)	


	# setting up packet mode
	spi_write(RH_RF95_REG_27_SYN_CFG, 0x00) # no sync word
	spi_write(RH_RF95_REG_30_PKT_CF1, 0x00) # fixed length, no DC-free encoding, no crc, no addr filter)
	spi_write(RH_RF95_REG_31_PKT_CF2, 0x40) # packet mode, no io-homecontrol compat. mode
	spi_write(RH_RF95_REG_32_PAY_LEN, 15)   # packet length = 13 octets (100 bits + 4 bits unused)

	# set OOK Shaping
	spi_write(RH_RF95_REG_0A_PA_RAMP, RH_RF95_OOK_FILTER_BR) # set OOK filtering to 1 * Bitrate

#end init chip


################
# Main code starts here

# init chip
initchip()



"""
packet format 
format:

N4.1 N3.1 N2.1 N1.1 N0.1 -  0  -  D0.0 D1.0 D2.0 D3.0 D4.0 -  P0.0 P1.0

_ = 0000 = 0x0 (no transmission)
0 = 1000 = 0x8
1 = 1110 = 0xE

"""


#device A
#        __   __   __   _1   11   11   11   11   10   00   10   10   10   10   10   00
devAL=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0x88,0xe8,0xe8,0xe8,0xe8,0xe8,0x88]
#        __   __   __   _1   11   11   11   11   10   00   10   10   10   10   00   10
devAR=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0x88,0xe8,0xe8,0xe8,0xe8,0x88,0xe8]

# device B
#        __   __   __   _1   11   11   11   11   10   10   00   10   10   10   10   00
devBL=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0x88,0xe8,0xe8,0xe8,0xe8,0x88]
#        __   __   __   _1   11   11   11   11   10   10   00   10   10   10   00   10
devBR=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0x88,0xe8,0xe8,0xe8,0x88,0xe8]

#device C
#        __   __   __   _1   11   11   11   11   10   10   10   00   10   10   10   00
devCL=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0xe8,0x88,0xe8,0xe8,0xe8,0x88]
#        __   __   __   _1   11   11   11   11   10   10   10   00   10   10   00   10
devCR=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0xe8,0x88,0xe8,0xe8,0x88,0xe8]

#device D
#        __   __   __   _1   11   11   11   11   10   10   10   10   00   10   10   00
devDL=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0xe8,0xe8,0x88,0xe8,0xe8,0x88]
#        __   __   __   _1   11   11   11   11   10   10   10   10   00   10   00   10
devDR=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0xe8,0xe8,0x88,0xe8,0x88,0xe8]

#device E
#        __   __   __   _1   11   11   11   11   10   10   10   10   10   00   10   00
devEL=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0xe8,0xe8,0xe8,0x88,0xe8,0x88]
#        __   __   __   _1   11   11   11   11   10   10   10   10   10   00   00   10
devER=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0xe8,0xe8,0xe8,0x88,0x88,0xe8]



CMDEND=[0x00,0x00,0x00,0x0e,0xee,0xee,0xee,0xee,0xe8,0xe8,0xe8,0xe8,0xe8,0xee,0xee,0xe8]




devAon =devAL*12+CMDEND*3
devAoff=devAR*12+CMDEND*3

devBon =devBL*12+CMDEND*3
devBoff=devBR*12+CMDEND*3

devCon =devCL*12+CMDEND*3
devCoff=devCR*12+CMDEND*3

devDon =devDL*12+CMDEND*3
devDoff=devDR*12+CMDEND*3

devEon =devEL*12+CMDEND*3
devEoff=devER*12+CMDEND*3



# configure payload length (fixed value)
spi_write(RH_RF95_REG_32_PAY_LEN, len(devAon)) # all messages are the same size (240 octets)


# as the chip is configured in packet-mode, we can already enable transmit mode
# the chip will only transmit when sufficiant data is send to the FIFO buffer
spi_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX | RH_RF95_MODE_OOK)




spi_write(RH_RF95_REG_00_FIFO,devAon)
spi_write(RH_RF95_REG_00_FIFO,devAoff)

spi_write(RH_RF95_REG_00_FIFO,devBon)
spi_write(RH_RF95_REG_00_FIFO,devBoff)

spi_write(RH_RF95_REG_00_FIFO,devCon)
spi_write(RH_RF95_REG_00_FIFO,devCoff)

spi_write(RH_RF95_REG_00_FIFO,devDon)
spi_write(RH_RF95_REG_00_FIFO,devDoff)

spi_write(RH_RF95_REG_00_FIFO,devEon)
spi_write(RH_RF95_REG_00_FIFO,devEoff)


