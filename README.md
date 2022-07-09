# oximeter
A small oximeter using MAX30102/STC8G1K08A/0.5-inch-OLED/Phase-Detect algorithm

features:

1. SpO2 detection.
2. Heart beat rate detection.
3. With a USB charging interface.
4. Small and cheap.
5. Compatible with Keil C51 or SDCC

!!!IMPORTANT!!!

First thing:
I made an ergonomic mistake in my design, my original idea was to use a button to turn on the power and the sensor underneath would just fit the finger.
But the excessive pressure affected the blood flowing in the finger, also the results.
I suggest that subsequent developers can modify the PCB and box case, use a toggle switche instead of the button, which can be placed on the side of the case, and place the sensor on the same side of the OLED (Top side), So the users can just touch the sensor surface gently by a finger, that would result in fast and accurate detection results.

Second thing:
Hex file for burning/Downloading is included here, TTL serial port is connected to D+/D- pin of USB charging interface, subsequent developers can also try to download hex file directly using pseudo USB downloading port of STC8G1K08A (but I've never tried, and I heard that there is a certain failure rate).
But STC-ISP V6.90 (the STC official downloading software) has an strange BUG, which will cause straightforward downloading to fail.
In my design, I put const parameters like fonts and some tables into EEPROM area (which is indexed from 0x2000), STC-ISP says "This file is over the valid scope, and the exceeding part has been moved to EEPROM area", which looks as expected, BUT it is not. STC-ISP does not split the hex file at 0x2000, but at 0x1FF9.
The workaround for this strange problem is to manually delete first 7 bytes at "EEPROM Buffer" tab-page on the STC-ISP interface, and "Save Buffer" to a .bin file, and re-load the file using "Open EEPROM file", the burn it.

Other downloading things:
IRC frequency is 22.1184MHz.
Don't enable WDT.
Rest pin is used as I/O port.
