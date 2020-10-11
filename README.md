# BooSTIno
A Teensy 3.6-based logger and gauge for my 2014 Subaru STI.

Photo of current version:

[![BooSTIno](https://i.ibb.co/PNfrHdX/IMG-9936.jpg)](https://ibb.co/G0bdSqY)

Short [YouTube video](https://www.youtube.com/watch?v=nscQxxUsuL8) of older version, without faceplate or wideband support:

[![YouTube](http://img.youtube.com/vi/nscQxxUsuL8/0.jpg)](https://www.youtube.com/watch?v=nscQxxUsuL8)

See [this thread on NASIOC](https://forums.nasioc.com/forums/showthread.php?t=2896098) for more info.

# Compilation

This project is built as an Arduino sketch. I used Arduino 1.8.8, but later versions will likely work too. You will also need the Teensyduino add-on
(https://www.pjrc.com/teensy/td_download.html), and the following libraries:

* My fork of ILI9341_t3: https://github.com/jasminpatry/ILI9341_t3
* My fork of Adafruit_GFX: https://github.com/jasminpatry/Adafruit-GFX-Library
* font_Exo-BoldItalic: https://github.com/FrankBoesing/fonts/tree/master/ofl/exo

# Hardware

This project was built using the following hardware:

* [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)
* MC33660 K-line serial interface chip, SOP-8 package ([datasheet](https://www.nxp.com/docs/en/data-sheet/MC33660.pdf)); I purchased mine from [eBay](https://www.ebay.com/itm/10PCS-MC33660-MC33660EF-SOP-8/292559702234)
* Custom PCB fabricated using the files in the [KiCad/boostino.gerber](KiCad/boostino.gerber) directory (I used [BasicPCB](https://www.basicpcb.com/)'s ValueSpec option, which in 2020 cost me about $30 for 3 boards).
* [HiLetgo ILI9341 2.8" SPI TFT LCD Display Touch Panel 240X320 with PCB 5V/3.3V STM32](https://www.amazon.com/gp/product/B073R7BH1B/)
* [OBD-2 cable](https://www.amazon.com/gp/product/B01ETRINYO/) &mdash; the switch and DB9 connector in the linked cable aren't necessary (I snipped them off), but the DB9 connector was useful when prototyping.
* [Innovate Motorsports (3877) LC-2 Digital Wideband Lambda Controller Kit with Bosch LSU 4.9 O2 Sensor](https://www.amazon.com/Innovate-Motorsports-Digital-Wideband-Controller/dp/B00FFTAJPC/) (optional; only necessary for AFR display and logging)
* [Molex Micro-Fit 3.0 plug](https://www.amazon.com/gp/product/B078Q798L9/) for connecting to the LC-2 serial output (optional; only necessary for AFR display and logging)
* [Elcoho 10 Pieces Plastic Waterproof Boxes Junction Case Compatible with Electronic Project 3.94 × 2.36 × 0.98 Inches, Black](https://www.amazon.com/gp/product/B07G8S6XLV/)
* For the faceplate, I had [faceplate.eps](faceplate.eps) laser-cut by https://www.ponoko.com/ using:
    * Acrylic - Two Color - Brushed Silver on Black
    * 0.059 inches thick
    * P1 - 7.126 inches long x 7.126 inches wide
* Miscellaneous cables, headers, resistors, capacitors, diodes, wires, screws & nuts, etc.

# Tactrix Version

A previous version of this project used the [Tactrix OpenPort 2.0](https://www.tactrix.com/index.php?option=com_virtuemart&page=shop.product_details&product_id=17&vmcchk=1&Itemid=53&redirected=1&Itemid=53) to communicate with the ECU. That version is available in the [Tactrix](../../tree/Tactrix/) branch.

# Donate

If this project proves useful to you, please consider making a donation!

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donate_SM.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=P3N5R4B3SER8S&currency_code=USD&source=url)
