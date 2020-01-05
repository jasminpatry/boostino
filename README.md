# BooSTIno
A Teensy 3.6/Tactrix OP 2.0 logger and gauge for my 2014 Subaru STI.

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
* [Tactrix OpenPort 2.0](https://www.tactrix.com/index.php?option=com_virtuemart&page=shop.product_details&product_id=17&vmcchk=1&Itemid=53&redirected=1&Itemid=53)
* [HiLetgo ILI9341 2.8" SPI TFT LCD Display Touch Panel 240X320 with PCB 5V/3.3V STM32](https://www.amazon.com/gp/product/B073R7BH1B/)
* [Elcoho 10 Pieces Plastic Waterproof Boxes Junction Case Compatible with Electronic Project 3.94 × 2.36 × 0.98 Inches, Black](https://www.amazon.com/gp/product/B07G8S6XLV/)
* Protoboard; I used a board from [this set](https://www.amazon.com/gp/product/B074X2GDH2), trimmed to fit, but any protoboard should work fine.
* For the faceplate, I had [faceplate.eps](../master/faceplate.eps) laser-cut by https://www.ponoko.com/ using:
    * Acrylic - Two Color - Brushed Silver on Black
    * 0.059 inches thick
    * P1 - 7.126 inches long x 7.126 inches wide
* Miscellaneous cables, headers, wires, screws & nuts, etc. 

# Donate

If this project proves useful to you, please consider making a donation!

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donate_SM.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=P3N5R4B3SER8S&currency_code=USD&source=url)
