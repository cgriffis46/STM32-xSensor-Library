# RTClib [![Build Status](https://github.com/adafruit/RTClib/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/RTClib/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/RTClib/html/index.html)

This is a fork of JeeLab's fantastic real time clock library for RP2040.

Works great with Adafruit RTC breakouts:

- [DS3231 Precision RTC](https://www.adafruit.com/product/3013)
- [PCF8523 RTC](https://www.adafruit.com/product/3295)
- [DS1307 RTC](https://www.adafruit.com/product/3296)

Please note that dayOfTheWeek() ranges from 0 to 6 inclusive with 0 being 'Sunday'.

Compatability: RP2040 

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

# Dependencies

# Contributing

Contributions are welcome! Please read our [Code of Conduct](https://github.com/adafruit/RTClib/blob/master/code-of-conduct.md)
before contributing to help this project stay welcoming.

## Documentation and doxygen
For the detailed API documentation, see https://adafruit.github.io/RTClib/html/index.html
Documentation is produced by doxygen. Contributions should include documentation for any new code added.

Some examples of how to use doxygen can be found in these guide pages:

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips

## Code formatting and clang-format
The code should be formatted according to the [LLVM Coding Standards](https://llvm.org/docs/CodingStandards.html), which is the default of the clang-format tool.  The easiest way to ensure conformance is to [install clang-format](https://llvm.org/builds/) and run

```shell
clang-format -i <source_file>
```

See [Formatting with clang-format](https://learn.adafruit.com/the-well-automated-arduino-library/formatting-with-clang-format) for details.

Written by JeeLabs
MIT license, check license.txt for more information
All text above must be included in any redistribution

To install, use the Arduino Library Manager and search for "RTClib" and install the library.
