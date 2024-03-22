## DS3231 Aging Calibration using GPS PPS

### An inexpensive GPS module with a PPS output can be used to set the Aging
### register of a DS3231 RTC chip to the optimum value for accurate timekeeping.


This is a sketch for Arduinos using the ATmega328P processor, which includes
the Uno, Nano and Pro Mini.  The GPS module used was the Goouuu GT-U7, but
any GPS module with a PPS output should work.  And any DS3231 module should
work, with either the DS3231SN or the DS3231M chip.

This process should find the optimum setting within a couple hours as opposed
to spending days adjusting Aging to get the best time.  The setting is unique
to each RTC chip.

Everything is explained in the PDF.
