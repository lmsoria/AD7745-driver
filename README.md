# AD7745/6 Driver
This is a little driver I wrote for the Analog Devices AD7745 (works the same for AD7746) for a humidity sensor proyect a few years ago. It's written in C and its meant to be as simple as possible, without losing details. This project is inspired in a repository I saw here on GitHub on 2016, but I couldn't find it to link it here (Thank you unknown user!)

[![AD7746-block-diagram](https://www.analog.com/-/media/analog/en/products/image/functional-block-diagrams/ad7745-fbl.gif?la=en&h=500&thn=1&hash=C416ECFBC035B58CCBD19FC524BA8F89)](https://www.analog.com/en/products/ad7745.html#product-overview)

# How to use it
Simply clone this project into your application folder. To make things work you must provide the API for the I2C communication (See `i2c.h`):

* `void i2c_init()`
* `void i2c_write(uint8_t slave_address, uint8_t* data_buffer, uint8_t data_size)`
* `void i2c_read(uint8_t slave_address, uint8_t* data_buffer, uint8_t data_size)`

Please note that you must initialize the I2C module before using the AD7745/6 driver!

The AD7745/6 initialization steps should be similar to the following:
* Call `AD7745_Reset()`.
* Call `AD7745_WriteCapSetupRegister(YOUR_DESIRED_CONFIGURATION)`.
* Call `AD7745_WriteExcSetupRegister(YOUR_DESIRED_CONFIGURATION)`.
* Call `AD7745_WriteConfigurationRegister(YOUR_DESIRED_CONFIGURATION)`.
* Call `AD7745_WriteCapDacARegister(CAPDACA_DACAENA_ON|YOUR_CAPDACA_DESIRED_VALUE)` (in case you need it).


## References
- [AD7745/6 Datasheet](http://www.analog.com/media/en/technical-documentation/data-sheets/AD7745_7746.pdf)
- [AN-1585 - Extending the Capacitive Input Range of the AD7745/AD7746 Capacitance-to-Digital Converter](https://www.analog.com/media/en/technical-documentation/application-notes/AN-1585.pdf)
