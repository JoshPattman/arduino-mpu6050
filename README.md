# arduino-mpu6050
This sketch converts your arduino into a roll and pitch sensor over a serial connection. It is based on the i2cdev example (by Jeff Rowberg).
## Hardware
* Arduino (I have only tested this on nanos, but it should work on any arduino)
* MPU6050 IMU board (for example [this](https://store.flytron.com/products/mpu6050-6dof-imu-module?variant=32071515930755&currency=GBP&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&utm_campaign=gs-2020-09-07&utm_source=google&utm_medium=smart_campaign)). These pins should be connected:
  * MPU `VCC` and arduino `5v`
  * MPU `GND` and arduino `Gnd`
  * MPU `SDA` and arduino SDA pin (on nanos this is `A4`)
  * MPU `SCL` and arduino SCL pin (on nanos this is `A5`)
  * MPU `INT` and arduino external interrupt #0 pin (on nanos this is pin `2`)
> I have found the code to work best when the MPU6050 has the side with the soldered chips facing upwards
## Libraries
You need the following arduino libraries for the code to work:
* `I2Cdev`
* `MPU6050`

For instructions of how to install these, go to the [I2cdev github](https://github.com/jrowberg/i2cdevlib)
## For people using this for `spotpuppy` python library
Use the `main` branch
## For people using this for `spotpuppy-go` golang package (or most other use cases)
Use the `simple` branch. It has a cleaner protocol and runs more reliably.
