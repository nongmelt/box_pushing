# UoB: Robotics Systems Assessment 2 (Box Pushing Problem)

## Introduction

This project uses Platform.io tools for programming. 

## Project Structure

The project have the following structure:

```
my_project/
├── .pio/
├── include/
├── lib/
├── src/
│   └── main.cpp
├── test/
└── platformio.ini
```

- `src/`: This is where your main code goes.
- `lib/`: For project-specific libraries.
- `include/`: For project-specific header files.
- `platformio.ini`: The configuration file for your project.

You may want to change `upload_port` when building and uploading your code inside `platformio.ini` 

This project depends on 
1. `Arduino` for Arduino functions
2. `pololu/PololuOLED@^2.0.0` for OLED display
3. `pololu/LIS3MDL@^2.0.0` for magnetometer
4. `pololu/LSM6@^2.0.1` for IMU

## Entry point

`src/main.cpp`.


## Building and Uploading

1. Build the project:
```sh
pio run
```

2. Upload the code to your board:
```sh
pio run --target upload
```