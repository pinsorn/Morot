# Morot

This is a PlatformIO project for motor control using the ESP32-S3 with the Arduino framework. The project includes the following key features and dependencies:

## Features

- Motor control using advanced libraries.
- Support for ESP32-S3 DevKitC-1-N16R8V board.
- Configurable build and upload options.

## Dependencies

- **TMCStepper**: For controlling stepper motors.
- **Streaming**: For efficient data streaming.
- **FastAccelStepper**: For high-performance stepper motor acceleration.
- **Adafruit NeoPixel**: For controlling NeoPixel LEDs.
- **EspSoftwareSerial**: For software-based serial communication.

## Configuration

The project is configured using `platformio.ini` with the following settings:

- Platform: `espressif32`
- Framework: `arduino`
- Upload protocol: `esptool`
- Monitor speed: `115200`

## How to Use

1. Clone this repository.
2. Open the project in PlatformIO.
3. Build and upload the code to your ESP32-S3 board.

---

_This project is designed for hobbyists and developers working on motor control applications._
