# mavlink_serial
Related to microcontroller and microprocessor implementation.

## Installation and Setup

1. Clone the repository:
```bash
git clone https://github.com/jazzazi007/mavlink_serial.git
cd mavlink_serial
```

2. Install required dependencies:
```bash
sudo apt-get update
sudo apt-get install -y build-essential libsdl2-dev libsdl2-ttf-dev
```

3. Build the project:
```bash
make clean
make
```

4. Run the program:
```bash
./rpi_gnc
```

## Requirements
- GCC compiler
- SDL2 libraries
- SDL2_ttf libraries
- MAVLink protocol

## Note
Make sure you have proper permissions to access serial ports if using them for communication with autopilot hardware.