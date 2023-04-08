# motor

```
mkdir build
cd build
cmake -DMCU=STM32F4 -DBOARD=board1 ..
make
```

    Motor control implementation: The motor control implementation is divided into the following files:
        src/motor_control.h: Header file containing the function declarations for motor initialization and control.
        src/motor_control.c: Source file implementing the motor initialization and control functions.

    Motor data: The MotorData structure, defined in src/MotorData.h, is used to store motor-specific parameters such as duty cycle, speed, and position.

    Microcontroller (MCU) abstraction: The MCU abstraction layer (HAL) is designed to be MCU-agnostic, with the following files:
        mcu/mcu.h: Contains the declarations for all the MCU functions needed by the motor control system.
        mcu/mcudefs.h: Provides a list of supported MCUs and their specific headers.
        mcu/mcus.h: Contains conditional preprocessor directives to include the correct MCU-specific header based on the chosen MCU.

    Board abstraction: The board abstraction layer (HAL) is designed to be board-agnostic, with the following files located in src/board/:
        hal_board.h: Header files for specific boards, providing the definitions to map the motor control internal named pins to the microcontroller physical pins.
        These files are organized in folders by MCU type, and then by specific boards within those folders.

    Encoder abstraction: The encoder abstraction is designed to be encoder-agnostic, with the following files:
        src/encoders/encoder.h: Common encoder interface header file.
        src/encoders/biss_encoder.c: Example of a specific encoder implementation for the BiSS protocol.
        Additional encoder-specific implementation files can be added to the src/encoders folder as needed.

    Main application: The main application is located in src/main.c. It initializes the motor control system, sets up the control loop, and calls the motor control function to update the motor parameters.

    CMake build system: The CMake build system is used to configure the project for building with various MCUs and boards, with the following files:
        CMakeLists.txt: The main CMake configuration file, which includes the source files, sets the MCU and board options, and specifies the include directories.
        src/board/CMakeLists.txt: CMake configuration file for the board-specific files, which sets the correct include directories based on the chosen MCU and board.

    This motor control system is designed to be modular and easily adaptable to different MCUs, boards, and encoders. The abstraction layers allow for easy integration of new hardware without affecting the core motor control code.
