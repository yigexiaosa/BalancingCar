cmake_minimum_required(VERSION 3.23)
project(BalancingCar C)

set(CMAKE_C_STANDARD 11)

include_directories(User)

include_directories(Library
        Start
        System
        Hardware
        User
        )


add_executable(main User/main.c)
add_executable(Key Hardware/Key.c)
add_executable(Motor Hardware/Motor.c)
add_executable(Timer System/Timer.c)
add_executable(KalmanFilter Hardware/KalmanFilter.c)
add_executable(MPU6050 Hardware/MPU6050.c)
add_executable(EncoderSpeed Hardware/EncoderSpeed.c)
add_executable(PID Hardware/PID.c)
add_executable(TimerCount System/TimerCount.c)
add_executable(HC_RS04 Hardware/HC_RS04.c)
add_executable(Serial Hardware/Serial.c)
add_executable(Bluetooth Hardware/Bluetooth.c)