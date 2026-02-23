#ifndef ROLE_CONFIG_H
#define ROLE_CONFIG_H

// Device Role Configuration
// Change this to select which firmware to build:
//   DEVICE_ROLE_BASE  = 1  (Base Station - sends RTCM corrections)
//   DEVICE_ROLE_ROVER = 2  (Rover - receives RTCM corrections)

#define DEVICE_ROLE_BASE  1
#define DEVICE_ROLE_ROVER 2

// *** SET YOUR DEVICE ROLE HERE ***
#define DEVICE_ROLE DEVICE_ROLE_ROVER

#endif // ROLE_CONFIG_H
