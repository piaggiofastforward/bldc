This is the source code for the VESC DC/BLDC/FOC controller. Read more at  
http://vesc-project.com/

## PFF-BLDC ##

Our vesc interface is done over `UART`, and all communication functionality can be found in `applications/app_uartcomm.c`. Here are some characteristics:

- Feedback publishing at 50Hz (configurable via `FB_RATE_MS`)
- Status publishing at 20Hz (configurable via `STATUS_RATE_MS`)
- Command speed is dictated by the Linux/embedded driver, but has been verified to function properly up to ~200Hz.

Support for linear motor is in the code, but untested over `UART`.
