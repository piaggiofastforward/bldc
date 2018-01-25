/**
  External interrupts for the application level UART stuff is contained within this file.
  */

#ifndef EXT_HANDLER_H_
#define EXT_HANDLER_H_

#include "conf_general.h"
#include "hw.h"

typedef void (*EstopHandlerFunc)(void);

/**
  Configure all interrupt sources that are needed for application_uart.c:
  - estop
*/
void configure_EXT();

void set_estop_callback(EstopHandlerFunc f);

void estop_state_change_handler();

#endif