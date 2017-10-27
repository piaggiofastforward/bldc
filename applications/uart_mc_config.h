#ifndef UART_MC_CONFIG_
#define UART_MC_CONFIG_

#include "control_msgs.h"
#include "datatypes.h"

/*
 * Sets the parameter to the provided value in the local configuration
 *
 * Does not update the configuration externally
 */
void setParameter(mc_config config, mc_configuration *mcconfig);

#endif
