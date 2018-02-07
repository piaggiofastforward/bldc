#include "ext_handler.h"
#include "app.h"

/**
  Somebody else can use this to be notified when the estop state changes
  Currently, that "somebody" is app_uartcomm.c
*/
static EstopHandlerFunc estop_handler = NULL;

void configure_EXT(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  // connect EXTI line to pin
  SYSCFG_EXTILineConfig(HW_ESTOP_EXTI_PORTSRC, HW_ESTOP_EXTI_PINSRC);

  // configure the EXTI line
  EXTI_InitStructure.EXTI_Line = HW_ESTOP_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // enable interrupt, set status to highest priority
  nvicEnableVector(HW_ESTOP_EXTI_CH, 0);
}

void set_estop_callback(EstopHandlerFunc f)
{
  estop_handler = f;
}

void estop_state_change_handler(void)
{
  if (estop_handler != NULL)
    estop_handler();
}