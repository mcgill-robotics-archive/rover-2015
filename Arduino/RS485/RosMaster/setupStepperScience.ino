#include <SPI.h>    // include the SPI library:
#include "L6470.h"  // include the register and bit definitions

void setupStepperScience()
{
    
  dSPIN_init();
  dSPIN_SetParam(dSPIN_STEP_MODE,
                 !dSPIN_SYNC_EN |
                 dSPIN_STEP_SEL_1_8 |
                 dSPIN_SYNC_SEL_1);

  dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(750));
  dSPIN_SetParam(dSPIN_FS_SPD, 0x3FF);
  dSPIN_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_6000mA);
  dSPIN_SetParam(dSPIN_CONFIG,
                 dSPIN_CONFIG_PWM_DIV_1 |
                 dSPIN_CONFIG_PWM_MUL_2 |
                 dSPIN_CONFIG_SR_180V_us |
                 dSPIN_CONFIG_OC_SD_ENABLE |
                 dSPIN_CONFIG_VS_COMP_DISABLE |
                 dSPIN_CONFIG_SW_HARD_STOP |
                 dSPIN_CONFIG_INT_16MHZ);
  dSPIN_SetParam(dSPIN_KVAL_HOLD, 0x2F);
  dSPIN_SetParam(dSPIN_KVAL_RUN, 0x43);
  dSPIN_SetParam(dSPIN_KVAL_ACC, 0x43);
  dSPIN_SetParam(dSPIN_KVAL_DEC, 0x43);
  dSPIN_SetParam(dSPIN_ST_SLP, 0x1B);
  dSPIN_SetParam(dSPIN_INT_SPD, 0x39E6);
  dSPIN_SetParam(dSPIN_FN_SLP_ACC, 0x2E);
  dSPIN_SetParam(dSPIN_FN_SLP_DEC, 0x2E);
  dSPIN_GetStatus();
}

void setupStepperGrab()
{
    
  dSPIN_init();
  dSPIN_SetParam(dSPIN_STEP_MODE,
                 !dSPIN_SYNC_EN |
                 dSPIN_STEP_SEL_1_8 |
                 dSPIN_SYNC_SEL_1);

  dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(750));
  dSPIN_SetParam(dSPIN_FS_SPD, 0x3FF);
  dSPIN_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_6000mA);
  dSPIN_SetParam(dSPIN_CONFIG,
                 dSPIN_CONFIG_PWM_DIV_1 |
                 dSPIN_CONFIG_PWM_MUL_2 |
                 dSPIN_CONFIG_SR_180V_us |
                 dSPIN_CONFIG_OC_SD_ENABLE |
                 dSPIN_CONFIG_VS_COMP_DISABLE |
                 dSPIN_CONFIG_SW_HARD_STOP |
                 dSPIN_CONFIG_INT_16MHZ);
  dSPIN_SetParam(dSPIN_KVAL_HOLD, 0x1A);
  dSPIN_SetParam(dSPIN_KVAL_RUN, 0x24);
  dSPIN_SetParam(dSPIN_KVAL_ACC, 0x24);
  dSPIN_SetParam(dSPIN_KVAL_DEC, 0x24);
  dSPIN_SetParam(dSPIN_ST_SLP, 0x1B);
  dSPIN_SetParam(dSPIN_INT_SPD, 0x0FA5);
  dSPIN_SetParam(dSPIN_FN_SLP_ACC, 0x41);
  dSPIN_SetParam(dSPIN_FN_SLP_DEC, 0x41);
  dSPIN_GetStatus();
}
