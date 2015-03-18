#ifndef XUNIT_TAB_LIB_H
#define XUNIT_TAB_LIB_H

#include <stdbool.h>
#include <defs.h>


/* VID and PID of SEE3CAM Products */

#define SEE3CAM_USB_VID		0x2560
#define SEE3CAM_10CUG_M_PID	0xc110
#define SEE3CAM_10CUG_C_PID	0xc111
#define SEE3CAM_80USB_PID	0xc080
#define SEE3CAM_11CUG_C_PID	0xc112
#define SEE3CAM_30_Z10X_PID	0xc034
#define SEE3CAM_CU50_PID	0xc151

/* GPIO pins of SEE3CAM_80USB Products */

#define GPIO_80_OUT	0x14
#define GPIO_80_IN1	0x13
#define GPIO_80_IN2	0x21

/* GPIO pins of SEE3CAM_80USB Products */

#define GPIO_50_OUT1	0x18
#define GPIO_50_OUT2	0x14
#define GPIO_50_IN1	0x13
#define GPIO_50_IN2	0x16

typedef bool BOOL;

BOOL g_flash_flag;
BOOL g_external_trigger_flag;

enum see3cam_device_index
{
	SEE3CAM_10CUG = 1,
	SEE3CAM_11CUG,
	SEE3CAM_80USB,
	SEE3CAM_30_Z10X,
	SEE3CAM_CU50,
};

/* Function Declarations */

BOOL InitExtensionUnit(UINT8 *);

BOOL UninitExtensionUnit();

BOOL SendOSCode();

BOOL SendCaptureComplete();

BOOL ReadFirmwareVersion (UINT8 *, UINT8 *, UINT16 *, UINT16 *);

BOOL EnableMasterMode();

BOOL EnableTriggerMode();

BOOL SetFocusPosition( UINT16 );

BOOL GetFocusPosition( UINT16 *);

BOOL SetFocusMode( UINT8 , UINT8 *);

BOOL GetFocusStatus();

BOOL GetFocusMode( UINT8 *);

BOOL GetGpioLevel(UINT8 , UINT8 *);

BOOL SetGpioLevel (UINT8 , UINT8 );

BOOL GetFlashLevel (UINT8 *);

BOOL SetFlashLevel (UINT8);

BOOL GetTorchLevel (UINT8 *);

BOOL SetTorchLevel (UINT8);

BOOL EnableCroppedVGAMode(UINT8 *);

BOOL EnableBinnedVGAMode(UINT8 *);

BOOL GetWhiteBalanceMode(UINT8 *);

BOOL SetWhiteBalanceMode(UINT8);

BOOL GetWhiteBalanceGain(UINT8 , UINT8 *);

BOOL SetWhiteBalanceGain(UINT8 , UINT8);

BOOL SetAllWhiteBalanceGains(UINT8 , UINT8, UINT8);

BOOL DefaultWhiteBalance(UINT8 *, UINT8 *,UINT8 *,UINT8 *);

BOOL ReadCameraFirmwareVersion (char *);

BOOL SendCameraCommand (BYTE , UINT8 , UINT8 , UINT8 *);

BOOL GetLEDState( UINT8 *, UINT8 *);

BOOL SetLEDState( UINT8 , UINT8);

BOOL RestoreFactorySettings (UINT8 *);

#endif
