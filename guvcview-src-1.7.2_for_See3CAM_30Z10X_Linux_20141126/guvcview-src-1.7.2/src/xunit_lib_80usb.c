#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>      
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/input.h>
#include <linux/hidraw.h>

#include "xunit_lib.h"
extern UINT8 curr_see3cam_dev;

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	GetFocusMode									    *
 *  Parameter1	:	UINT8 * (FocusMode)								    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      	Reads the focus mode from the device						    *	
  **********************************************************************************************************
*/

BOOL GetFocusMode(UINT8 *FocusMode)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
	g_out_packet_buf[2] = GET_FOCUS_MODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	/* Read the Status code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == CAMERA_CONTROL_80)&&
				(g_in_packet_buf[1]==GET_FOCUS_MODE)) {
					*FocusMode = (g_in_packet_buf[2]);
					timeout=FALSE;
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT){
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;	

}
/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	GetFocusStatus									    *
 *  Parameter1	:	UINT8 * (FocusStatus)								    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      	Reads the focus status code from the device 					    *
  **********************************************************************************************************
*/

BOOL GetFocusStatus(UINT8 *FocusStatus)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
	g_out_packet_buf[2] = GET_FOCUS_STATUS; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	/* Read the Status code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == CAMERA_CONTROL_80)&&
				(g_in_packet_buf[1]==GET_FOCUS_STATUS)) {
				*FocusStatus = (g_in_packet_buf[2]);
				timeout=FALSE;
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;	
}
/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	SetFocusMode									    *
 *  Parameter1	:	UINT8 (focus mode)								    *
 *  Parameter2	:	UINT8 * (focus status)								    *
 *  Parameter3	:											    *
 *  Parameter4	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      	Set focus mode of device 							    *	
  **********************************************************************************************************
*/

BOOL SetFocusMode(UINT8 FocusMode,UINT8 *FocusStatus)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
	g_out_packet_buf[2] = SET_FOCUS_MODE; /* Report Number */
	g_out_packet_buf[3] = FocusMode; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}

	/* Read the Status code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == CAMERA_CONTROL_80)&&
				(g_in_packet_buf[1]==SET_FOCUS_MODE)&&
				(g_in_packet_buf[2]==FocusMode)) {
				*FocusStatus = (g_in_packet_buf[3]);
				timeout=FALSE;
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	GetFocusPosition								    *
 *  Parameter1	:	UINT16 * (focus position)							    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       Reads the focus position from the device					    *	
  **********************************************************************************************************
*/

BOOL GetFocusPosition(UINT16 *FocusPosition)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;
	unsigned short int MSB = 0, LSB = 0;
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
	g_out_packet_buf[2] = GET_FOCUS_POSITION; /* Report Number */
	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	/* Read the position code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == CAMERA_CONTROL_80)&&
				(g_in_packet_buf[1]==GET_FOCUS_POSITION)) {
				MSB = (g_in_packet_buf[2]);
				LSB = (g_in_packet_buf[3]);
				
				*FocusPosition=LSB;
				*FocusPosition|=(MSB<<8);

				timeout = FALSE;
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}		
	}
	return TRUE;
}
/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	SetFocusPosition								    *
 *  Parameter1	:	UINT16 (focus position)								    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      Sends the extension unit command to set the focus position of the camera		    *	
  **********************************************************************************************************
*/

BOOL SetFocusPosition(UINT16 FocusPosition)
{
	int ret =0;
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
	g_out_packet_buf[2] = SET_FOCUS_POSITION; /* Report Number */
	g_out_packet_buf[3] = ((FocusPosition&0xFF00)>>8); /* MSB of focus postion */
	g_out_packet_buf[4] = (FocusPosition&0x00FF); /* LSB of focus postion */
	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	return TRUE;
}


/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	GetGpioLevel									    *
 *  Parameter1	:	UINT8 (GPIO pin number)								    *
 *  Parameter2	:	UINT8 * (GPIO value)								    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       This function sends the extension unit command to Get the Level of the Gpio pin.    *
  **********************************************************************************************************
*/

BOOL GetGpioLevel (UINT8 GpioPin, UINT8 *GpioValue)
{	

	BOOL timeout = TRUE;
	int ret = 0;
	unsigned int start, end = 0;
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = GPIO_OPERATION; 	/* Report Number */
	g_out_packet_buf[2] = GPIO_GET_LEVEL; 	/* Report Number */
	g_out_packet_buf[3] = GpioPin; 		/* GPIO Pin Number */
	/* Send a Report to the Device */
	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): wrote %d bytes\n", __func__,ret);
	}
	/* Read the GPIO level and status of read from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);

		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == GPIO_OPERATION &&
				g_in_packet_buf[1] == GPIO_GET_LEVEL &&
				g_in_packet_buf[2] == GpioPin) {
					*GpioValue = g_in_packet_buf[3];
					if(g_in_packet_buf[4] == GPIO_LEVEL_FAIL) {
						return FALSE;
					} else if(g_in_packet_buf[4]==GPIO_LEVEL_SUCCESS) {
						timeout = FALSE;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}		
	}
	return TRUE;
}


/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	SetGpioLevel									    *
 *  Parameter1	:	UINT8 (GPIO pin number)								    *
 *  Parameter2	:	UINT8 (GPIO value)								    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       This function sends the extension unit command to Set the Level of the Gpio pin.    *
  **********************************************************************************************************
*/

BOOL SetGpioLevel (UINT8 GpioPin, UINT8 GpioValue)
{	

	BOOL timeout = TRUE;
	int ret = 0;
	unsigned int start, end = 0;
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = GPIO_OPERATION; 	/* Report Number */
	g_out_packet_buf[2] = GPIO_SET_LEVEL; 	/* Report Number */
	g_out_packet_buf[3] = GpioPin; 		/* GPIO Pin Number */
	g_out_packet_buf[4] = GpioValue; 	/* GPIO Value */

	/* Send a Report to the Device */
	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): wrote %d bytes\n", __func__,ret);
	}
	/* Read the GPIO level and status of read from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == GPIO_OPERATION &&
				g_in_packet_buf[1] == GPIO_SET_LEVEL &&
				g_in_packet_buf[2] == GpioPin &&
				g_in_packet_buf[3] == GpioValue) {
					if(g_in_packet_buf[4] == GPIO_LEVEL_FAIL) {
						return FALSE;
					} else if(g_in_packet_buf[4]==GPIO_LEVEL_SUCCESS) {
						timeout = FALSE;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}		
	}
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	GetFlashLevel									    *
 *  Parameter1	:	UINT8 *	(Flash Level)								    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      	Reads the flash level from the device 						    *	
  **********************************************************************************************************
*/

BOOL GetFlashLevel(UINT8 *Level)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
if(SEE3CAM_80USB ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
else if(SEE3CAM_CU50 ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
	g_out_packet_buf[2] = GET_FLASH_LEVEL; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	/* Read the Status code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
				(g_in_packet_buf[1]==GET_FLASH_LEVEL)) {
				*Level = (g_in_packet_buf[2]);
				timeout=FALSE;
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;	
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	SetFlashLevel									    *
 *  Parameter1	:	UINT8 (Flash Level)								    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      	Sets the flash level to the device 						    *	
  **********************************************************************************************************
*/

BOOL SetFlashLevel(UINT8 Level)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
if(SEE3CAM_80USB ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
else if(SEE3CAM_CU50 ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
	g_out_packet_buf[2] = SET_FLASH_LEVEL; 	/* Report Number */
	g_out_packet_buf[3] = Level;		/* Flash mode */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	/* Read the Status code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
				(g_in_packet_buf[1]==SET_FLASH_LEVEL) &&
				(g_in_packet_buf[2]==Level )) {
					if(g_in_packet_buf[3] == SET_FAIL) {
						return FALSE;
					} else if(g_in_packet_buf[3]==SET_SUCCESS) {
						timeout = FALSE;
					}
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	GetTorchLevel									    *
 *  Parameter1	:	UINT8 *	(Flash Level)								    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      	Reads the flash level from the device 						    *	
  **********************************************************************************************************
*/

BOOL GetTorchLevel(UINT8 *Level)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
if(SEE3CAM_80USB ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
else if(SEE3CAM_CU50 ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
	g_out_packet_buf[2] = GET_TORCH_LEVEL; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	/* Read the Status code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
				(g_in_packet_buf[1]==GET_TORCH_LEVEL)) {
				*Level = (g_in_packet_buf[2]);
				timeout=FALSE;
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;	
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	SetTorchLevel									    *
 *  Parameter1	:	UINT8 (Flash Level)								    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:      	Sets the flash level to the device 						    *	
  **********************************************************************************************************
*/

BOOL SetTorchLevel(UINT8 Level)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
if(SEE3CAM_80USB ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
else if(SEE3CAM_CU50 ==curr_see3cam_dev)
	g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
	g_out_packet_buf[2] = SET_TORCH_LEVEL; 	/* Report Number */
	g_out_packet_buf[3] = Level;		/* Flash mode */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);

	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	/* Read the Status code from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
				(g_in_packet_buf[1]==SET_TORCH_LEVEL) &&
				(g_in_packet_buf[2]==Level )) {
					if(g_in_packet_buf[3] == SET_FAIL) {
						return FALSE;
					} else if(g_in_packet_buf[3]==SET_SUCCESS) {
						timeout = FALSE;
					}
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;
}

BOOL SendCaptureComplete()
{
	int ret = 0;
	BOOL timeout = TRUE;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number for OS identification
	g_out_packet_buf[1] = CAPTURE_COMPLETE; 	/* Report Number for capture complete acknowledgement */

	/* Send a Report to the Device */
	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): wrote %d bytes\n", __func__,ret);
	}
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == CAPTURE_COMPLETE) {
					if(g_in_packet_buf[1] == 0x01)
						printf("\nAck Success\n");
					else if (g_in_packet_buf[1] == 0x00){
						printf("\nAck Failed\n");
						return FALSE;
					}
					else {
						printf("\nUnknown return value\n");
						return FALSE;
					}
					timeout = FALSE;
			}
		}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			printf("\ncapture ack Failed\n");
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;
}
