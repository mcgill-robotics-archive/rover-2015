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

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	EnableTriggerMode								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera master mode			    *	
  **********************************************************************************************************
*/

BOOL EnableMasterMode()
{
	int ret =0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLEMASTERMODE; /* Report Number */

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
 *  Name	:	EnableTriggerMode								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera trigger mode			    *	
  **********************************************************************************************************
*/

BOOL EnableTriggerMode()
{
	int ret =0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLETRIGGERMODE; /* Report Number */

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
 *  Name	:	EnableCroppedVGAMode								    *
 *  Parameter1	:	UINT8 * ( Cropped VGA status )							    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera cropped VGA mode		    *	
  **********************************************************************************************************
*/

BOOL EnableCroppedVGAMode(UINT8 *VGAStatus)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLE_CROPPED_VGA_MODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
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
			if(g_in_packet_buf[0] == ENABLE_CROPPED_VGA_MODE) {
				*VGAStatus = g_in_packet_buf[1];
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
 *  Name	:	EnableBinnedVGAMode								    *
 *  Parameter1	:	UINT8 * ( Binned VGA status )							    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera binned VGA mode		    *	
  **********************************************************************************************************
*/

BOOL EnableBinnedVGAMode(UINT8 *VGAStatus)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLE_BINNED_VGA_MODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
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
			if(g_in_packet_buf[0] == ENABLE_BINNED_VGA_MODE) {
				*VGAStatus = g_in_packet_buf[1];
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
 *  Name	:	GetWhiteBalanceMode								    *
 *  Parameter1	:	UINT8 * ( WhiteBalanceMode )							    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to get the white balance mode of the camera	    *	
  **********************************************************************************************************
*/

BOOL GetWhiteBalanceMode(UINT8 *WhiteBalanceMode)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHITE_BAL_CONTROL; /* Report Number */
	g_out_packet_buf[2] = GET_WB_MODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
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
			if(g_in_packet_buf[0] == WHITE_BAL_CONTROL  &&
				g_in_packet_buf[1] == GET_WB_MODE ) {
					if(g_in_packet_buf[3] == WB_FAIL)
						return FALSE;
					else if (g_in_packet_buf[3] == WB_SUCCESS) {
						*WhiteBalanceMode = g_in_packet_buf[2];
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
 *  Name	:	SetWhiteBalanceMode								    *
 *  Parameter1	:	UINT8 ( WhiteBalanceMode )							    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to set the specified white balance mode in the	    *
			camera										    *	
  **********************************************************************************************************
*/

BOOL SetWhiteBalanceMode(UINT8 WhiteBalanceMode)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHITE_BAL_CONTROL; /* Report Number */
	g_out_packet_buf[2] = SET_WB_MODE; /* Report Number */
	g_out_packet_buf[3] = WhiteBalanceMode; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
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
			if(g_in_packet_buf[0] == WHITE_BAL_CONTROL  &&
				g_in_packet_buf[1] == SET_WB_MODE &&
				g_in_packet_buf[2] == WhiteBalanceMode ) {
					if(g_in_packet_buf[3] == WB_FAIL)
						return FALSE;
					else if (g_in_packet_buf[3] == WB_SUCCESS) {
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
 *  Name	:	GetWhiteBalanceGain								    *
 *  Parameter1	:	UINT8 ( WhiteBalanceColor )							    *
 *  Parameter2	:	UINT8 * ( WhiteBalanceGain )							    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to get the white balance gain of a color from the  *
			camera										    *	
  **********************************************************************************************************
*/

BOOL GetWhiteBalanceGain(UINT8 WhiteBalanceColor,UINT8 *WhiteBalanceGain)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHITE_BAL_CONTROL; /* Report Number */
	g_out_packet_buf[2] = GET_WB_GAIN; /* Report Number */
	g_out_packet_buf[3] = WhiteBalanceColor; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
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
			if(g_in_packet_buf[0] == WHITE_BAL_CONTROL  &&
				g_in_packet_buf[1] == GET_WB_GAIN && 
				g_in_packet_buf[2] == WhiteBalanceColor ) {
					if(g_in_packet_buf[4] == WB_FAIL)
						return FALSE;
					else if (g_in_packet_buf[4] == WB_SUCCESS) {
						*WhiteBalanceGain = g_in_packet_buf[3];
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
 *  Name	:	SetWhiteBalanceGain								    *
 *  Parameter1	:	UINT8 ( WhiteBalanceColor )							    *
 *  Parameter2	:	UINT8 ( WhiteBalanceGain )							    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to set the white balance gain of a color from the  *
			camera										    *	
  **********************************************************************************************************
*/

BOOL SetWhiteBalanceGain(UINT8 WhiteBalanceColor,UINT8 WhiteBalanceGain)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHITE_BAL_CONTROL; /* Report Number */
	g_out_packet_buf[2] = SET_WB_GAIN; /* Report Number */
	g_out_packet_buf[3] = WhiteBalanceColor; /* Report Number */
	g_out_packet_buf[4] = WhiteBalanceGain; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
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
			if(g_in_packet_buf[0] == WHITE_BAL_CONTROL  &&
				g_in_packet_buf[1] == SET_WB_GAIN && 
				g_in_packet_buf[2] == WhiteBalanceColor &&
				g_in_packet_buf[3] == WhiteBalanceGain ) {
					if(g_in_packet_buf[4] == WB_FAIL)
						return FALSE;
					else if (g_in_packet_buf[4] == WB_SUCCESS) {
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
 *  Name	:	DefaultWhiteBalance								    *
 *  Parameter1	:	UINT8 * ( WhiteBalanceMode )							    *
 *  Parameter2	:	UINT8 * ( WhiteBalanceRedGain)							    *
 *  Parameter3	:	UINT8 * ( WhiteBalanceGreenGain)						    *
 *  Parameter4	:	UINT8 * ( WhiteBalanceBlueGain)							    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to set the default white balance mode in the	    *
			camera										    *	
  **********************************************************************************************************
*/

BOOL DefaultWhiteBalance(UINT8 *WhiteBalanceMode, UINT8 *WhiteBalanceRedGain,UINT8 *WhiteBalanceGreenGain,UINT8 *WhiteBalanceBlueGain)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHITE_BAL_CONTROL; /* Report Number */
	g_out_packet_buf[2] = SET_WB_DEFAULTS; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
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
			if(g_in_packet_buf[0] == WHITE_BAL_CONTROL  &&
				g_in_packet_buf[1] == SET_WB_DEFAULTS ) {
					if(g_in_packet_buf[6] == WB_FAIL)
						return FALSE;
					else if (g_in_packet_buf[6] == WB_SUCCESS) {
						*WhiteBalanceMode = g_in_packet_buf[2];
						*WhiteBalanceRedGain = g_in_packet_buf[3];
						*WhiteBalanceGreenGain = g_in_packet_buf[4];
						*WhiteBalanceBlueGain = g_in_packet_buf[5];
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

