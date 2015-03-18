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

UINT8 curr_see3cam_dev=0;
int hid_fd = -1;

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	InitExtensionUnit								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       finds see3cam device based on the VID, PID and busname and initialize the HID	    *
			device. Application should call this function before calling any other function.    *
  **********************************************************************************************************
*/

BOOL InitExtensionUnit(UINT8 *busname)
{
	int i, ret, desc_size = 0;
	char buf[256];
	struct hidraw_devinfo info;
	struct hidraw_report_descriptor rpt_desc;
	ret = find_hid_device((char*)busname);
	if(ret < 0)
	{
		printf("%s(): Not able to find the e-con's see3cam device\n", __func__);
		return FALSE;
	}
	printf(" Selected HID Device : %s\n",hid_device);

	/* Open the Device with non-blocking reads. In real life,
	   don't use a hard coded path; use libudev instead. */
	hid_fd = open(hid_device, O_RDWR|O_NONBLOCK);

	if (hid_fd < 0) {
		perror("Unable to open device");
		return FALSE;
	}

	memset(&rpt_desc, 0x0, sizeof(rpt_desc));
	memset(&info, 0x0, sizeof(info));
	memset(buf, 0x0, sizeof(buf));

	/* Get Report Descriptor Size */
	ret = ioctl(hid_fd, HIDIOCGRDESCSIZE, &desc_size);
	if (ret < 0) {
		perror("HIDIOCGRDESCSIZE");
		return FALSE;
	}
	else
		printf("Report Descriptor Size: %d\n", desc_size);

	/* Get Report Descriptor */
	rpt_desc.size = desc_size;
	ret = ioctl(hid_fd, HIDIOCGRDESC, &rpt_desc);
	if (ret < 0) {
		perror("HIDIOCGRDESC");
		return FALSE;
	} else {
		printf("Report Descriptors:\n");
		for (i = 0; i < rpt_desc.size; i++)
			printf("%hhx ", rpt_desc.value[i]);
		puts("\n");
	}

	/* Get Raw Name */
	ret = ioctl(hid_fd, HIDIOCGRAWNAME(256), buf);
	if (ret < 0) {
		perror("HIDIOCGRAWNAME");
		return FALSE;
	}
	else
		printf("Raw Name: %s\n", buf);

	/* Get Physical Location */
	ret = ioctl(hid_fd, HIDIOCGRAWPHYS(256), buf);
	if (ret < 0) {
		perror("HIDIOCGRAWPHYS");
		return FALSE;
	}
	else
		printf("Raw Phys: %s\n", buf);

	/* Get Raw Info */
	ret = ioctl(hid_fd, HIDIOCGRAWINFO, &info);
	if (ret < 0) {
		perror("HIDIOCGRAWINFO");
		return FALSE;
	} else {
		printf("Raw Info:\n");
		printf("\tbustype: %d (%s)\n", info.bustype, bus_str(info.bustype));
		printf("\tvendor: 0x%04hx\n", info.vendor);
		printf("\tproduct: 0x%04hx\n", info.product);
	}
	
#if 1
	if(curr_see3cam_dev == SEE3CAM_11CUG || 
		curr_see3cam_dev == SEE3CAM_10CUG ||
		curr_see3cam_dev == SEE3CAM_CU50 ||
		curr_see3cam_dev == SEE3CAM_80USB ) {
			ret=SendOSCode();
		if (ret == FALSE) {
			printf("OS Identification failed\n");
//			return FALSE;
		}
	} else {
		printf("\nOS identification not supported by camera\n");
	}
#endif
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	SendOSCode									    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       Sends the camera command code for Linux OS identification		 	    *
  **********************************************************************************************************
*/
BOOL SendOSCode()
{
	int ret = 0;
	BOOL timeout = TRUE;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number for OS identification
	g_out_packet_buf[1] = OS_CODE; 	/* Report Number OS identification */
	g_out_packet_buf[2] = LINUX_OS;	/* Report Number for Linux OS */

	/* Send a Report to the Device */
	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		printf("\nOS Identification Failed\n");
		return FALSE;
	} else {
		printf("%s(): wrote %d bytes\n", __func__,ret);
		printf("\nSent Linux OS identificaton code\n");
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
			if(g_in_packet_buf[0] == OS_CODE &&
				g_in_packet_buf[1] == LINUX_OS ) {
					if(g_in_packet_buf[2] == SET_SUCCESS)
						printf("\nSet Success\n");
					else if (g_in_packet_buf[2] == SET_FAIL){
						printf("\nSet Failed\n");
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
			printf("\nOS Identification Failed\n");
			timeout = FALSE;
			return FALSE;
		}
	}
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	ReadFirmwareVersion								    *
 *  Parameter1	:	unsigned char *		(Major Version)						    *
 *  Parameter2	:	unsigned char *		(Minor Version 1)					    *
 *  Parameter3	:	unsigned short int * 	(Minor Version 2)					    *
 *  Parameter4	:	unsigned short int * 	(Minor Version 3)					    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command for reading firmware version to the UVC device     *
 *			and then device sends back the firmware version will be stored in the variables	    *	
  **********************************************************************************************************
*/

BOOL ReadFirmwareVersion (UINT8 *pMajorVersion, UINT8 *pMinorVersion1, UINT16 *pMinorVersion2, UINT16 *pMinorVersion3)
{	

	BOOL timeout = TRUE;
	int ret = 0;
	unsigned int start, end = 0;
	unsigned short int sdk_ver=0, svn_ver=0;
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = READFIRMWAREVERSION; 	/* Report Number */

	/* Send a Report to the Device */
	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): wrote %d bytes\n", __func__,ret);
	}
	/* Read the Firmware Version from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == READFIRMWAREVERSION) {
				sdk_ver = (g_in_packet_buf[3]<<8)+g_in_packet_buf[4];
				svn_ver = (g_in_packet_buf[5]<<8)+g_in_packet_buf[6];

				*pMajorVersion = g_in_packet_buf[1];
				*pMinorVersion1 = g_in_packet_buf[2];
				*pMinorVersion2 = sdk_ver;
				*pMinorVersion3 = svn_ver;

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
 *  Name	:	UninitExtensionUnit								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       to release all the extension unit objects and other internal library objects	    *	
  **********************************************************************************************************
*/

BOOL UninitExtensionUnit()
{
	int ret=0;
	/* Close the hid fd */
	if(hid_fd > 0)
	{
		ret=close(hid_fd);
	}
	if(ret<0)
		return FALSE ;
	else
		return TRUE;
	//When threads are used don't forget to terminate them here.
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	bus_str										    *
 *  Parameter1	:	int										    *
 *  Parameter2	:											    *
 *  Returns	:	const char *									    *
 *  Description	:       to convert integer bus type to string					 	    *	
  **********************************************************************************************************
*/
const char *bus_str(int bus)
{
	switch (bus) {
	case BUS_USB:
		return "USB";
		break;
	case BUS_HIL:
		return "HIL";
		break;
	case BUS_BLUETOOTH:
		return "Bluetooth";
		break;
	case BUS_VIRTUAL:
		return "Virtual";
		break;
	default:
		return "Other";
		break;
	}
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	Internal API 									    *
 *  Name	:	GetTicketCount									    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	unsigned int									    *
 *  Description	:       to return current time in the milli seconds				 	    *	
  **********************************************************************************************************
*/

unsigned int GetTickCount()
{
        struct timeval tv;
        if(gettimeofday(&tv, NULL) != 0)
                return 0;

        return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	find_hid_device									    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	int (SUCCESS or FAILURE)							    *
 *  Description	:       to find the first e-con's hid device connected to the linux pc		 	    *	
  **********************************************************************************************************
*/


int find_hid_device(char *videobusname)
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev, *pdev;
	int ret = FAILURE;
	char buf[256];
	
   	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		exit(1);
	}

	/* Create a list of the devices in the 'hidraw' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "hidraw");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	
	/* For each item enumerated, print out its information. udev_list_entry_foreach is a macro which expands to a loop. The loop will be executed for each member in
	   devices, setting dev_list_entry to a list entry which contains the device's path in /sys. */
	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *path;
		
		/* Get the filename of the /sys entry for the device and create a udev_device object (dev) representing it */
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);

		/* usb_device_get_devnode() returns the path to the device node itself in /dev. */
		//printf("Device Node Path: %s\n", udev_device_get_devnode(dev));
		
		/* The device pointed to by dev contains information about the hidraw device. In order to get information about the USB device, get the parent device with the subsystem/devtype pair of "usb"/"usb_device". This will be several levels up the tree, but the function will find it.*/
		pdev = udev_device_get_parent_with_subsystem_devtype(
		       dev,
		       "usb",
		       "usb_device");
		if (!pdev) {
			printf("Unable to find parent usb device.");
			exit(1);
		}
	
		/* From here, we can call get_sysattr_value() for each file in the device's /sys entry. The strings passed into these functions (idProduct, idVendor, serial, 			etc.) correspond directly to the files in the /sys directory which represents the USB device. Note that USB strings are Unicode, UCS2 encoded, but the strings    		returned from udev_device_get_sysattr_value() are UTF-8 encoded. */
		/*printf("  VID/PID: %s %s\n",
		        udev_device_get_sysattr_value(pdev,"idVendor"),
		        udev_device_get_sysattr_value(pdev, "idProduct"));
		printf("  %s\n  %s\n",
		        udev_device_get_sysattr_value(pdev,"manufacturer"),
		        udev_device_get_sysattr_value(pdev,"product"));
		printf("  serial: %s\n",
		         udev_device_get_sysattr_value(pdev, "serial"));*/

		if(!strncmp(udev_device_get_sysattr_value(pdev,"idVendor"), "2560", 4)) {
			switch(curr_see3cam_dev) {
			case SEE3CAM_10CUG :
				if(!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c110", 4) || (!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c111", 4)) ) { 
					hid_device = udev_device_get_devnode(dev);
					udev_device_unref(pdev);
				}
					break;			
			case SEE3CAM_11CUG :
				if(!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c112", 4)) {
					hid_device = udev_device_get_devnode(dev);
					udev_device_unref(pdev);
				}
					break;			
			case SEE3CAM_80USB :
				if(!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c080", 4)) {
					hid_device = udev_device_get_devnode(dev);
					udev_device_unref(pdev);
				}
					break;			
			case SEE3CAM_30_Z10X :
				if(!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c034", 4)) {
					hid_device = udev_device_get_devnode(dev);
					udev_device_unref(pdev);
				}
					break;			
			case SEE3CAM_CU50 :
				if(!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c151", 4)) {
					hid_device = udev_device_get_devnode(dev);
					udev_device_unref(pdev);
				}
					break;			
			}
		}
		//Open each hid device and Check for bus name here
		hid_fd = open(hid_device, O_RDWR|O_NONBLOCK);

		if (hid_fd < 0) {
			perror("Unable to open device");
			continue;
		}else
			memset(buf, 0x00, sizeof(buf));

		/* Get Physical Location */
		ret = ioctl(hid_fd, HIDIOCGRAWPHYS(256), buf);
		if (ret < 0) {
			perror("HIDIOCGRAWPHYS");
		}
		//check if bus names are same or else close the hid device
		if(!strncmp(videobusname,buf,strlen(videobusname))){
			ret = SUCCESS;
		}
		/* Close the hid fd */
		if(hid_fd > 0)
		{
			if(close(hid_fd) < 0) {
				printf("\nFailed to close %s\n",hid_device);
			}
		}
		if(ret == SUCCESS) {
			return TRUE;
		}
	}
/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);

	return ret;
}

