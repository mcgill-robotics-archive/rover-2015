/*******************************************************************************#
#           guvcview              http://guvcview.sourceforge.net               #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#           Nobuhiro Iwamatsu <iwamatsu@nigauri.org>                            #
#                                                                               #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/

#include <SDL/SDL.h>
#include <glib.h>
#include <glib/gprintf.h>
/* support for internationalization - i18n */
#include <glib/gi18n.h>
#include <gtk/gtk.h>
#include "globals.h"
#include "callbacks.h"
#include "v4l2uvc.h"
#include "string_utils.h"
#include "vcodecs.h"

#include "xunit_tab_lib.h"
#include "xunit_lib.h"
#include "xunit_tab.h"

extern UINT8 curr_see3cam_dev;

void ShowErrorDialog(const char *err_msg, struct ALL_DATA *all_data)
{
	GtkWidget *error_dialog = gtk_message_dialog_new ( NULL,
			GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
			GTK_MESSAGE_ERROR,
			GTK_BUTTONS_CLOSE,
			NULL);

	gtk_message_dialog_format_secondary_text(GTK_MESSAGE_DIALOG(error_dialog),
		"%s",gettext(err_msg));

	gtk_window_set_title(GTK_WINDOW (error_dialog), "ERROR");
	
	gtk_widget_show(error_dialog);
	gtk_dialog_run (GTK_DIALOG (error_dialog));
	gtk_widget_destroy(error_dialog);
	return;
}

void ShowFwVersion(GtkButton *FwVersionButt, struct ALL_DATA *all_data)
{
	GtkWidget *FW_msg_dialog;
	UINT8 MajorVersion = 0, MinorVersion1= 0;
	UINT16 MinorVersion2= 0, MinorVersion3= 0;
	char CameraFirmwareVersion[BUFFER_LENGTH] ;
	BOOL ret = FALSE;

	ret = ReadFirmwareVersion (&MajorVersion, &MinorVersion1, &MinorVersion2, &MinorVersion3);
	if(ret == FALSE)
	{
		ShowErrorDialog("Error in reading the Firmware Version",all_data);
		return;
	}
	if(SEE3CAM_30_Z10X == curr_see3cam_dev) {
		ret = ReadCameraFirmwareVersion (CameraFirmwareVersion);
		if(ret == FALSE)
		{
			ShowErrorDialog("Error in reading the Camera Firmware Version",all_data);
			return;
		}
		printf("\n\nCamera Firmware Version : %s\n\n", CameraFirmwareVersion );
		FW_msg_dialog = gtk_message_dialog_new (NULL, 
			GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT, 
			GTK_MESSAGE_INFO, 
			GTK_BUTTONS_OK,
			"USB Firmware Version is %d.%d.%d.%d\n\nCamera Firmware Version is %s",MajorVersion, MinorVersion1, MinorVersion2, MinorVersion3, CameraFirmwareVersion);
	}else {
		FW_msg_dialog = gtk_message_dialog_new (NULL, 
			GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT, 
			GTK_MESSAGE_INFO, 
			GTK_BUTTONS_OK,
			"\nFirmware Version is %d.%d.%d.%d",MajorVersion, MinorVersion1, MinorVersion2, MinorVersion3);
	}

	gtk_window_set_title(GTK_WINDOW (FW_msg_dialog),"See3Cam Firmware Version");

	gtk_widget_show_all(FW_msg_dialog);
	gtk_dialog_run (GTK_DIALOG (FW_msg_dialog));
	gtk_widget_destroy(FW_msg_dialog);

	return;
}

void ShowAbout(GtkButton *AboutButt, struct ALL_DATA *all_data)
{
	gchar* econLogoPath = g_strconcat (PACKAGE_DATA_DIR,"/pixmaps/guvcview/econ_logo.png",NULL);
	//don't test for file - use default empty image if load fails
	//get icon image
	GdkPixbuf *econLogo = gdk_pixbuf_new_from_file(econLogoPath,NULL);
	g_free(econLogoPath);

	GtkWidget *AboutDialog = gtk_about_dialog_new();
	gtk_about_dialog_set_program_name(GTK_ABOUT_DIALOG (AboutDialog), "See3CAM GUVCViewer");
	gtk_about_dialog_set_version(GTK_ABOUT_DIALOG (AboutDialog), "1.7.2-s2.23");
	gtk_about_dialog_set_copyright(GTK_ABOUT_DIALOG (AboutDialog), "Copyright © 2014 e-con Systems");
	gtk_about_dialog_set_comments(GTK_ABOUT_DIALOG (AboutDialog),
		"This program see3camguvcview is a free software distributed under GNU GPL. With full credits to the original contributor (guvcview.sourceforge.net) this program is a custom version to support e-con's See3CAM USB Cameras and its XU controls.");
	gtk_about_dialog_set_website(GTK_ABOUT_DIALOG (AboutDialog),
		"http://www.e-consystems.com");
	gtk_about_dialog_set_website_label(GTK_ABOUT_DIALOG (AboutDialog),
		"e-consystems");
	gtk_about_dialog_set_logo(GTK_ABOUT_DIALOG (AboutDialog), econLogo);	

	gtk_widget_show(AboutDialog);
	gtk_dialog_run (GTK_DIALOG (AboutDialog));
	gtk_widget_destroy(AboutDialog);
	return;	
}
void xunit_tab(struct ALL_DATA *all_data)
{
	struct vdIn *uvc_videoIn = all_data->videoIn;
	struct GWIDGET *gwidget = all_data->gwidget;
	GtkWidget *scroll4;
	GtkWidget *Tab4;
	GtkWidget *Tab4Label;
	GtkWidget *Tab4Icon;
	GtkWidget *table4;

	int i;
	BOOL ret = FALSE;
	g_flash_flag=FALSE;
	int valid_econ_device = 0;

	//TABLE
	table4 = gtk_grid_new();
	gtk_grid_set_column_homogeneous (GTK_GRID(table4), FALSE);
	gtk_widget_set_hexpand (table4, TRUE);
	gtk_widget_set_halign (table4, GTK_ALIGN_FILL);
	
	gtk_grid_set_row_spacing (GTK_GRID(table4), 4);
	gtk_grid_set_column_spacing (GTK_GRID (table4), 4);
	gtk_container_set_border_width (GTK_CONTAINER (table4), 2);
	gtk_widget_show (table4);
	//SCROLL
	scroll4 = gtk_scrolled_window_new(NULL,NULL);
    gtk_scrolled_window_set_placement(GTK_SCROLLED_WINDOW(scroll4), GTK_CORNER_TOP_LEFT);

	//ADD TABLE TO SCROLL

    //viewport is only needed for gtk < 3.8
    //for 3.8 and above s->table can be directly added to scroll1
    GtkWidget* viewport = gtk_viewport_new(NULL,NULL);
    gtk_container_add(GTK_CONTAINER(viewport),table4);
    gtk_widget_show(viewport);

    gtk_container_add(GTK_CONTAINER(scroll4), viewport);
	gtk_widget_show(scroll4);
	
	//new hbox for tab label and icon
	Tab4 = gtk_grid_new();
	Tab4Label = gtk_label_new(_("XU Controls"));
	gtk_widget_show (Tab4Label);
	
	gchar* Tab4IconPath = g_strconcat (PACKAGE_DATA_DIR,"/pixmaps/guvcview/see3cam.png",NULL);
	//don't test for file - use default empty image if load fails
	//get icon image
	Tab4Icon = gtk_image_new_from_file(Tab4IconPath);
	g_free(Tab4IconPath);
	gtk_widget_show (Tab4Icon);
	gtk_grid_attach (GTK_GRID(Tab4), Tab4Icon, 0, 0, 1, 1);
	gtk_grid_attach (GTK_GRID(Tab4), Tab4Label, 1, 0, 1, 1);
	
	gtk_widget_show (Tab4);
	gtk_notebook_append_page(GTK_NOTEBOOK(gwidget->boxh),scroll4,Tab4);

	for(i=0;i<(uvc_videoIn->listDevices->num_devices);i++)
	{
		if(uvc_videoIn->listDevices->listVidDevices[i].current)
		{
			if((uvc_videoIn->listDevices->listVidDevices[i].vendor == SEE3CAM_USB_VID))
			{
				valid_econ_device = 1;
				if((uvc_videoIn->listDevices->listVidDevices[i].product == SEE3CAM_10CUG_M_PID) || (uvc_videoIn->listDevices->listVidDevices[i].product == SEE3CAM_10CUG_C_PID))
				{
					printf("vid: %x, pid: %x",uvc_videoIn->listDevices->listVidDevices[i].vendor, uvc_videoIn->listDevices->listVidDevices[i].product);
					curr_see3cam_dev = SEE3CAM_10CUG;
					break;
				}
				if(uvc_videoIn->listDevices->listVidDevices[i].product == SEE3CAM_11CUG_C_PID)
				{
					printf("vid: %x, pid: %x",uvc_videoIn->listDevices->listVidDevices[i].vendor, uvc_videoIn->listDevices->listVidDevices[i].product);
					curr_see3cam_dev = SEE3CAM_11CUG ;
					break;
				}
				if(uvc_videoIn->listDevices->listVidDevices[i].product == SEE3CAM_80USB_PID)
				{
					printf("vid: %x, pid: %x",uvc_videoIn->listDevices->listVidDevices[i].vendor, uvc_videoIn->listDevices->listVidDevices[i].product);
					curr_see3cam_dev = SEE3CAM_80USB ;
					break;
				}
				if(uvc_videoIn->listDevices->listVidDevices[i].product == SEE3CAM_30_Z10X_PID)
				{
					printf("vid: %x, pid: %x",uvc_videoIn->listDevices->listVidDevices[i].vendor, uvc_videoIn->listDevices->listVidDevices[i].product);
					curr_see3cam_dev = SEE3CAM_30_Z10X ;
					break;
				}
				if(uvc_videoIn->listDevices->listVidDevices[i].product == SEE3CAM_CU50_PID)
				{
					printf("vid: %x, pid: %x",uvc_videoIn->listDevices->listVidDevices[i].vendor, uvc_videoIn->listDevices->listVidDevices[i].product);
					curr_see3cam_dev = SEE3CAM_CU50 ;
					break;
				}
				else
				{
					valid_econ_device = 0;
					break;
				}
			}
		}
	}
	if(0==valid_econ_device)
	{
		printf("\nExtension Unit Control is not supported for the current %s device\n", uvc_videoIn->videodevice);
		gtk_widget_set_sensitive(table4, FALSE);
	}
	else
	{	
		ret = InitExtensionUnit(uvc_videoIn->cap.bus_info);
		if(ret == TRUE)
		{
			printf("E-con device : %s\n", uvc_videoIn->videodevice);
		        /*Add udev device monitoring timer*/
		        //global->hid_udev_timer_id=g_timeout_add( 500, check_hid_udev_events, &all_data);
		}
		else
		{
			ret=UninitExtensionUnit();
			printf("\nDevice Driver Initialisation is failed, please run 'see3camguvcview' as root (or with sudo)\n");
			gtk_widget_set_sensitive(table4, FALSE);
		}

		if(curr_see3cam_dev == SEE3CAM_10CUG) {
			Draw10CUGControl(table4,all_data);
		}else if(curr_see3cam_dev == SEE3CAM_11CUG ){
			Draw11CUGControl(table4,all_data);
		} else if(curr_see3cam_dev == SEE3CAM_80USB) {
			Draw80USBControl(table4,all_data);
		} else if(curr_see3cam_dev == SEE3CAM_30_Z10X) {
			DrawZ10XControl(table4,all_data);
		} else if(curr_see3cam_dev == SEE3CAM_CU50) {
			DrawCU50Control(table4,all_data);
		}
	}
	return;
}
