
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
#include "sys/mount.h"
#include "sys/stat.h"
#include "sys/sendfile.h"
#include "fcntl.h"
#include "unistd.h"

#include "xunit_tab_lib.h"
#include "xunit_lib.h"
#include "xunit_tab.h"


GtkWidget *Scene_normal_Radio;
GtkWidget *Scene_macro_Radio;
GtkWidget *check_Flip;
GtkWidget *check_Mirror;
GtkWidget *combo_Focus;
GtkWidget *FocusButt;
GtkWidget *Focus_Slider;
GtkWidget *Brightness_Slider;
GtkWidget *Contrast_Slider;
GtkWidget *Shutter_speed_Slider;
GtkWidget *Optical_zoom_Slider;
GtkWidget *Digital_zoom_Slider;
GtkWidget *Focus_Spin;
GtkWidget *Brightness_Spin;
GtkWidget *Contrast_Spin;
GtkWidget *Shutter_speed_Spin;
GtkWidget *Optical_zoom_Spin;
GtkWidget *Digital_zoom_Spin;

GtkWidget *combo_White_balance;
GtkWidget *Red_gain_Slider;
GtkWidget *Blue_gain_Slider;
#if 1
GtkWidget *Exposure_Slider;
GtkWidget *Exposure_minimum_Slider;
GtkWidget *Exposure_maximum_Slider;
GtkWidget *Exposure_Spin;
GtkWidget *Exposure_minimum_Spin;
GtkWidget *Exposure_maximum_Spin;
#endif
GtkWidget *Noise_filter_Slider;
GtkWidget *Red_gain_Spin;
GtkWidget *Blue_gain_Spin;
GtkWidget *Noise_filter_Spin;

GtkWidget *Entry_File_name;
GtkWidget *Compression_fine_Radio;
GtkWidget *Compression_best_Radio;
GtkWidget *Capture_single_Radio;
GtkWidget *Capture_burst_Radio;
GtkWidget *combo_Burst_size;

GtkWidget* FileDialog;
GtkWidget* check_Inc_Fname;

UINT8 g_combobox_choice;
UINT8 g_burst_size;
BOOL g_capture_mode;
//BOOL g_compression_mode;
UINT8 g_prev_focus_val=0;
UINT8 g_curr_focus_val=0;

void evt_fx320_focus_button_clicked(GtkButton *FocusButt, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 status=0;
	if(gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(Scene_normal_Radio))==TRUE)
		ret=SendCameraCommand(SET_CAM_AF,1,4,&status);
	else if(gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(Scene_macro_Radio))==TRUE)
		ret=SendCameraCommand(SET_CAM_AF,1,6,&status);
	if(ret == FALSE) {
		ShowErrorDialog("Error in setting Focus",all_data);
	} else {
		if(FX320_ACK == status)
			printf("\nsuccess\n");
		else if(FX320_NACK == status)
			printf("\nfail\n");
		else if(FX320_T_OUT == status)
			printf("\ntimeout\n");
		else
			printf("\nunknown return\n");
	}	
	return;
		
}

void evt_fx320_scene_changed(GtkRadioButton *Radio, struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	UINT8 status=0;
	struct GLOBAL *global = all_data->global;

	g_combobox_choice=gtk_combo_box_get_active(GTK_COMBO_BOX(combo_Focus));
	if(2 != g_combobox_choice){
		if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(Radio))) {
			global->FX320_scene_mode=2;
			if(0 == g_combobox_choice)
				ret=SendCameraCommand(SET_CAM_AF,1,5,&status);
			else if(1 == g_combobox_choice)
				ret=SendCameraCommand(SET_CAM_AF,1,3,&status);
		} else {
			global->FX320_scene_mode=1;
			if(0 == g_combobox_choice)
				ret=SendCameraCommand(SET_CAM_AF,1,6,&status);
			else if(1 == g_combobox_choice)
				ret=SendCameraCommand(SET_CAM_AF,1,4,&status);
		}
		if(ret == FALSE) {
			ShowErrorDialog("Error in setting Focus",all_data);
		} else {
			if(FX320_ACK == status)
				printf("\nsuccess\n");
			else if(FX320_NACK == status)
				printf("\nfail\n");
			else if(FX320_T_OUT == status)
				printf("\ntimeout\n");
			else
				printf("\nunknown return\n");
			} 
	}
	return;
}

void evt_fx320_Focus_changed(GtkComboBox *Focus, struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	UINT8 status=0;
	struct GLOBAL *global = all_data->global;

	g_combobox_choice=gtk_combo_box_get_active(Focus);
	switch(g_combobox_choice)
	{
		case 0:
		{
			global->FX320_focus_mode=1;
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Spin,FALSE);
			if(gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(Scene_normal_Radio))==TRUE)
				ret=SendCameraCommand(SET_CAM_AF,1,3,&status);
			else if(gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(Scene_macro_Radio))==TRUE)
				ret=SendCameraCommand(SET_CAM_AF,1,5,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting Focus",all_data);
			} else {
				if(FX320_ACK == status)
					printf("\nsuccess\n");
				else if(FX320_NACK == status)
					printf("\nfail\n");
				else if(FX320_T_OUT == status)
					printf("\ntimeout\n");
				else
					printf("\nunknown return\n");
			} 
			break;
		}
		case 1:
		{
			global->FX320_focus_mode=2;
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Spin,FALSE);
			if(gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(Scene_normal_Radio))==TRUE)
				ret=SendCameraCommand(SET_CAM_AF,1,4,&status);
			else if(gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(Scene_macro_Radio))==TRUE)
				ret=SendCameraCommand(SET_CAM_AF,1,6,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting Focus",all_data);
			} else {
				if(FX320_ACK == status)
					printf("\nsuccess\n");
				else if(FX320_NACK == status)
					printf("\nfail\n");
				else if(FX320_T_OUT == status)
					printf("\ntimeout\n");
				else
					printf("\nunknown return\n");
			} 
			break;
		}
		case 2:
		{
			global->FX320_focus_mode=3;
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Spin,TRUE);
				ret=SendCameraCommand(SET_CAM_AF,1,0,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting Focus",all_data);
			} else {
				if(FX320_ACK == status) {
					printf("\nsuccess\n");
					ret=SendCameraCommand(SET_FM_INC,2,gtk_range_get_value(GTK_RANGE(Focus_Slider)),&status);
					if(ret == FALSE) {
						ShowErrorDialog("Error in setting Focus value",all_data);
					} else {
						if(FX320_ACK == status)
							printf("\nsuccess\n");
						else if(FX320_NACK == status)
							printf("\nfail\n");
						else if(FX320_T_OUT == status)
							printf("\ntimeout\n");
						else
							printf("\nunknown return\n");
					} 
				} else if(FX320_NACK == status)
					printf("\nfail\n");
				else if(FX320_T_OUT == status)
					printf("\ntimeout\n");
				else
					printf("\nunknown return\n");
			} 
			break;
		}
		default:
		{
			printf("Error getting type of focus");
			break;
		}
	}
	return;
}
void evt_fx320_white_bal_changed(GtkComboBox *W_bal, struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	UINT8 status=0;
	struct GLOBAL *global = all_data->global;
	g_combobox_choice=gtk_combo_box_get_active(W_bal);
	global->FX320_wb_mode=g_combobox_choice+1;
	switch(g_combobox_choice)
	{
		case 0:
		case 1:
		case 2:
			gtk_widget_set_sensitive((GtkWidget*)Red_gain_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Blue_gain_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Red_gain_Spin,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Blue_gain_Spin,FALSE);
			ret=SendCameraCommand(SET_WHITE_BAL,1,g_combobox_choice,&status);
			if(ret == FALSE){ 
				ShowErrorDialog("Error in setting White Balance",all_data);
				return;
			}
				break;
		case 3: 
		{
			gtk_widget_set_sensitive((GtkWidget*)Red_gain_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Blue_gain_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Red_gain_Spin,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Blue_gain_Spin,FALSE);
				ret=SendCameraCommand(SET_WHITE_BAL,1,g_combobox_choice+1,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting White Balance",all_data);
				return;
			}
			break;
		}
		case 4:
		{
			gtk_widget_set_sensitive((GtkWidget*)Red_gain_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Blue_gain_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Red_gain_Spin,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Blue_gain_Spin,TRUE);
				ret=SendCameraCommand(SET_WHITE_BAL,1,g_combobox_choice+1,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting White Balance",all_data);
				return;
			} 
			break;
		}
		default:
		{
			printf("Error getting type of WB");
			return;
		}
	}
	if(FX320_ACK == status){
		if( 4 == g_combobox_choice){
			ret=SendCameraCommand(SET_CAM_RED,2,gtk_range_get_value((GtkRange*)Red_gain_Slider),&status);
			if(FALSE == ret)
			{
				printf("\nError in setting Red gain \n");
			}
			ret=SendCameraCommand(SET_CAM_BLUE,2,gtk_range_get_value((GtkRange*)Blue_gain_Slider),&status);
			if(FALSE == ret)
			{
				printf("\nError in setting Blue gain \n");
			}
				
		}
		printf("\nsuccess\n");
	} else if(FX320_NACK == status)
		printf("\nfail\n");
	else if(FX320_T_OUT == status)
		printf("\ntimeout\n");
	else
		printf("\nunknown return\n");
	return;
}
gboolean evt_fx320_slider_released(GtkRange *Slider,GdkEvent *event,struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	UINT8 status = 0 ;
	struct GLOBAL *global = all_data->global;
if ((GtkRange*)Optical_zoom_Slider == Slider) {
		global->FX320_optical_zoom = gtk_range_get_value((GtkRange*)Slider);
		ret=SendCameraCommand(SET_O_ZOOM,2,gtk_range_get_value((GtkRange*)Optical_zoom_Slider),&status);
		if(FALSE == ret)
		{
printf("\nError in setting optical zoom\n");
//			ShowErrorDialog("Error in setting Optical Zoom",all_data);
			return FALSE;
		}
} else if ((GtkRange*)Digital_zoom_Slider == Slider) {
		global->FX320_digital_zoom = gtk_range_get_value((GtkRange*)Slider);
		ret=SendCameraCommand(SET_D_ZOOM,1,gtk_range_get_value((GtkRange*)Digital_zoom_Slider),&status);
		if(FALSE == ret)
		{
printf("\nError in setting digital zoom\n");
//			ShowErrorDialog("Error in setting Digital Zoom",all_data);
			return FALSE;
		}
} else if ( (GtkRange*)Brightness_Slider == Slider || (GtkRange*)Contrast_Slider == Slider ) {
		global->FX320_brightness = gtk_range_get_value((GtkRange*)Brightness_Slider);
		global->FX320_contrast = gtk_range_get_value((GtkRange*)Contrast_Slider);
		UINT8 temp = 0xFF & (int)gtk_range_get_value((GtkRange*)Brightness_Slider);
		temp = temp<<4;
		temp |= (int)gtk_range_get_value((GtkRange*)Contrast_Slider);
		ret=SendCameraCommand(SET_CONTRAST_BRIGHT,2,temp,&status);
		if(FALSE == ret)
		{
printf("\nError in setting Brightness/Contrast \n");
//			ShowErrorDialog("Error in setting Digital Zoom",all_data);
			return FALSE;
		}
} else if ((GtkRange*)Shutter_speed_Slider == Slider) {
		global->FX320_shutter_speed = gtk_range_get_value((GtkRange*)Slider);
		ret=SendCameraCommand(SET_SHUTTER_SPEED,1,gtk_range_get_value((GtkRange*)Shutter_speed_Slider),&status);
		if(FALSE == ret)
		{
printf("\nError in setting Shutter Speed \n");
//			ShowErrorDialog("Error in setting Digital Zoom",all_data);
			return FALSE;
		}
} else if ((GtkRange*)Red_gain_Slider == Slider) {
		global->FX320_red_gain = gtk_range_get_value((GtkRange*)Slider);
		ret=SendCameraCommand(SET_CAM_RED,2,gtk_range_get_value((GtkRange*)Red_gain_Slider),&status);
		if(FALSE == ret)
		{
printf("\nError in setting Red gain \n");
//			ShowErrorDialog("Error in setting Digital Zoom",all_data);
			return FALSE;
		}
} else if ((GtkRange*)Blue_gain_Slider == Slider) {
		global->FX320_blue_gain = gtk_range_get_value((GtkRange*)Slider);
		ret=SendCameraCommand(SET_CAM_BLUE,2,gtk_range_get_value((GtkRange*)Blue_gain_Slider),&status);
		if(FALSE == ret)
		{
printf("\nError in setting Blue gain \n");
//			ShowErrorDialog("Error in setting Digital Zoom",all_data);
			return FALSE;
		}
} else if ((GtkRange*)Noise_filter_Slider == Slider) {
		global->FX320_temp_noise_filter = gtk_range_get_value((GtkRange*)Slider);
		ret=SendCameraCommand(SET_TEMP_FILT,1,gtk_range_get_value((GtkRange*)Noise_filter_Slider),&status);
		if(FALSE == ret)
		{
printf("\nError in setting Temporal noise filter strength \n");
//			ShowErrorDialog("Error in setting Digital Zoom",all_data);
			return FALSE;
		}
}
		if(FX320_ACK == status)
			printf("\nsuccess\n");
		else if(FX320_NACK == status)
			printf("\nfail\n");
		else if(FX320_T_OUT == status)
			printf("\ntimeout\n");
		else
			printf("\nunknown return\n");
	return FALSE;
}
void evt_fx320_check_button_toggled(GtkWidget *widget, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 status = 0 ;
	struct GLOBAL *global = all_data->global;
	if(!gtk_toggle_button_get_active((GtkToggleButton*)check_Flip) && !gtk_toggle_button_get_active((GtkToggleButton*)check_Mirror) ) {
		global->FX320_orientation=1;
		ret=SendCameraCommand(SET_CAM_ORIENTATION,1,0,&status);
	} else if(!gtk_toggle_button_get_active((GtkToggleButton*)check_Flip) && gtk_toggle_button_get_active((GtkToggleButton*)check_Mirror) ) {
		global->FX320_orientation=2;
		ret=SendCameraCommand(SET_CAM_ORIENTATION,1,1,&status);
	} else if(gtk_toggle_button_get_active((GtkToggleButton*)check_Flip) && !gtk_toggle_button_get_active((GtkToggleButton*)check_Mirror) ) {
		global->FX320_orientation=3;
		ret=SendCameraCommand(SET_CAM_ORIENTATION,1,2,&status);
	} else if(gtk_toggle_button_get_active((GtkToggleButton*)check_Flip) && gtk_toggle_button_get_active((GtkToggleButton*)check_Mirror) ) {
		global->FX320_orientation=4;
		ret=SendCameraCommand(SET_CAM_ORIENTATION,1,3,&status);
	}
	if(FALSE == ret)
	{
printf("\nError in setting camera orientation\n");
//			ShowErrorDialog("Error in setting Optical Zoom",all_data);
	}		
	return;
}

void set_all_controls_defaults(struct ALL_DATA *all_data)
{
	//set defult scene mode : normal
	gtk_toggle_button_set_active((GtkToggleButton*)Scene_normal_Radio,TRUE);

	//set default AF mode : Continous AF mode
	g_signal_handlers_block_by_func(GTK_COMBO_BOX_TEXT(combo_Focus), G_CALLBACK (evt_fx320_Focus_changed), all_data);
	gtk_combo_box_set_active((GtkComboBox*)combo_Focus,0);
	g_signal_handlers_unblock_by_func(GTK_COMBO_BOX_TEXT(combo_Focus), G_CALLBACK (evt_fx320_Focus_changed), all_data);

	//Disable focus button, spin and slider
	gtk_widget_set_sensitive(FocusButt,FALSE);
	gtk_widget_set_sensitive(Focus_Slider,FALSE);
	gtk_widget_set_sensitive(Focus_Spin,FALSE);

	//set default brightness value : 8
	g_signal_handlers_block_by_func(GTK_RANGE(Brightness_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);
	gtk_range_set_value((GtkRange*)Brightness_Slider,8);
	g_signal_handlers_unblock_by_func(GTK_RANGE(Brightness_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);

	//set default contrast value : 8
	g_signal_handlers_block_by_func(GTK_RANGE(Contrast_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);
	gtk_range_set_value((GtkRange*)Contrast_Slider,8);
	g_signal_handlers_unblock_by_func(GTK_RANGE(Contrast_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);

	//set default shutter speed value : 0
	g_signal_handlers_block_by_func(GTK_RANGE(Shutter_speed_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);
	gtk_range_set_value((GtkRange*)Shutter_speed_Slider,0);
	g_signal_handlers_unblock_by_func(GTK_RANGE(Shutter_speed_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);

	//set default optical zoom value : 0
	g_signal_handlers_block_by_func(GTK_RANGE(Optical_zoom_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);
	gtk_range_set_value((GtkRange*)Optical_zoom_Slider,0);
	g_signal_handlers_unblock_by_func(GTK_RANGE(Optical_zoom_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);

	//set default digital zoom value : 0
	g_signal_handlers_block_by_func(GTK_RANGE(Digital_zoom_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);
	gtk_range_set_value((GtkRange*)Digital_zoom_Slider,0);
	g_signal_handlers_unblock_by_func(GTK_RANGE(Digital_zoom_Slider), G_CALLBACK (evt_fx320_slider_released), all_data);

	//set default Flip mode : disabled
	g_signal_handlers_block_by_func((GtkToggleButton*)(check_Flip), G_CALLBACK (evt_fx320_check_button_toggled), all_data);
	gtk_toggle_button_set_active((GtkToggleButton*)check_Flip,FALSE);
	g_signal_handlers_unblock_by_func((GtkToggleButton*)(check_Flip), G_CALLBACK (evt_fx320_check_button_toggled), all_data);

	//set default Mirror mode : disabled
	g_signal_handlers_block_by_func((GtkToggleButton*)(check_Mirror), G_CALLBACK (evt_fx320_check_button_toggled), all_data);
	gtk_toggle_button_set_active((GtkToggleButton*)check_Mirror,FALSE);
	g_signal_handlers_unblock_by_func((GtkToggleButton*)(check_Mirror), G_CALLBACK (evt_fx320_check_button_toggled), all_data);

	return;
}

void evt_z10x_update_spin(GtkRange *Slider,struct ALL_DATA *all_data)
{
	if(Slider == (GtkRange*)Focus_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Focus_Spin), gtk_range_get_value((GtkRange*)Focus_Slider));
	} else if(Slider == (GtkRange*)Brightness_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Brightness_Spin), gtk_range_get_value((GtkRange*)Brightness_Slider));
	} else if(Slider == (GtkRange*)Contrast_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Contrast_Spin), gtk_range_get_value((GtkRange*)Contrast_Slider));
	} else if(Slider == (GtkRange*)Shutter_speed_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Shutter_speed_Spin), gtk_range_get_value((GtkRange*)Shutter_speed_Slider));
	} else if(Slider == (GtkRange*)Optical_zoom_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Optical_zoom_Spin), gtk_range_get_value((GtkRange*)Optical_zoom_Slider));
	} else if(Slider == (GtkRange*)Digital_zoom_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Digital_zoom_Spin), gtk_range_get_value((GtkRange*)Digital_zoom_Slider));
	} else if(Slider == (GtkRange*)Red_gain_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Red_gain_Spin), gtk_range_get_value((GtkRange*)Red_gain_Slider));
	} else if(Slider == (GtkRange*)Blue_gain_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Blue_gain_Spin), gtk_range_get_value((GtkRange*)Blue_gain_Slider));
	} else if(Slider == (GtkRange*)Exposure_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Exposure_Spin), gtk_range_get_value((GtkRange*)Exposure_Slider));
	} else if(Slider == (GtkRange*)Exposure_minimum_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Exposure_minimum_Spin), gtk_range_get_value((GtkRange*)Exposure_minimum_Slider));
	} else if(Slider == (GtkRange*)Exposure_maximum_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Exposure_maximum_Spin), gtk_range_get_value((GtkRange*)Exposure_maximum_Slider));
	} else if(Slider == (GtkRange*)Noise_filter_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(Noise_filter_Spin), gtk_range_get_value((GtkRange*)Noise_filter_Slider));
	}
	return;
}

gboolean evt_fx320_manual_slider_pressed(GtkRange *Slider,GdkEvent *event,struct ALL_DATA *all_data)
{
	g_prev_focus_val=gtk_range_get_value((GtkRange*)Slider);
	return FALSE;
}
gboolean evt_fx320_manual_slider_released(GtkRange *Slider,GdkEvent *event,struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	struct GLOBAL *global = all_data->global;
	UINT8 status = 0 ;
	if(2==gtk_combo_box_get_active((GtkComboBox*)combo_Focus)) {
		g_curr_focus_val=gtk_range_get_value((GtkRange*)Slider);
		global->FX320_focus_position=g_curr_focus_val;
		if (g_prev_focus_val<g_curr_focus_val)
			ret=SendCameraCommand(SET_FM_INC,2,(g_curr_focus_val),&status);
		else if (g_prev_focus_val>g_curr_focus_val)
			ret=SendCameraCommand(SET_FM_DEC,2,(g_curr_focus_val),&status);
		else
			return FALSE;
		if(FALSE == ret)
		{
printf("\nError in setting Manual focus position \n");
//			ShowErrorDialog("Error in setting Digital Zoom",all_data);
			return FALSE;
		}
		if(FX320_ACK == status)
			printf("\nsuccess\n");
		else if(FX320_NACK == status)
			printf("\nfail\n");
		else if(FX320_T_OUT == status)
			printf("\ntimeout\n");
		else
			printf("\nunknown return\n");
	} else
		printf("\nFocus mode is not in Manual mode\n");
	return FALSE;
}

void ShowAdvancedControlsWindow(GtkButton *AdvancedButt, struct ALL_DATA *all_data)
{
	GtkWidget *label_Advanced_title;
	GtkWidget *label_White_balance;
	GtkWidget *label_Red_gain;
	GtkWidget *label_Blue_gain;

	GtkWidget *label_Noise_filter;

	GtkAdjustment *Red_gain_SliderAdjust;
	GtkAdjustment *Blue_gain_SliderAdjust;
	GtkAdjustment *Noise_filter_SliderAdjust;

	UINT8 row = 0;
	struct GLOBAL *global = all_data->global;

	//Create new dialog for advanced controls
	GtkWidget *advanced_controls_dialog = gtk_dialog_new ();
	gtk_window_set_default_size(GTK_WINDOW(advanced_controls_dialog),500,350);

	//Get the content area of the dialog window
	GtkWidget *content_area = gtk_dialog_get_content_area (GTK_DIALOG(advanced_controls_dialog));

	//Draw a table in the dialog
	GtkWidget *table = gtk_grid_new();
	gtk_grid_set_column_homogeneous (GTK_GRID(table), FALSE);
	gtk_widget_set_hexpand (table, TRUE);
	gtk_widget_set_halign (table, GTK_ALIGN_FILL);
	
	gtk_grid_set_row_spacing (GTK_GRID(table), 4);
	gtk_grid_set_column_spacing (GTK_GRID (table), 4);
	gtk_container_set_border_width (GTK_CONTAINER (table), 4);
	gtk_widget_show (table);

	row++;

	//Label "Advanced Camera Controls"
	label_Advanced_title=gtk_label_new(_("---- Advanced Camera Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_Advanced_title), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Advanced_title, 1, row, 3, 1);
	gtk_widget_show (label_Advanced_title);

	row++;

	//Label "White Balance"
	label_White_balance=gtk_label_new(_("White Balance :"));
	gtk_misc_set_alignment (GTK_MISC (label_White_balance), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_White_balance, 1, row, 1, 1);
	gtk_widget_show (label_White_balance);

	//Combo box for white balance
	combo_White_balance = gtk_combo_box_text_new ();
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_White_balance),"Auto");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_White_balance),"Daylight");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_White_balance),"Cloudy");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_White_balance),"Fluorescent");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_White_balance),"Manual");
	gtk_widget_set_halign(combo_White_balance,GTK_ALIGN_FILL);
	gtk_widget_set_hexpand(combo_White_balance,TRUE);
	gtk_grid_attach(GTK_GRID(table),combo_White_balance,2,row,1,1);
	gtk_widget_show (combo_White_balance);

	row++;

	//Label "Red Gain"
	label_Red_gain=gtk_label_new(_("Red Gain :"));
	gtk_misc_set_alignment (GTK_MISC (label_Red_gain), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Red_gain, 1, row, 1, 1);
	gtk_widget_show (label_Red_gain);

	//Slider for setting red gain
	Red_gain_SliderAdjust=gtk_adjustment_new(0,0,256,1,1,1);
	Red_gain_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Red_gain_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Red_gain_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Red_gain_Slider), 0);
	gtk_grid_attach (GTK_GRID(table), Red_gain_Slider, 2, row, 1, 1);
	gtk_widget_show (Red_gain_Slider);

	//Spin button for red gain
	Red_gain_Spin=gtk_spin_button_new_with_range(0,255,1);
	gtk_editable_set_editable(GTK_EDITABLE(Red_gain_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table), Red_gain_Spin, 3, row, 1, 1);
	gtk_widget_show (Red_gain_Spin);
if(0<=global->FX320_red_gain && 256>global->FX320_red_gain) {
	gtk_range_set_value (GTK_RANGE (Red_gain_Slider), global->FX320_red_gain);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Red_gain_Spin), global->FX320_red_gain);
}
	g_signal_connect (GTK_SCALE(Red_gain_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Red_gain_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Red_gain_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

	row++;

	//Label "Blue Gain"
	label_Blue_gain=gtk_label_new(_("Blue Gain :"));
	gtk_misc_set_alignment (GTK_MISC (label_Blue_gain), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Blue_gain, 1, row, 1, 1);
	gtk_widget_show (label_Blue_gain);

	//Slider for setting blue gain
	Blue_gain_SliderAdjust=gtk_adjustment_new(0,0,256,1,1,1);
	Blue_gain_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Blue_gain_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Blue_gain_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Blue_gain_Slider), 0);
	gtk_grid_attach (GTK_GRID(table), Blue_gain_Slider, 2, row, 1, 1);
	gtk_widget_show (Blue_gain_Slider);

	//Spin button for blue gain
	Blue_gain_Spin=gtk_spin_button_new_with_range(0,255,1);
	gtk_editable_set_editable(GTK_EDITABLE(Blue_gain_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table), Blue_gain_Spin, 3, row, 1, 1);
	gtk_widget_show (Blue_gain_Spin);
if(0<=global->FX320_blue_gain && 256>global->FX320_blue_gain) {
	gtk_range_set_value (GTK_RANGE (Blue_gain_Slider), global->FX320_blue_gain);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Blue_gain_Spin), global->FX320_blue_gain);
}
	g_signal_connect (GTK_SCALE(Blue_gain_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Blue_gain_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Blue_gain_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

if(1<=global->FX320_wb_mode && 6>global->FX320_wb_mode) {
	if(5==global->FX320_wb_mode) {
	gtk_widget_set_sensitive(GTK_WIDGET(Red_gain_Slider),TRUE);
	gtk_widget_set_sensitive(GTK_WIDGET(Blue_gain_Slider),TRUE);
	} else {
	gtk_widget_set_sensitive(GTK_WIDGET(Red_gain_Slider),FALSE);
	gtk_widget_set_sensitive(GTK_WIDGET(Blue_gain_Slider),FALSE);
	}
	gtk_combo_box_set_active(GTK_COMBO_BOX(combo_White_balance),(global->FX320_wb_mode)-1);
}
	g_signal_connect(GTK_COMBO_BOX_TEXT(combo_White_balance), "changed",G_CALLBACK (evt_fx320_white_bal_changed), all_data);

	row++;

	//Label "Noise Filter Threshold"
	label_Noise_filter=gtk_label_new(_("Noise Filter Threshold :"));
	gtk_misc_set_alignment (GTK_MISC (label_Noise_filter), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Noise_filter, 1, row, 1, 1);
	gtk_widget_show (label_Noise_filter);

	//Slider for setting noise filter threshold
	Noise_filter_SliderAdjust=gtk_adjustment_new(0,0,16,1,1,1);
	Noise_filter_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Noise_filter_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Noise_filter_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Noise_filter_Slider), 0);
	gtk_grid_attach (GTK_GRID(table), Noise_filter_Slider, 2, row, 1, 1);
	gtk_widget_show (Noise_filter_Slider);

	//Spin button for noise filter threshold
	Noise_filter_Spin=gtk_spin_button_new_with_range(0,15,1);
	gtk_editable_set_editable(GTK_EDITABLE(Noise_filter_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table), Noise_filter_Spin, 3, row, 1, 1);
	gtk_widget_show (Noise_filter_Spin);
if(0<=global->FX320_temp_noise_filter && 16>global->FX320_temp_noise_filter) {
	gtk_range_set_value (GTK_RANGE (Noise_filter_Slider), global->FX320_temp_noise_filter);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Noise_filter_Spin), global->FX320_temp_noise_filter);
}
	g_signal_connect (GTK_SCALE(Noise_filter_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Noise_filter_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Noise_filter_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

	gtk_container_add (GTK_CONTAINER (content_area), table);
	gtk_dialog_add_buttons (GTK_DIALOG (advanced_controls_dialog),
		_("_OK"),
		GTK_RESPONSE_OK,
		NULL);

	gtk_window_set_title(GTK_WINDOW (advanced_controls_dialog),"See3CAM Advanced Controls");
	gtk_widget_show_all (advanced_controls_dialog);

	gtk_dialog_run (GTK_DIALOG (advanced_controls_dialog));
	gtk_widget_destroy(advanced_controls_dialog);

	return;
}

int CopyFile(const char* source, const char* destination)
{    
    int input, output;    
    if ((input = open(source, O_RDONLY)) == -1)
    {
        return -1;
    }    
    if ((output = open(destination, O_RDWR | O_CREAT, 0666)) == -1)
    {
        close(input);
        return -1;
    }

    //Here we use kernel-space copying for performance reasons
    //sendfile will work with non-socket output (i.e. regular file) on Linux 2.6.33+
    off_t bytesCopied = 0;
    struct stat fileinfo = {0};
    fstat(input, &fileinfo);
    int result = sendfile(output, input, &bytesCopied, fileinfo.st_size);

    close(input);
    close(output);

    return result;
}

void evt_inc_fname_toggled(GtkWidget *widget, struct ALL_DATA *all_data)
{
	struct GLOBAL *global = all_data->global;
	global->FX320_image_inc = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(check_Inc_Fname)) ? 1 : 0;
	if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(check_Inc_Fname)))
		gtk_file_chooser_set_do_overwrite_confirmation (GTK_FILE_CHOOSER (FileDialog), TRUE);
	else
		gtk_file_chooser_set_do_overwrite_confirmation (GTK_FILE_CHOOSER (FileDialog), FALSE);
	return;
}
void UpdateFilenames(struct ALL_DATA *all_data)
{
	int i;
	struct GLOBAL *global = all_data->global;
	//Dont increment filename based on actual file. Keep fixed file names.
	if(!global->FX320_image_inc>0){	
		for(i=0;i<8;i++){
			global->FX320_imgFName[i] = g_strdup(global->FX320_imgFPath[0]);
			global->FX320_imgFName[i] = incFilename(global->FX320_imgFName[i],global->FX320_imgFPath,i+1);
			printf("FX320_imgFName[%d] : %s\n",i,global->FX320_imgFName[i]);
		}
	//Increment filename. First get the current file suffix reading from file
	}else{
		uint64_t suffix = get_file_suffix(global->FX320_imgFPath[1], global->FX320_imgFPath[0]);
		fprintf(stderr, "Image file suffix detected: %" PRIu64 "\n", suffix);
		for(i=0;i<8;i++){
			global->FX320_imgFName[i] = g_strdup(global->FX320_imgFPath[0]);
			global->FX320_imgFName[i] = incFilename(global->FX320_imgFName[i],global->FX320_imgFPath,i+1+suffix);
			printf("FX320_imgFName[%d] : %s\n",i,global->FX320_imgFName[i]);
		}
		if(suffix >= G_MAXUINT64)
		{
			global->FX320_imgFPath[0] = add_file_suffix(global->FX320_imgFPath[0], suffix);
			suffix = 0;
		}
		if(suffix >= 0)
			global->FX320_image_inc = suffix + 1;
	}
	return;
}

void
z10x_file_chooser (GtkWidget * OpenButt, struct ALL_DATA *all_data)
{
	struct GWIDGET *gwidget = all_data->gwidget;
	struct GLOBAL *global = all_data->global;

	FileDialog = gtk_file_chooser_dialog_new (_("See3CAM Save File"),
		GTK_WINDOW(gwidget->mainwin),
		GTK_FILE_CHOOSER_ACTION_SAVE,
		"_Cancel", GTK_RESPONSE_CANCEL,
		"_Save", GTK_RESPONSE_ACCEPT,
		NULL);

	GError *error = NULL;
	GtkIconTheme *icon_theme;
	GdkPixbuf *pixbuf;
	icon_theme = gtk_icon_theme_get_default();
	pixbuf = gtk_icon_theme_load_icon(icon_theme,"document-save",20,0,&error);
	if(!pixbuf){
		g_warning("couldnt load icon : %s",error->message);
		g_error_free(error);
	}else{
		gtk_button_set_image(GTK_BUTTON(gtk_dialog_get_widget_for_response (GTK_DIALOG(FileDialog),GTK_RESPONSE_ACCEPT)),gtk_image_new_from_pixbuf(pixbuf));
		g_object_unref(pixbuf);
	}

	/** create a file filter */
	GtkFileFilter *filter = gtk_file_filter_new();

	GtkWidget *FBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
	GtkWidget *format_label = gtk_label_new(_("See3CAM File Format:"));
	gtk_widget_set_halign (FBox, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (FBox, TRUE);
	gtk_widget_set_hexpand (format_label, FALSE);
	gtk_widget_show(FBox);
	gtk_widget_show(format_label);
	gtk_box_pack_start(GTK_BOX(FBox), format_label, FALSE, FALSE, 2);

	/* Image File chooser*/
	gtk_file_chooser_set_current_folder (GTK_FILE_CHOOSER (FileDialog),global->FX320_imgFPath[1]);
	gtk_file_chooser_set_current_name (GTK_FILE_CHOOSER (FileDialog),global->FX320_imgFPath[0]);
int i;
for(i=0;i<2;i++)
g_print("file path[%d] : %s \n",i,global->FX320_imgFPath[i]);
	/** add format file filters*/
	GtkWidget *ImgFormat = gtk_combo_box_text_new ();
	gtk_widget_set_halign (ImgFormat, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (ImgFormat, TRUE);

	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ImgFormat),_("Jpeg"));
	gtk_combo_box_set_active(GTK_COMBO_BOX(ImgFormat), 0);
	gtk_box_pack_start(GTK_BOX(FBox), ImgFormat, FALSE, FALSE, 2);
	gtk_widget_show(ImgFormat);

	//Check button for increment filename
	check_Inc_Fname=gtk_check_button_new_with_label(_("Increment Filename"));
	gtk_widget_set_halign (check_Inc_Fname, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (check_Inc_Fname, TRUE);
	gtk_box_pack_start(GTK_BOX(FBox), check_Inc_Fname, FALSE, FALSE, 2);
	gtk_widget_show (check_Inc_Fname);
	gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (check_Inc_Fname), (global->FX320_image_inc > 0));
	g_signal_connect (GTK_BUTTON(check_Inc_Fname), "toggled",G_CALLBACK (evt_inc_fname_toggled), all_data);

	/**add a pattern to the filter*/
	gtk_file_filter_add_pattern(filter, "*.jpg");
	gtk_file_chooser_set_filter(GTK_FILE_CHOOSER (FileDialog), filter);

	gtk_file_chooser_set_extra_widget(GTK_FILE_CHOOSER (FileDialog), FBox);

	if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(check_Inc_Fname)))
		gtk_file_chooser_set_do_overwrite_confirmation (GTK_FILE_CHOOSER (FileDialog), TRUE);
	else
		gtk_file_chooser_set_do_overwrite_confirmation (GTK_FILE_CHOOSER (FileDialog), FALSE);
	if (gtk_dialog_run (GTK_DIALOG (FileDialog)) == GTK_RESPONSE_ACCEPT)
	{
		gchar *fullname = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (FileDialog));
		global->FX320_imgFPath=splitPath(fullname, global->FX320_imgFPath);
		g_free(fullname);
		UpdateFilenames(all_data);
		gtk_entry_set_text((GtkEntry*)Entry_File_name,global->FX320_imgFPath[0]);
	}

	gtk_widget_destroy (FileDialog);
}

void evt_compression_changed(GtkWidget* Radio,struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 status = 0;
	struct GLOBAL *global = all_data->global;

	global->FX320_compression = !gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(Compression_fine_Radio));
	ret=SendCameraCommand(SET_COMP_RATIO,1,global->FX320_compression,&status);	
	if(FALSE == ret)
	{
		printf("\nError in setting compression ratio \n");
//			ShowErrorDialog("Error in setting Capture",all_data);
		return;
	}
	if(FX320_ACK == status){
		printf("\nsucces\n");
	} else if(FX320_NACK == status)
		printf("\nfail\n");
	else if(FX320_T_OUT == status)
		printf("\ntimeout\n");
	else
		printf("\nunknown return\n");
	return;
}

void evt_capture_mode_changed(GtkWidget* Radio,struct ALL_DATA *all_data)
{
	if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(Capture_single_Radio))){
		gtk_widget_set_sensitive(GTK_WIDGET(combo_Burst_size),TRUE);
		g_capture_mode=TRUE; //Burst mode
	} else {
		gtk_widget_set_sensitive(GTK_WIDGET(combo_Burst_size),FALSE);
		g_capture_mode=FALSE; //Single mode
	}
	return;
}

void evt_burst_size_changed(GtkWidget* Button,struct ALL_DATA *all_data)
{
	g_burst_size = gtk_combo_box_get_active(GTK_COMBO_BOX(combo_Burst_size));
	return;
}

void evt_z10x_capture(GtkWidget* OkButton,struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	BOOL timeout = TRUE;
	unsigned int start, end = 0;
	UINT8 status = 0;
	char storage_path[10];
	struct GLOBAL *global = all_data->global;
	if(FALSE == g_capture_mode) { //single capture mode 1
		ret=SendCameraCommand(SET_CAPTURE,1,1,&status);	
	}else{
		ret=SendCameraCommand(SET_CAPTURE,1,gtk_combo_box_get_active(GTK_COMBO_BOX(combo_Burst_size))+2,&status);	
	}
	if(FALSE == ret)
	{
		printf("\nError in capturing \n");
//			ShowErrorDialog("Error in setting Capture",all_data);
		ret=SendCameraCommand(SET_CAPTURE,1,13,&status);	
		if(FALSE == ret)
		{
			printf("\nError in deleting images \n");
			return;
		}
		return;
	}
	if(FX320_ACK == status)
	{
		printf("\nsuccess in capturing. \n");
		//TODO : add a 3 seconds wait before doing any operation to
		// wait for MASS STORAGE device to be enumerated.
		sleep(3);
		// Then copy the images to your preferred location. (Do NOT cut the images)
		start = GetTickCount();
		while(timeout) 
		{
			ret=find_z10x_storage(&storage_path);
			end = GetTickCount();
			if(TRUE == ret){
				timeout = FALSE;
			}
			if(end - start > FX320_TIMEOUT){
				printf("%s(): Timeout occurred\n", __func__);
				timeout = FALSE;
			}
		}
		if(FALSE == ret)
		{
			printf("\nError in getting mass storage device \n");
			return;
		} else {
			printf("\nPath  :  %s \n",storage_path);
			//TODO : check if temporary path already exists otherwise create one for mounting
			struct stat s;
			int err = stat("/mnt_z10x", &s);
			if(-1 == err) {
				perror("stat");
				printf("\nCreating Directory /mnt_z10x\n");
				err = mkdir("/mnt_z10x",0777);
				if(-1 == err) {
					perror("mkdir");
				}
			} else {
				if(S_ISDIR(s.st_mode)) {
			        	/* it's a dir */
					printf("\nAlready exists and also a directory. exiting stat..\n");
				} else {
				        /* exists but is no dir */
					printf("\nAlready exists but not a directory.\n");
					printf("\nunlinking file z10x\n");
					err = unlink("/mnt_z10x");	
					if(-1 == err) {
						perror("unlink");
					}
					printf("\nnow Creating Directory /mnt_z10x\n");
					err = mkdir("/mnt_z10x",0777);
					if(-1 == err) {
						perror("mkdir");
					}
				}
			}
			//TODO : Mount the path in a temporary path .(/mnt_z10x)
			printf("storage_path = %s \n",storage_path);
			err=mount(storage_path,"/mnt_z10x","vfat",MS_SILENT,"");
			if(-1 == err) {
				perror("mount");
				if(errno == EBUSY){
					printf("Already mounted on /mnt_z10x. so unmounting\n");
					err=umount("/mnt_z10x");
					if(-1 == err) {
						perror("umount");
					}else{
						printf("Success unmounting\n");
					}
					printf("remounting \n");
					err=mount(storage_path,"/mnt_z10x","vfat",MS_SILENT,"");
					if(-1 == err){
						perror("mount");
					}else {
						printf("Success mounting\n");
					}
				}else {
					printf("removing dir /mnt_z10x\n");
					remove("/mnt_z10x");
					printf("Deleting Images\n");
					ret=SendCameraCommand(SET_CAPTURE,1,13,&status);	
					if(FALSE == ret) {
						printf("\nError in deleting images \n");
						return;
					}
					printf("Exiting\n");
					return;
				}
			}else {
				printf("Success mounting\n");
			}
			//TODO : copy files based on mode of capture (single or burst) and burst size
			if(FALSE == g_capture_mode) { //single capture mode 1
				printf("copy %s to %s \n",global->FX320_imgFName_read[0],global->FX320_imgFName[0]);
				err = CopyFile(global->FX320_imgFName_read[0],global->FX320_imgFName[0]);
				if(err < 0)
				{
					printf("error copying %s\n",global->FX320_imgFName_read[0]);
				}else{ 
					printf("success copying %s\n",global->FX320_imgFName_read[0]);
				}
			}else{
				int copied_bytes =0;
				int i;
				g_combobox_choice = gtk_combo_box_get_active(GTK_COMBO_BOX(combo_Burst_size));
				for(i=g_combobox_choice+2;i!=0;i--){
					printf("iter : %d copy %s to %s \n",i,global->FX320_imgFName_read[i-1],global->FX320_imgFName[i-1]);
					err = CopyFile(global->FX320_imgFName_read[i-1],global->FX320_imgFName[i-1]);
					copied_bytes+=err;
					if(err < 0)
						printf("error copying %s\n",global->FX320_imgFName_read[i-1]);
					else
						printf("success copying %s\n",global->FX320_imgFName_read[i-1]);
				}
				printf("Copied %d bytes\n",err);
			}
			err=umount("/mnt_z10x");
			if(-1 == err) {
				perror("umount");
			}else{
				printf("Success unmounting\n");
			}
			err = remove("/mnt_z10x");
			if(-1 == err) {
				perror("remove");
			}else{ 
				printf("Success removing dir /mnt_z10x\n");
			}
		}
		ret=SendCameraCommand(SET_CAPTURE,1,13,&status);	
		if(FALSE == ret)
		{
			printf("\nError in deleting images \n");
			return;
		}
		if(FX320_ACK == status)
		{
			printf("Success deleting images\n");
		} else if(FX320_NACK == status)
			printf("\nfail\n");
		else if(FX320_T_OUT == status)
			printf("\ntimeout\n");
		else
			printf("\nunknown return\n");
	} else if(FX320_NACK == status)
		printf("\nfail\n");
	else if(FX320_T_OUT == status)
		printf("\ntimeout\n");
	else
		printf("\nunknown return\n");

	return;
}

void ShowCapturePropertiesWindow(GtkWidget* table4,struct ALL_DATA *all_data)
{
	GtkWidget *label_Image_name;
	GtkWidget *OpenButt;
	GtkWidget *label_Capture_mode;
	GtkWidget *label_Compression;
	GtkWidget *label_Burst_size;
	UINT8 row = 0;

	struct GLOBAL *global = all_data->global;

	//Create new dialog for advanced controls
	GtkWidget *capture_properties_dialog = gtk_dialog_new ();
	gtk_window_set_default_size(GTK_WINDOW(capture_properties_dialog),500,200);

	//Get the content area of the dialog window
	GtkWidget *content_area = gtk_dialog_get_content_area (GTK_DIALOG(capture_properties_dialog));

	//Draw a table in the dialog
	GtkWidget *table = gtk_grid_new();
	gtk_grid_set_column_homogeneous (GTK_GRID(table), FALSE);
	gtk_widget_set_hexpand (table, TRUE);
	gtk_widget_set_halign (table, GTK_ALIGN_FILL);
	
	gtk_grid_set_row_spacing (GTK_GRID(table), 4);
	gtk_grid_set_column_spacing (GTK_GRID (table), 4);
	gtk_container_set_border_width (GTK_CONTAINER (table), 4);
	gtk_widget_show (table);

	row++;

	//Label "Image File"
	label_Image_name=gtk_label_new(_("Image File name :"));
	gtk_misc_set_alignment (GTK_MISC (label_Image_name), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Image_name, 1, row, 1, 1);
	gtk_widget_show (label_Image_name);

	//Text box to display file name
	Entry_File_name=gtk_entry_new();
	gtk_editable_set_editable(GTK_EDITABLE(Entry_File_name),FALSE);
	gtk_grid_attach (GTK_GRID(table), Entry_File_name, 2, row, 1, 1);
	gtk_widget_show (Entry_File_name);
	gtk_entry_set_text((GtkEntry*)Entry_File_name,global->FX320_imgFPath[0]);

	OpenButt = gtk_button_new_with_label (_("     Open     "));
	gtk_grid_attach (GTK_GRID(table), OpenButt, 3, row, 1, 1);
	gtk_widget_show (OpenButt);

	g_signal_connect (GTK_BUTTON(OpenButt), "clicked", G_CALLBACK (z10x_file_chooser), all_data);
	UpdateFilenames(all_data);

	row++;

	//Label "Compression"
	label_Compression=gtk_label_new(_("Compression Level :"));
	gtk_misc_set_alignment (GTK_MISC (label_Compression), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Compression, 1, row, 1, 1);
	gtk_widget_show (label_Compression);

	Compression_fine_Radio = gtk_radio_button_new_with_label (NULL, _("Fine"));
	Compression_best_Radio = gtk_radio_button_new_with_label (NULL, _("Best"));
	gtk_radio_button_join_group((GtkRadioButton*)Compression_fine_Radio,(GtkRadioButton*)Compression_best_Radio);
	gtk_grid_attach (GTK_GRID(table), Compression_fine_Radio, 2, row, 1, 1);
	gtk_widget_show (Compression_fine_Radio);

	row++;

	gtk_grid_attach (GTK_GRID(table), Compression_best_Radio, 2, row, 1, 1);
	gtk_widget_show (Compression_best_Radio);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(Compression_fine_Radio),TRUE);

if(global->FX320_compression)
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(Compression_best_Radio),TRUE);

	g_signal_connect (GTK_TOGGLE_BUTTON(Compression_fine_Radio), "toggled", G_CALLBACK (evt_compression_changed), all_data);

	row++;

	//Label "Capture Mode"
	label_Capture_mode=gtk_label_new(_("Capture Mode :"));
	gtk_misc_set_alignment (GTK_MISC (label_Capture_mode), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Capture_mode, 1, row, 1, 1);
	gtk_widget_show (label_Capture_mode);

	Capture_single_Radio = gtk_radio_button_new_with_label (NULL, _("Single"));
	Capture_burst_Radio = gtk_radio_button_new_with_label (NULL, _("Burst"));
	gtk_radio_button_join_group((GtkRadioButton*)Capture_single_Radio,(GtkRadioButton*)Capture_burst_Radio);
	gtk_grid_attach (GTK_GRID(table), Capture_single_Radio, 2, row, 1, 1);
	gtk_widget_show (Capture_single_Radio);

	row++;

	gtk_grid_attach (GTK_GRID(table), Capture_burst_Radio, 2, row, 1, 1);
	gtk_widget_show (Capture_burst_Radio);
if(TRUE == g_capture_mode){
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(Capture_burst_Radio),TRUE);
}else{
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(Capture_single_Radio),TRUE);
}
	g_signal_connect (GTK_TOGGLE_BUTTON(Capture_burst_Radio), "toggled", G_CALLBACK (evt_capture_mode_changed), all_data);

	row++;

	//Label "Burst Size"
	label_Burst_size=gtk_label_new(_("Burst Size :"));
	gtk_misc_set_alignment (GTK_MISC (label_Burst_size), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table), label_Burst_size, 1, row, 1, 1);
	gtk_widget_show (label_Burst_size);

	//Combo box for burst size
	combo_Burst_size = gtk_combo_box_text_new ();
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Burst_size),"2");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Burst_size),"3");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Burst_size),"4");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Burst_size),"5");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Burst_size),"6");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Burst_size),"7");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Burst_size),"8");
	gtk_widget_set_halign(combo_Burst_size,GTK_ALIGN_FILL);
	gtk_widget_set_hexpand(combo_Burst_size,TRUE);
	gtk_grid_attach(GTK_GRID(table),combo_Burst_size,2,row,1,1);

	gtk_widget_show (combo_Burst_size);
	printf("burst size : %d \n",g_burst_size);
	gtk_combo_box_set_active(GTK_COMBO_BOX(combo_Burst_size),g_burst_size);

if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(Capture_single_Radio)))
	gtk_widget_set_sensitive(GTK_WIDGET(combo_Burst_size),TRUE);
else
	gtk_widget_set_sensitive(GTK_WIDGET(combo_Burst_size),FALSE);

	g_signal_connect (GTK_COMBO_BOX_TEXT(combo_Burst_size), "changed", G_CALLBACK (evt_burst_size_changed), all_data);

	gtk_container_add (GTK_CONTAINER (content_area), table);
	gtk_dialog_add_buttons (GTK_DIALOG (capture_properties_dialog),
		"Capture",
		GTK_RESPONSE_OK,
		NULL);
	gtk_dialog_add_buttons (GTK_DIALOG (capture_properties_dialog),
		_("_Cancel"),
		GTK_RESPONSE_CANCEL,
		NULL);

	g_signal_connect (GTK_BUTTON(gtk_dialog_get_widget_for_response (GTK_DIALOG(capture_properties_dialog),GTK_RESPONSE_OK)),"clicked",G_CALLBACK(evt_z10x_capture),all_data);
	gtk_window_set_title(GTK_WINDOW (capture_properties_dialog),"See3CAM Capture Properties");
	gtk_widget_show_all (capture_properties_dialog);

	gtk_dialog_run (GTK_DIALOG (capture_properties_dialog));
	gtk_widget_destroy(capture_properties_dialog);

	return;
}

void set_global_data(struct ALL_DATA *all_data)
{
	struct GLOBAL *global = all_data->global;
	global->FX320_scene_mode=1;
	global->FX320_focus_mode=1;
	global->FX320_focus_position=0;
	global->FX320_brightness=8;
	global->FX320_contrast=8;
	global->FX320_shutter_speed=0;
	global->FX320_orientation=1;
	global->FX320_optical_zoom=0;
	global->FX320_digital_zoom=0;
	global->FX320_wb_mode=1;
	global->FX320_red_gain=0;
	global->FX320_blue_gain=0;
	global->FX320_temp_noise_filter=0;
	global->FX320_compression=0;

	return;
}

void evt_DefaultsButt_clicked(GtkButton *DefaultsButt, struct ALL_DATA *all_data)
{
	BOOL ret = FALSE;
	UINT8 status = 0 ;
	struct GLOBAL *global = all_data->global;
	global->FX320_defaults_flag=1;	
	ret = RestoreFactorySettings (&status);
	if(FALSE == ret )
	{
		printf("\nRestoreFactorySettings API Failed\n");
	} else {
		if(FX320_ACK == status){
			printf("\nsuccess\n");
			set_all_controls_defaults(all_data);
			set_global_data(all_data);
		} else if(FX320_NACK == status)
			printf("\nfail\n");
		else if(FX320_T_OUT == status)
			printf("\ntimeout\n");
		else
			printf("\nunknown return\n");
	}
	global->FX320_defaults_flag=0;
	ret = SendCameraCommand(SET_COMP_RATIO,1,1,&status);	
	if(FALSE == ret )
	{
		printf("\nSetting compression ratio default Failed\n");
	} else {
		if(FX320_ACK == status){
			global->FX320_compression=1;
			printf("\nsuccess in setting compression\n");
		} else if(FX320_NACK == status)
			printf("\nfail\n");
		else if(FX320_T_OUT == status)
			printf("\ntimeout\n");
		else
			printf("\nunknown return\n");
	}
	return;
}

void DrawZ10XControl(GtkWidget* table4,struct ALL_DATA *all_data)
{
	GtkWidget *label_Basic_title;
	GtkWidget *label_Orientation;
	GtkWidget *label_Scene_mode;
	GtkWidget *label_Focus_mode;
	GtkWidget *label_Focus_position;
	GtkWidget *label_Brightness;
	GtkWidget *label_Contrast;
	GtkWidget *label_Shutter_speed;
	GtkWidget *label_Zoom_title;
	GtkWidget *label_Optical_zoom;
	GtkWidget *label_Digital_zoom;

	GtkAdjustment *Focus_SliderAdjust;
	GtkAdjustment *Brightness_SliderAdjust;
	GtkAdjustment *Contrast_SliderAdjust;
	GtkAdjustment *Shutter_speed_SliderAdjust;
	GtkAdjustment *Optical_zoom_SliderAdjust;
	GtkAdjustment *Digital_zoom_SliderAdjust;


	GtkWidget *HButtonBox1;

	GtkWidget *AdvancedButton_Img;
	GtkWidget *CaptureButton_Img;
	GtkWidget *FwVersionButton_Img;
	GtkWidget *AboutButton_Img;
	GtkWidget *DefaultsButton_Img;

	UINT8 row = 0;
	struct GLOBAL *global = all_data->global;

	row++;

	//Label "Basic Camera Controls"
	label_Basic_title=gtk_label_new(_("---- Basic Camera Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_Basic_title), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Basic_title, 1, row, 4, 1);
	gtk_widget_show (label_Basic_title);

	row++;

	//Label "Scene Mode"
	label_Scene_mode=gtk_label_new(_("Scene Mode :"));
	gtk_misc_set_alignment (GTK_MISC (label_Scene_mode), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Scene_mode, 1, row, 1, 1);
	gtk_widget_show (label_Scene_mode);

	//Radio buttons for scene mode selection
	Scene_normal_Radio = gtk_radio_button_new_with_label (NULL, _("Normal"));
	Scene_macro_Radio = gtk_radio_button_new_with_label (NULL, _("Macro"));
	gtk_radio_button_join_group((GtkRadioButton*)Scene_normal_Radio,(GtkRadioButton*)Scene_macro_Radio);
	gtk_grid_attach (GTK_GRID(table4), Scene_normal_Radio, 2, row, 2, 1);
	gtk_widget_show (Scene_normal_Radio);

	row++;

	gtk_grid_attach (GTK_GRID(table4), Scene_macro_Radio, 2, row, 2, 1);
	gtk_widget_show (Scene_macro_Radio);
if(1==global->FX320_scene_mode)
	gtk_toggle_button_set_active((GtkToggleButton*)Scene_normal_Radio,TRUE);
else if(2==global->FX320_scene_mode)
	gtk_toggle_button_set_active((GtkToggleButton*)Scene_macro_Radio,TRUE);
	g_signal_connect(GTK_CHECK_BUTTON(Scene_macro_Radio), "toggled",G_CALLBACK (evt_fx320_scene_changed), all_data);

	row++;

	//Label "Focus Mode"
	label_Focus_mode=gtk_label_new(_("Focus Mode :"));
	gtk_misc_set_alignment (GTK_MISC (label_Focus_mode), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Focus_mode, 1, row, 1, 1);
	gtk_widget_show (label_Focus_mode);

	//Combo box for selecting Focus type
	combo_Focus = gtk_combo_box_text_new ();
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Focus),"Continous Auto Focus");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Focus),"Single Trigger Focus");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_Focus),"Manual Focus");
	gtk_widget_set_halign(combo_Focus,GTK_ALIGN_FILL);
	gtk_widget_set_hexpand(combo_Focus,TRUE);
	gtk_grid_attach(GTK_GRID(table4),combo_Focus,2,row,2,1);
	gtk_widget_show (combo_Focus);

	//Button for Focus
	FocusButt = gtk_button_new_with_label (_("Focus"));
	gtk_grid_attach(GTK_GRID(table4),FocusButt,4,row,1,1);
	gtk_widget_show(FocusButt);
	g_signal_connect (GTK_BUTTON(FocusButt), "clicked", G_CALLBACK (evt_fx320_focus_button_clicked), all_data);

	row++;

	//Label "Focus Position"
	label_Focus_position=gtk_label_new(_("Focus Position :"));
	gtk_misc_set_alignment (GTK_MISC (label_Focus_position), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Focus_position, 1, row, 1, 1);
	gtk_widget_show (label_Focus_position);

	//Slider for setting manual focus
	Focus_SliderAdjust=gtk_adjustment_new(0,0,256,1,1,1);
	Focus_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Focus_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Focus_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Focus_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), Focus_Slider, 2, row, 2, 1);
	gtk_widget_show (Focus_Slider);

	//Spin button for manual focus slider
	Focus_Spin=gtk_spin_button_new_with_range(0,255,1);
	gtk_editable_set_editable(GTK_EDITABLE(Focus_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Focus_Spin, 4, row, 1, 1);
	gtk_widget_show (Focus_Spin);
if(0<=global->FX320_focus_position && 255>global->FX320_focus_position) {
	gtk_range_set_value(GTK_RANGE(Focus_Slider),global->FX320_focus_position);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Focus_Spin), global->FX320_focus_position);
}
	g_signal_connect (GTK_SCALE(Focus_Slider), "button-press-event",G_CALLBACK (evt_fx320_manual_slider_pressed), all_data);
	g_signal_connect (GTK_SCALE(Focus_Slider), "key-press-event",G_CALLBACK (evt_fx320_manual_slider_pressed), all_data);
	g_signal_connect (GTK_SCALE(Focus_Slider), "button-release-event",G_CALLBACK (evt_fx320_manual_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Focus_Slider), "key-release-event",G_CALLBACK (evt_fx320_manual_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Focus_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

if(1==global->FX320_focus_mode) {
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Spin,FALSE);
	gtk_combo_box_set_active(GTK_COMBO_BOX(combo_Focus),0);
} else if(2==global->FX320_focus_mode) {
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Spin,FALSE);
	gtk_combo_box_set_active(GTK_COMBO_BOX(combo_Focus),1);
} else if(3==global->FX320_focus_mode) {
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Focus_Spin,TRUE);
	gtk_combo_box_set_active(GTK_COMBO_BOX(combo_Focus),2);
}
	g_signal_connect(GTK_COMBO_BOX_TEXT(combo_Focus), "changed",G_CALLBACK (evt_fx320_Focus_changed), all_data);

	row++;

	//Label "Brightness"
	label_Brightness=gtk_label_new(_("Brightness :"));
	gtk_misc_set_alignment (GTK_MISC (label_Brightness), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Brightness, 1, row, 1, 1);
	gtk_widget_show (label_Brightness);

	//Slider for setting brightness
	Brightness_SliderAdjust=gtk_adjustment_new(0,0,16,1,1,1);
	Brightness_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Brightness_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Brightness_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Brightness_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), Brightness_Slider, 2, row, 2, 1);
	gtk_widget_show (Brightness_Slider);

	//Spin button for brightness
	Brightness_Spin=gtk_spin_button_new_with_range(0,15,1);
	gtk_editable_set_editable(GTK_EDITABLE(Brightness_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Brightness_Spin, 4, row, 1, 1);
	gtk_widget_show (Brightness_Spin);
if(0<=global->FX320_brightness && 16>global->FX320_brightness) { 
	printf("\nfx320_brightness = %d\n", global->FX320_brightness);
	gtk_range_set_value(GTK_RANGE(Brightness_Slider),global->FX320_brightness);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Brightness_Spin), global->FX320_brightness);
}else
printf("\nfx320_brightness = %d\n", global->FX320_brightness);
	g_signal_connect (GTK_SCALE(Brightness_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Brightness_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Brightness_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

	row++;

	//Label "Contrast"
	label_Contrast=gtk_label_new(_("Contrast :"));
	gtk_misc_set_alignment (GTK_MISC (label_Contrast), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Contrast, 1, row, 1, 1);
	gtk_widget_show (label_Contrast);

	//Slider for setting contrast
	Contrast_SliderAdjust=gtk_adjustment_new(0,0,16,1,1,1);
	Contrast_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Contrast_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Contrast_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Contrast_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), Contrast_Slider, 2, row, 2, 1);
	gtk_widget_show (Contrast_Slider);

	//Spin button for contrast
	Contrast_Spin=gtk_spin_button_new_with_range(0,15,1);
	gtk_editable_set_editable(GTK_EDITABLE(Contrast_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Contrast_Spin, 4, row, 1, 1);
	gtk_widget_show (Contrast_Spin);
if(0<=global->FX320_contrast && 16>global->FX320_contrast) { 
	gtk_range_set_value(GTK_RANGE(Contrast_Slider),global->FX320_contrast);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Contrast_Spin), global->FX320_contrast);
}
	g_signal_connect (GTK_SCALE(Contrast_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Contrast_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Contrast_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

	row++;

	//Label "Shutter Speed"
	label_Shutter_speed=gtk_label_new(_("Shutter Speed :"));
	gtk_misc_set_alignment (GTK_MISC (label_Shutter_speed), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Shutter_speed, 1, row, 1, 1);
	gtk_widget_show (label_Shutter_speed);

	//Slider for setting shutter speed
	Shutter_speed_SliderAdjust=gtk_adjustment_new(0,0,10,1,1,1);
	Shutter_speed_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Shutter_speed_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Shutter_speed_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Shutter_speed_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), Shutter_speed_Slider, 2, row, 2, 1);
	gtk_widget_show (Shutter_speed_Slider);

	//Spin button for shutter speed
	Shutter_speed_Spin=gtk_spin_button_new_with_range(0,9,1);
	gtk_editable_set_editable(GTK_EDITABLE(Shutter_speed_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Shutter_speed_Spin, 4, row, 1, 1);
	gtk_widget_show (Shutter_speed_Spin);
if(0<=global->FX320_shutter_speed && 10>global->FX320_shutter_speed) { 
	gtk_range_set_value(GTK_RANGE(Shutter_speed_Slider),global->FX320_shutter_speed);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Shutter_speed_Spin), global->FX320_shutter_speed);
}
	g_signal_connect (GTK_SCALE(Shutter_speed_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Shutter_speed_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Shutter_speed_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

	row++;

	//Label "Orientation"
	label_Orientation=gtk_label_new(_("Orientation :"));
	gtk_misc_set_alignment (GTK_MISC (label_Orientation), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Orientation, 1, row, 1, 1);
	gtk_widget_show (label_Orientation);

	//Check buttons for Flip and Mirror
	check_Flip=gtk_check_button_new_with_label(_("Flip"));
	gtk_widget_set_halign (check_Flip, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (check_Flip, TRUE);
	gtk_grid_attach (GTK_GRID(table4), check_Flip, 2, row, 1, 1);
	gtk_widget_show (check_Flip);

	check_Mirror=gtk_check_button_new_with_label(_("Mirror"));
	gtk_widget_set_halign (check_Mirror, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (check_Mirror, TRUE);
	gtk_grid_attach (GTK_GRID(table4), check_Mirror, 3, row, 1, 1);
	gtk_widget_show (check_Mirror);
if(1<=global->FX320_orientation && 5>global->FX320_orientation) {
	printf("\nfx320_orientation = %d\n", global->FX320_orientation);
			if(1==global->FX320_orientation) {
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Flip),FALSE);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Mirror),FALSE);
			} else if(2==global->FX320_orientation) {
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Flip),FALSE);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Mirror),TRUE);
			} else if(3==global->FX320_orientation) {
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Flip),TRUE);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Mirror),FALSE);
			} else if(4==global->FX320_orientation) {
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Flip),TRUE);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Mirror),TRUE);
			}
} 
	g_signal_connect (GTK_BUTTON(check_Flip), "toggled",G_CALLBACK (evt_fx320_check_button_toggled), all_data);
	g_signal_connect (GTK_BUTTON(check_Mirror), "toggled",G_CALLBACK (evt_fx320_check_button_toggled), all_data);

	row++;

	//Label "Zoom Controls"
	label_Zoom_title=gtk_label_new(_("---- Zoom Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_Zoom_title), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Zoom_title, 1, row, 4, 1);
	gtk_widget_show (label_Zoom_title);

	row++;

	//Label "Optical Zoom"
	label_Optical_zoom=gtk_label_new(_("Optical Zoom :"));
	gtk_misc_set_alignment (GTK_MISC (label_Optical_zoom), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Optical_zoom, 1, row, 1, 1);
	gtk_widget_show (label_Optical_zoom);

	//Slider for setting optical zoom
	Optical_zoom_SliderAdjust=gtk_adjustment_new(0,0,44,1,1,1);
	Optical_zoom_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Optical_zoom_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Optical_zoom_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Optical_zoom_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), Optical_zoom_Slider, 2, row, 2, 1);
	gtk_widget_show (Optical_zoom_Slider);

	//Spin button for optical zoom
	Optical_zoom_Spin=gtk_spin_button_new_with_range(0,43,1);
	gtk_editable_set_editable(GTK_EDITABLE(Optical_zoom_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Optical_zoom_Spin, 4, row, 1, 1);
	gtk_widget_show (Optical_zoom_Spin);

if(0<=global->FX320_optical_zoom && 44>global->FX320_optical_zoom) {
	gtk_range_set_value(GTK_RANGE(Optical_zoom_Slider),global->FX320_optical_zoom);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Optical_zoom_Spin), global->FX320_optical_zoom);
}
	g_signal_connect (GTK_SCALE(Optical_zoom_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Optical_zoom_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Optical_zoom_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

	row++;

	//Label "Digital Zoom"
	label_Digital_zoom=gtk_label_new(_("Digital Zoom :"));
	gtk_misc_set_alignment (GTK_MISC (label_Digital_zoom), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Digital_zoom, 1, row, 1, 1);
	gtk_widget_show (label_Digital_zoom);

	//Slider for setting digital zoom
	Digital_zoom_SliderAdjust=gtk_adjustment_new(0,0,16,1,1,1);
	Digital_zoom_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,Digital_zoom_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Digital_zoom_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Digital_zoom_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), Digital_zoom_Slider, 2, row, 2, 1);
	gtk_widget_show (Digital_zoom_Slider);

	//Spin button for digital zoom
	Digital_zoom_Spin=gtk_spin_button_new_with_range(0,15,1);
	gtk_editable_set_editable(GTK_EDITABLE(Digital_zoom_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Digital_zoom_Spin, 4, row, 1, 1);
	gtk_widget_show (Digital_zoom_Spin);
if(0<=global->FX320_digital_zoom && 16>global->FX320_digital_zoom) {
	gtk_range_set_value(GTK_RANGE(Digital_zoom_Slider),global->FX320_digital_zoom);
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Digital_zoom_Spin), global->FX320_digital_zoom);
}
	g_signal_connect (GTK_SCALE(Digital_zoom_Slider), "button-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Digital_zoom_Slider), "key-release-event",G_CALLBACK (evt_fx320_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Digital_zoom_Slider), "value-changed",G_CALLBACK (evt_z10x_update_spin), all_data);

	row++;

	/*------------------------------ Add Buttons ------------------------------*/

	HButtonBox1 = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
        gtk_widget_set_halign (HButtonBox1, GTK_ALIGN_FILL);
		gtk_widget_set_hexpand (HButtonBox1, TRUE);
        gtk_button_box_set_layout(GTK_BUTTON_BOX(HButtonBox1),GTK_BUTTONBOX_SPREAD);
        gtk_box_set_homogeneous(GTK_BOX(HButtonBox1),TRUE);

	GtkWidget *AdvancedButt = gtk_button_new_with_label (_("Advanced"));
	GtkWidget *CaptureButt = gtk_button_new_with_label (_("Capture"));
	GtkWidget *FwVersionButt = gtk_button_new_with_label (_("F/W Verison"));
	GtkWidget *AboutButt = gtk_button_new_with_label (_("About"));
	GtkWidget *DefaultsButt = gtk_button_new_with_label (_("Defaults"));

	gtk_box_pack_start (GTK_BOX (HButtonBox1), AdvancedButt, TRUE, TRUE, 1);
	gtk_box_pack_start (GTK_BOX (HButtonBox1), CaptureButt, TRUE, TRUE, 1);
	gtk_box_pack_start (GTK_BOX (HButtonBox1), FwVersionButt, TRUE, TRUE, 1);
	gtk_box_pack_start (GTK_BOX (HButtonBox1), AboutButt, TRUE, TRUE, 1);
	gtk_box_pack_start (GTK_BOX (HButtonBox1), DefaultsButt, TRUE, TRUE, 1);

        gchar* advanced_icon_path = g_strconcat (PACKAGE_DATA_DIR,"/pixmaps/guvcview/advanced.png",NULL);
        if (g_file_test(advanced_icon_path,G_FILE_TEST_EXISTS))
        {
                AdvancedButton_Img = gtk_image_new_from_file (advanced_icon_path);
                gtk_button_set_image(GTK_BUTTON(AdvancedButt),AdvancedButton_Img);
                gtk_button_set_image_position(GTK_BUTTON(AdvancedButt),GTK_POS_TOP);
	}
	g_free(advanced_icon_path);
        gchar* capture_icon_path = g_strconcat (PACKAGE_DATA_DIR,"/pixmaps/guvcview/capture.png",NULL);
        if (g_file_test(capture_icon_path,G_FILE_TEST_EXISTS))
        {
                CaptureButton_Img = gtk_image_new_from_file (capture_icon_path);
                gtk_button_set_image(GTK_BUTTON(CaptureButt),CaptureButton_Img);
                gtk_button_set_image_position(GTK_BUTTON(CaptureButt),GTK_POS_TOP);
	}
	g_free(capture_icon_path);
        gchar* fw_version_icon_path = g_strconcat (PACKAGE_DATA_DIR,"/pixmaps/guvcview/fw_version.png",NULL);
        if (g_file_test(fw_version_icon_path,G_FILE_TEST_EXISTS))
        {
                FwVersionButton_Img = gtk_image_new_from_file (fw_version_icon_path);
                gtk_button_set_image(GTK_BUTTON(FwVersionButt),FwVersionButton_Img);
                gtk_button_set_image_position(GTK_BUTTON(FwVersionButt),GTK_POS_TOP);
	}
	g_free(fw_version_icon_path);
        gchar* about_icon_path = g_strconcat (PACKAGE_DATA_DIR,"/pixmaps/guvcview/about.png",NULL);
        if (g_file_test(about_icon_path,G_FILE_TEST_EXISTS))
        {
                AboutButton_Img = gtk_image_new_from_file (about_icon_path);
                gtk_button_set_image(GTK_BUTTON(AboutButt),AboutButton_Img);
                gtk_button_set_image_position(GTK_BUTTON(AboutButt),GTK_POS_TOP);
	}
	g_free(about_icon_path);
        gchar* defaults_icon_path = g_strconcat (PACKAGE_DATA_DIR,"/pixmaps/guvcview/defaults.png",NULL);
        if (g_file_test(defaults_icon_path,G_FILE_TEST_EXISTS))
        {
                DefaultsButton_Img = gtk_image_new_from_file (defaults_icon_path);
                gtk_button_set_image(GTK_BUTTON(DefaultsButt),DefaultsButton_Img);
                gtk_button_set_image_position(GTK_BUTTON(DefaultsButt),GTK_POS_TOP);
	}
	g_free(defaults_icon_path);
	gtk_widget_show_all (AdvancedButt);
	gtk_widget_show_all (CaptureButt);
	gtk_widget_show_all (FwVersionButt);
	gtk_widget_show_all (AboutButt);
	gtk_widget_show_all (DefaultsButt);

	g_signal_connect (GTK_BUTTON(AdvancedButt), "clicked", G_CALLBACK (ShowAdvancedControlsWindow), all_data);
	g_signal_connect (GTK_BUTTON(CaptureButt), "clicked", G_CALLBACK (ShowCapturePropertiesWindow), all_data);
	g_signal_connect (GTK_BUTTON(FwVersionButt), "clicked", G_CALLBACK (ShowFwVersion), all_data);
	g_signal_connect (GTK_BUTTON(AboutButt), "clicked", G_CALLBACK (ShowAbout), all_data);
	g_signal_connect (GTK_BUTTON(DefaultsButt), "clicked", G_CALLBACK (evt_DefaultsButt_clicked), all_data);

	gtk_grid_attach (GTK_GRID(table4), HButtonBox1, 1, row, 4, 20);

	gtk_widget_show_all (HButtonBox1);

	return;
}
