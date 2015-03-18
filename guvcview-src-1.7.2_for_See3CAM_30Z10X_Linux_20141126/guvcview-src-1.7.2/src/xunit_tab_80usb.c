
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

GtkWidget *Focus;
GtkWidget *FocusButt;
GtkWidget *PositionButt;
GtkWidget *Slider;
GtkAdjustment *SliderAdjust;
GtkWidget *Spin;
GtkWidget *Entry_Position;
GtkWidget *check_Flash;
GtkWidget *check_Torch;
GtkWidget *GPIO_In;
GtkWidget *GPIO_UpdateButt;
GtkWidget *GPIO_Radio1;
GtkWidget *GPIO_Radio2;
GtkWidget *label_GPIO_in_status;

UINT16 g_focus_value=0;
UINT8 g_combobox_choice;

void evt_Focus_changed(GtkComboBox *Focus, struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	UINT8 status=0;
	g_combobox_choice=gtk_combo_box_get_active(Focus);
	switch(g_combobox_choice)
	{
		case 0:
		{
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Spin,FALSE);
			ret=SetFocusMode(CONTINOUS_FOCUS,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting Focus",all_data);
			} else {
				if(status == FOCUS_FAILED) {
					ShowErrorDialog("Failed to set focus",all_data);
					return;
				} else if (status == FOCUS_BUSY) {
					ShowErrorDialog("Camera is busy",all_data);
					return;
				} else {
					printf("Auto Focus mode set successfully\n");
				}
			} 
			break;
		}
		case 1:
		{
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Spin,TRUE);
			g_focus_value=gtk_range_get_value((GtkRange*)Slider);
			ret=SetFocusPosition(g_focus_value);
			if(ret == FALSE)
			{
				ShowErrorDialog("Error in setting Focus position",all_data);
				return;
			}
			ret=SetFocusMode(MANUAL_FOCUS,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting Focus",all_data);
			} else {
				if(status == FOCUS_FAILED) {
					ShowErrorDialog("Failed to set focus",all_data);
					return;
				} else if (status == FOCUS_BUSY) {
					ShowErrorDialog("Camera is busy",all_data);
					return;
				} else {
					printf("Manual Focus mode set successfully\n");
				}
			}	
			break;
		}
		case 2:
		{
			gtk_widget_set_sensitive((GtkWidget*)FocusButt,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)Spin,FALSE);
			ret=SetFocusMode(SINGLE_TRIG_FOCUS,&status);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting Focus",all_data);
			} else {
				if(status == FOCUS_FAILED) {
					ShowErrorDialog("Failed to set focus",all_data);
					return;
				} else if (status == FOCUS_BUSY) {
					ShowErrorDialog("Camera is busy",all_data);
					return;
				} else {
					printf("Single trigger focus set successfully\n");
				}
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

void evt_Get_Position(GtkButton *PositionButt, struct ALL_DATA *all_data)
{
	//get position and display in Entry_Position box
	BOOL ret =FALSE;
	UINT16 value=0;
	char *position_text;
	ret = GetFocusPosition(&value);
	position_text = g_strdup_printf ("%u",value);
	gtk_entry_set_text((GtkEntry*)Entry_Position,position_text);
	g_free(position_text);
	if(ret == FALSE)
	{
		ShowErrorDialog("Error in getting Focus Position",all_data);
		return;
	}
	return;
}

void evt_focus_button_clicked(GtkButton *FocusButt, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 status=0;
	ret=SetFocusMode(SINGLE_TRIG_FOCUS,&status);
	if(ret == FALSE) {
		ShowErrorDialog("Error in setting Focus",all_data);
	} else {
		if(status == FOCUS_FAILED) {
			ShowErrorDialog("Failed to set focus",all_data);
		} else if (status == FOCUS_BUSY) {
			ShowErrorDialog("Camera is busy",all_data);
		} else {
			printf("Single trigger focus set successfully\n");
		}
	}	
	return;
		
}
void evt_80_GPIO_radio_changed(GtkRadioButton *GPIO_Radio1, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 GpioValue = 0;
	GpioValue=(gtk_toggle_button_get_active((GtkToggleButton*)GPIO_Radio1) ? 1 : 0);
	ret = SetGpioLevel(GPIO_80_OUT,GpioValue);
	if(ret == FALSE) {
		ShowErrorDialog("Error in setting GPIO level",all_data);
		return;
	}
	return;
}

void evt_80_GPIO_button_clicked(GtkButton *GPIO_UpdateButt, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 GpioValue = 0;
	g_combobox_choice=gtk_combo_box_get_active((GtkComboBox*)GPIO_In);
	if(g_combobox_choice==0) {
		ret = GetGpioLevel(GPIO_80_IN1,&GpioValue);
	} else {
		ret = GetGpioLevel(GPIO_80_IN2,&GpioValue);
	}
	if(ret == FALSE) {
		ShowErrorDialog("Error in getting GPIO level",all_data);
		return;
	}
	if(GpioValue == 1 ) {
		gtk_label_set_markup(GTK_LABEL (label_GPIO_in_status), "<span foreground = \"blue\">HIGH</span>");
	} else {
		gtk_label_set_markup(GTK_LABEL (label_GPIO_in_status), "<span foreground = \"red\">LOW</span>");
	}
	return;	
}

void evt_80_flash_toggled(GtkWidget *Widget, struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	g_flash_flag = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(Widget)) ? TRUE : FALSE;
	ret=SetFlashLevel(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(Widget)) ? 1 : 0);
	if(ret == FALSE)
	{
		ShowErrorDialog("Error in setting Flash Mode",all_data);
		return;
	}
	return;
}
void evt_80_torch_toggled(GtkWidget *Widget, struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	ret=SetTorchLevel(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(Widget)) ? 1 : 0);
	if(ret == FALSE)
	{
		ShowErrorDialog("Error in setting Torch Mode",all_data);
		return;
	}
	return;
}
gboolean evt_slider_released(GtkRange *Slider,GdkEvent *event,struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	UINT8 status=0;
	g_focus_value=gtk_range_get_value((GtkRange*)Slider);
	ret=SetFocusPosition(g_focus_value);
	if(ret == FALSE)
	{
		ShowErrorDialog("Error in setting focus position",all_data);
		return FALSE;
	}
	else
	{	
		ret=SetFocusMode(MANUAL_FOCUS,&status);
		if(ret == FALSE) {
			ShowErrorDialog("Error in setting Focus",all_data);
		} else {
			if(status == FOCUS_FAILED) {
				ShowErrorDialog("Failed to set focus",all_data);
			} else if (status == FOCUS_BUSY) {
				ShowErrorDialog("Camera is busy",all_data);
			} else {
				printf("Manual Focus mode set successfully\n");
			}
		}
	}
	return FALSE;
}
#if 0
gboolean evt_spin_released(GtkSpinButton *Spin,GdkEvent *event,struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	UINT8 status=0;
	g_focus_value=gtk_spin_button_get_value_as_int((GtkSpinButton*)Spin);
	ret=SetFocusPosition(g_focus_value);
	if(ret == FALSE)
	{
		ShowErrorDialog("Error in setting focus position",all_data);
		return FALSE;
	}
	else
	{	
		ret=SetFocusMode(MANUAL_FOCUS,&status);
		if(ret == FALSE) {
			ShowErrorDialog("Error in setting Focus",all_data);
		} else {
			if(status == FOCUS_FAILED) {
				ShowErrorDialog("Failed to set focus",all_data);
			} else if (status == FOCUS_BUSY) {
				ShowErrorDialog("Camera is busy",all_data);
			} else {
				printf("Manual Focus mode set successfully\n");
			}
		}
	}
	return FALSE;
}
#endif
void evt_update_spin(GtkRange *Slider,struct ALL_DATA *all_data)
{
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Spin), gtk_range_get_value((GtkRange*)Slider));
	return;
}
#if 0
static void evt_update_slider(GtkRange *Spin,struct ALL_DATA *all_data)
{
	gtk_range_set_value ((GtkRange*)Slider, gtk_spin_button_get_value_as_int((GtkSpinButton*)Spin));
	return;
}
#endif

void Draw80USBControl(GtkWidget* table4,struct ALL_DATA *all_data)
{
	GtkWidget *HButtonBox2;
	GtkWidget *label_Focus;
	GtkWidget *label_focus_title;
	GtkWidget *label_Position;
	GtkWidget *label_GPIO_controls;
	GtkWidget *label_GPIO_out_level;
	GtkWidget *label_GPIO_in_no;
	GtkWidget *label_GPIO_in_level;
	GtkWidget *label_flash_controls;
	GtkWidget *label_other;

	UINT8 GpioValue = 0, focus_mode = 0, flash_level = 0, torch_level=0;
	UINT16 value = 0;
	UINT8 row = 0;
	BOOL ret = FALSE;

	row++;

	//label "Focus controls"
	label_focus_title=gtk_label_new(_("---- Focus Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_focus_title), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_focus_title, 1, row, 3, 1);
	gtk_widget_show (label_focus_title);

	row++;

	//Label "Focus"
	label_Focus=gtk_label_new(_("Focus Mode :"));
	gtk_misc_set_alignment (GTK_MISC (label_Focus), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Focus, 1, row, 1, 1);
	gtk_widget_show (label_Focus);

	//Combo box for selecting Focus type
	Focus = gtk_combo_box_text_new ();

	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(Focus),"Auto Focus");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(Focus),"Manual Focus");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(Focus),"Single Trigger Focus");
	gtk_widget_set_halign(Focus,GTK_ALIGN_FILL);
	gtk_widget_set_hexpand(Focus,TRUE);
	gtk_grid_attach(GTK_GRID(table4),Focus,2,row,1,1);
	gtk_widget_show(Focus);
	ret=GetFocusMode(&focus_mode);
	if( ret == TRUE )
	{
		switch(focus_mode)
		{
			case CONTINOUS_FOCUS :
			{
				printf("camera in continous focus mode\n");
				break;
			}
			case MANUAL_FOCUS :
			{
				printf("camera in manual focus mode\n");
				break;
			}
			case SINGLE_TRIG_FOCUS :
			{
				printf("camera in single trigger focus mode\n");
				break;
			}
			default :
			{
				printf("Unspecified mode error\n");
			}
		}
		gtk_combo_box_set_active(GTK_COMBO_BOX(Focus),focus_mode-1);
	}
	else
	{
		printf("Error in mode selection.");
	}
	g_signal_connect(GTK_COMBO_BOX_TEXT(Focus), "changed",G_CALLBACK (evt_Focus_changed), all_data);

	//Button for Focus
	FocusButt = gtk_button_new_with_label (_("Focus"));
	gtk_grid_attach(GTK_GRID(table4),FocusButt,3,row,1,1);
	gtk_widget_show(FocusButt);
	g_signal_connect (GTK_BUTTON(FocusButt), "clicked", G_CALLBACK (evt_focus_button_clicked), all_data);

	row++;

	//Label "Focus Position"
	label_Position=gtk_label_new(_("Focus position :"));
	gtk_misc_set_alignment (GTK_MISC (label_Position), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Position, 1, row, 1, 1);
	gtk_widget_show (label_Position);

	//Slider for setting manual focus
	SliderAdjust=gtk_adjustment_new(0,0,1023,1,1,1);
	Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE (Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), Slider, 2, row, 1, 1);
	gtk_widget_show (Slider);

	//Spin button for manual focus slider
	Spin=gtk_spin_button_new_with_range(0,1022,1);
	gtk_editable_set_editable(GTK_EDITABLE(Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Spin, 3, row, 1, 1);
	gtk_widget_show (Spin);

	//Enable or disable controls according to focus mode of camera
	if(focus_mode)
	{
		switch(focus_mode)
		{
			case 1://auto focus
			{
				gtk_widget_set_sensitive(FocusButt,FALSE);
				gtk_widget_set_sensitive(Slider,FALSE);
				gtk_widget_set_sensitive(Spin,FALSE);
				break;
			}
			case 2://manual focus
			{
				gtk_widget_set_sensitive(FocusButt,FALSE);
				gtk_widget_set_sensitive(Slider,TRUE);
				gtk_widget_set_sensitive(Spin,TRUE);
				break;
			}
			case 3://single trigger focus
			{
				gtk_widget_set_sensitive(FocusButt,TRUE);
				gtk_widget_set_sensitive(Slider,FALSE);
				gtk_widget_set_sensitive(Spin,FALSE);
				break;
			}
			default :
			{
				printf("Error reading focus mode :");
				gtk_combo_box_set_active(GTK_COMBO_BOX(Focus),-1);
				break;
			}
		}
	}

	g_signal_connect (GTK_SCALE(Slider), "button-release-event",G_CALLBACK (evt_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Slider), "key-release-event",G_CALLBACK (evt_slider_released), all_data);
	g_signal_connect (GTK_SCALE(Slider), "value-changed",G_CALLBACK (evt_update_spin), all_data);
#if 0
	g_signal_connect (GTK_SPIN_BUTTON(Spin), "button-release-event",G_CALLBACK (evt_spin_released), all_data);
	g_signal_connect (GTK_SPIN_BUTTON(Spin), "key-release-event",G_CALLBACK (evt_spin_released), all_data);
	g_signal_connect (GTK_SPIN_BUTTON(Spin), "value-changed",G_CALLBACK (evt_update_slider), all_data);
#endif
	row++;

	//Label "Current Position"
	label_Position=gtk_label_new(_("Current position :"));
	gtk_misc_set_alignment (GTK_MISC (label_Position), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_Position, 1, row, 1, 1);
	gtk_widget_show (label_Position);

	//Text box to display motor position
	Entry_Position=gtk_entry_new();
	gtk_editable_set_editable(GTK_EDITABLE(Entry_Position),FALSE);
	gtk_grid_attach (GTK_GRID(table4), Entry_Position, 2, row, 1, 1);
	gtk_widget_show (Entry_Position);

	//Button for reading Motor position
	PositionButt=gtk_button_new_with_label(_("Get Focus Position"));
	gtk_grid_attach (GTK_GRID(table4), PositionButt, 3, row, 1, 1);
	gtk_widget_show (PositionButt);
	g_signal_connect(GTK_BUTTON(PositionButt),"clicked",G_CALLBACK(evt_Get_Position),all_data);

	//Get the Manual Focus position from the sensor and set it in the slider and entry box
	ret = GetFocusPosition(&value);
	if( ret == FALSE )
	{
		printf("Error in getting Focus position\n");
		gtk_range_set_value((GtkRange*)Slider, 1);
		gtk_entry_set_text((GtkEntry*)Entry_Position,"1");
	}
	else
	{
		gtk_range_set_value((GtkRange*)Slider, value);
		char *position_text;
		position_text = g_strdup_printf ("%u",value);
		gtk_entry_set_text((GtkEntry*)Entry_Position,position_text);
		g_free(position_text);
	}
	gtk_spin_button_set_value (GTK_SPIN_BUTTON(Spin), gtk_range_get_value((GtkRange*)Slider));

	row++;

	//label "GPIO controls"
	label_GPIO_controls=gtk_label_new(_("---- GPIO Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_GPIO_controls), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_GPIO_controls, 1, row, 3, 1);
	gtk_widget_show (label_GPIO_controls);

	row++;

	//Label "Output GPIO level"
	label_GPIO_out_level = gtk_label_new(_("GPO level :"));
	gtk_misc_set_alignment (GTK_MISC (label_GPIO_out_level), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_GPIO_out_level, 1, row, 1, 1);
	gtk_widget_show (label_GPIO_out_level);

	//Radio buttons for GPIO
	GPIO_Radio1 = gtk_radio_button_new_with_label (NULL, _("High"));
	GPIO_Radio2 = gtk_radio_button_new_with_label (NULL, _("Low"));
	gtk_radio_button_join_group((GtkRadioButton*)GPIO_Radio1,(GtkRadioButton*)GPIO_Radio2);
	gtk_grid_attach (GTK_GRID(table4), GPIO_Radio1, 2, row, 1, 1);
	gtk_widget_show (GPIO_Radio1);

	row++;

	gtk_grid_attach (GTK_GRID(table4), GPIO_Radio2, 2, row, 1, 1);
	gtk_widget_show (GPIO_Radio2);

	ret = GetGpioLevel(GPIO_80_OUT,&GpioValue);
	if(ret == FALSE )
	{		
		printf("Error in getting GPIO level \n");
		gtk_toggle_button_set_active((GtkToggleButton*)GPIO_Radio1,FALSE);
	}
	else
	{
		if(GpioValue == 1 ) {
			gtk_toggle_button_set_active((GtkToggleButton*)GPIO_Radio1,TRUE);
		} else {
			gtk_toggle_button_set_active((GtkToggleButton*)GPIO_Radio2,TRUE);
		}
	}
	g_signal_connect(GTK_CHECK_BUTTON(GPIO_Radio1), "toggled",G_CALLBACK (evt_80_GPIO_radio_changed), all_data);

	row++;

	//Label "Input GPIO number"
	label_GPIO_in_no = gtk_label_new(_("GPI number :"));
	gtk_misc_set_alignment (GTK_MISC (label_GPIO_in_no), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_GPIO_in_no, 1, row, 1, 1);
	gtk_widget_show (label_GPIO_in_no);

	//Combo box for selecting input GPIO
	GPIO_In = gtk_combo_box_text_new ();
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(GPIO_In),"1");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(GPIO_In),"2");
	gtk_combo_box_set_active(GTK_COMBO_BOX(GPIO_In),0);
	gtk_grid_attach (GTK_GRID(table4), GPIO_In, 2, row, 1, 1);
	gtk_widget_show (GPIO_In);

	g_signal_connect(GTK_COMBO_BOX_TEXT(GPIO_In), "changed",G_CALLBACK (evt_80_GPIO_button_clicked), all_data);

	row++;

	//Label for GPIO input level
	label_GPIO_in_level=gtk_label_new(_("GPI level :"));
	gtk_misc_set_alignment (GTK_MISC (label_GPIO_in_level), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_GPIO_in_level, 1, row, 1, 1);
	gtk_widget_show (label_GPIO_in_level);

	//Label for GPIO input status
	label_GPIO_in_status=gtk_label_new(_(""));
	gtk_misc_set_alignment (GTK_MISC (label_GPIO_in_status), 0, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_GPIO_in_status, 2, row, 1, 1);
	gtk_widget_show (label_GPIO_in_status);

	//Button for updating status of GPIO
	GPIO_UpdateButt = gtk_button_new_with_label (_("Get GPI Status"));
	gtk_grid_attach (GTK_GRID(table4), GPIO_UpdateButt, 3, row, 1, 1);
	gtk_widget_show (GPIO_UpdateButt);
	g_signal_connect(GTK_BUTTON(GPIO_UpdateButt), "clicked",G_CALLBACK (evt_80_GPIO_button_clicked), all_data);
	gtk_button_clicked (GTK_BUTTON(GPIO_UpdateButt));

	row++;

	//Label for Flash Controls
	label_flash_controls = gtk_label_new(_("---- Flash Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_flash_controls), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_flash_controls, 1, row, 3, 1);
	gtk_widget_show (label_flash_controls);

	row++;

	//Check buttons for Flash and Torch
	check_Flash=gtk_check_button_new_with_label(_("Flash"));
	gtk_widget_set_halign (check_Flash, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (check_Flash, TRUE);
	gtk_grid_attach (GTK_GRID(table4), check_Flash, 2, row, 1, 1);
	gtk_widget_show (check_Flash);

	row++;

	check_Torch=gtk_check_button_new_with_label(_("Torch"));
	gtk_widget_set_halign (check_Torch, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (check_Torch, TRUE);
	gtk_grid_attach (GTK_GRID(table4), check_Torch, 2, row, 1, 1);
	gtk_widget_show (check_Torch);

	ret = GetFlashLevel(&flash_level);
	if(ret == FALSE ) {
		printf("Error in getting flash level\n");
		//gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Flash),FALSE);
	} else {
		if(flash_level) {
			gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Flash),TRUE);
			g_flash_flag=TRUE;
		}
	}
	ret = GetTorchLevel(&torch_level);
	if(ret == FALSE ) {
		printf("Error in getting torch level\n");
		//gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Torch),FALSE);
	} else {
		if(torch_level)
			gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check_Torch),TRUE);
	}

	g_signal_connect (GTK_BUTTON(check_Flash), "toggled",G_CALLBACK (evt_80_flash_toggled), all_data);
	g_signal_connect (GTK_BUTTON(check_Torch), "toggled",G_CALLBACK (evt_80_torch_toggled), all_data);

	row++;

	//Label for Other Options
	label_other = gtk_label_new(_("---- Other Options ----"));
	gtk_misc_set_alignment (GTK_MISC (label_other), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_other, 1, row, 3, 1);
	gtk_widget_show (label_other);

	row++;

	//Buttons for F/W version and About
	GtkWidget *FwVersionButt = gtk_button_new_with_label (_("F/W Verison"));
	GtkWidget *AboutButt = gtk_button_new_with_label (_("About"));

	//Box for F/W version and About buttons
	HButtonBox2 = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
	gtk_widget_set_halign (HButtonBox2, GTK_ALIGN_FILL);
	gtk_widget_set_hexpand (HButtonBox2, TRUE);
	gtk_button_box_set_layout(GTK_BUTTON_BOX(HButtonBox2),GTK_BUTTONBOX_SPREAD);
	gtk_box_set_homogeneous(GTK_BOX(HButtonBox2), TRUE);
	gtk_box_pack_start (GTK_BOX (HButtonBox2), FwVersionButt, TRUE, TRUE, 0);
	gtk_box_pack_start (GTK_BOX (HButtonBox2), AboutButt, TRUE, TRUE, 0);
	gtk_grid_attach (GTK_GRID(table4), HButtonBox2, 1, row, 3, 1);
	gtk_widget_show_all(HButtonBox2);

	g_signal_connect (GTK_BUTTON(FwVersionButt), "clicked", G_CALLBACK (ShowFwVersion), all_data);
	g_signal_connect (GTK_BUTTON(AboutButt), "clicked", G_CALLBACK (ShowAbout), all_data);

	return;

}

