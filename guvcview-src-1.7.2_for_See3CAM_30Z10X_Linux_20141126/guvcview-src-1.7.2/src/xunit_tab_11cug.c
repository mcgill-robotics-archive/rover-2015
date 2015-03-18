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

GtkWidget *W_Balance;
GtkAdjustment *R_SliderAdjust;
GtkAdjustment *G_SliderAdjust;
GtkAdjustment *B_SliderAdjust;
GtkWidget *R_Slider;
GtkWidget *G_Slider;
GtkWidget *B_Slider;
GtkWidget *R_Spin;
GtkWidget *G_Spin;
GtkWidget *B_Spin;
GtkWidget *DefaultButt;

UINT8 g_combobox_choice=0;
UINT8 g_white_bal_gain=0;

void SetCroppedVGAMode(GtkButton *CroppedButt, struct ALL_DATA *all_data)
{
	BOOL ret = FALSE;
	UINT8 cropped_vga_status = 0;
	ret = EnableCroppedVGAMode(&cropped_vga_status);
	if(ret == FALSE)
	{
		ShowErrorDialog("Unable to switch to cropped VGA Mode",all_data);
		return;
	}
	else
	{
		switch(cropped_vga_status)
		{
			case 1 : 
				printf("Cropped VGA mode set successfully\n");
				break;
			case 2 :
				ShowErrorDialog("The current resolution is not 640x480, please switch to 640x480 before using the Cropping and Binning modes",all_data);
				break;
			case 3 :
				ShowErrorDialog("Device is already in Cropped VGA mode",all_data);
				break;
			case 4 :
				ShowErrorDialog("Failed to set Cropped VGA mode",all_data);
				break;
			default :
				printf("Unknown %d\n ",cropped_vga_status);
				
		}
	}
	return;
}
void SetBinnedVGAMode(GtkButton *BinnedButt, struct ALL_DATA *all_data)
{
	BOOL ret = FALSE;
	UINT8 binned_vga_status = 0;
	ret = EnableBinnedVGAMode(&binned_vga_status);
	if(ret == FALSE)
	{
		ShowErrorDialog("Unable to switch to binned VGA Mode",all_data);
		return;
	}
	else
	{
		switch(binned_vga_status)
		{
			case 1 : 
				printf("Binned VGA mode set successfully\n");
				break;
			case 2 :
				ShowErrorDialog("The current resolution is not 640x480, please switch to 640x480 before using the Cropping and Binning modes",all_data);
				break;
			case 3 :
				ShowErrorDialog("Device is already in Binned VGA mode",all_data);
				break;
			case 4 :
				ShowErrorDialog("Failed to set Binned VGA mode",all_data);
				break;
			default :
				printf("Unknown %d \n",binned_vga_status);
				
		}
	}
	return;
}

void evt_W_Balance_changed(GtkComboBox *W_Balance, struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	g_combobox_choice=gtk_combo_box_get_active(W_Balance);
	switch(g_combobox_choice)
	{
		case 0:
		{
			gtk_widget_set_sensitive((GtkWidget*)R_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)G_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)B_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)R_Spin,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)G_Spin,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)B_Spin,FALSE);
			ret=SetWhiteBalanceMode(WB_AUTO);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting white balance mode",all_data);
			} else {
			} 
			break;
		}
		case 1:
		{
			gtk_widget_set_sensitive((GtkWidget*)R_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)G_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)B_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)R_Spin,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)G_Spin,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)B_Spin,TRUE);
			ret=SetWhiteBalanceMode(WB_MANUAL);
			if(ret == FALSE) {
				ShowErrorDialog("Error in setting white balance mode",all_data);
			} else {
				ret=SetWhiteBalanceGain(WB_RED,gtk_range_get_value((GtkRange*)R_Slider));
				if(ret == FALSE)
					ShowErrorDialog("Error in setting white balance gain for red",all_data);
				ret=SetWhiteBalanceGain(WB_GREEN,gtk_range_get_value((GtkRange*)G_Slider));
				if(ret == FALSE)
					ShowErrorDialog("Error in setting white balance gain for green",all_data);
				ret=SetWhiteBalanceGain(WB_BLUE,gtk_range_get_value((GtkRange*)B_Slider));
				if(ret == FALSE)
					ShowErrorDialog("Error in setting white balance gain for blue",all_data);
			} 
			break;
		}
		default:
		{
			printf("Error getting type of White Balance");
			break;
		}
	}
	return;
}
gboolean evt_wb_slider_released(GtkRange *WB_Slider,GdkEvent *event,struct ALL_DATA *all_data)
{
	BOOL ret =FALSE;
	g_white_bal_gain=gtk_range_get_value((GtkRange*)WB_Slider);
	if(WB_Slider == (GtkRange*)R_Slider) {
		ret=SetWhiteBalanceGain(WB_RED,g_white_bal_gain);
	} else if(WB_Slider == (GtkRange*)G_Slider) {
		ret=SetWhiteBalanceGain(WB_GREEN,g_white_bal_gain);
	} else if(WB_Slider == (GtkRange*)B_Slider) {
		ret=SetWhiteBalanceGain(WB_BLUE,g_white_bal_gain);
	}
	if(ret == FALSE)
	{
		ShowErrorDialog("Error in setting white balance",all_data);
		return FALSE;
	}
	else
	{	
		printf("white balance set successfully\n");
	}
	return FALSE;
}

void evt_default_button_clicked(GtkButton *DefaultButt, struct ALL_DATA *all_data)
{
	BOOL ret= FALSE;
	UINT8 white_balance_mode =0, white_balance_red=0, white_balance_green=0, white_balance_blue=0 ;
	ret=DefaultWhiteBalance(&white_balance_mode,&white_balance_red,&white_balance_green,&white_balance_blue);
	if(ret == FALSE) {
		ShowErrorDialog("Error in setting default white balance",all_data);
	} else {
		g_signal_handlers_block_by_func((GtkComboBox*)W_Balance,G_CALLBACK (evt_W_Balance_changed), all_data);
		gtk_combo_box_set_active(GTK_COMBO_BOX(W_Balance),white_balance_mode-1);
		g_signal_handlers_unblock_by_func((GtkComboBox*)W_Balance,G_CALLBACK (evt_W_Balance_changed), all_data);
		gtk_range_set_value ((GtkRange*)R_Slider, white_balance_red);
		gtk_range_set_value ((GtkRange*)G_Slider, white_balance_green);
		gtk_range_set_value ((GtkRange*)B_Slider, white_balance_blue);
		if(white_balance_mode == WB_AUTO ) {
			gtk_widget_set_sensitive((GtkWidget*)R_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)G_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)B_Slider,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)R_Spin,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)G_Spin,FALSE);
			gtk_widget_set_sensitive((GtkWidget*)B_Spin,FALSE);
		} else if (white_balance_mode == WB_MANUAL) {
			gtk_widget_set_sensitive((GtkWidget*)R_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)G_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)B_Slider,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)R_Spin,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)G_Spin,TRUE);
			gtk_widget_set_sensitive((GtkWidget*)B_Spin,TRUE);
		}
	}
	return;	
}
void evt_update_wb_spin(GtkRange *WB_Slider,struct ALL_DATA *all_data)
{
	if(WB_Slider == (GtkRange*)R_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(R_Spin), gtk_range_get_value((GtkRange*)R_Slider));
	} else if(WB_Slider == (GtkRange*)G_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(G_Spin), gtk_range_get_value((GtkRange*)G_Slider));
	} else if(WB_Slider == (GtkRange*)B_Slider) {
		gtk_spin_button_set_value (GTK_SPIN_BUTTON(B_Spin), gtk_range_get_value((GtkRange*)B_Slider));
	}
	return;
}
void Draw11CUGControl(GtkWidget* table4,struct ALL_DATA *all_data)
{
	GtkWidget *label_mode_title;
	GtkWidget *label_w_balance_title;
	GtkWidget *label_w_balance;
	GtkWidget *label_other;

	UINT8 row=0;
	UINT8 white_balance_mode =0 ;
	BOOL ret=FALSE;	

	row++;

	//label "Camera mode controls"
	label_mode_title=gtk_label_new(_("---- Camera Mode Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_mode_title), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_mode_title, 1, row, 5, 1);
	gtk_widget_show (label_mode_title);

	row++;

	//Buttons for Master and Trigger Mode
	GtkWidget *MasterButt = gtk_button_new_with_label (_("Master Mode"));
	GtkWidget *TriggerButt = gtk_button_new_with_label (_("Trigger Mode"));
	gtk_grid_attach (GTK_GRID(table4), MasterButt, 2, row, 1, 1);
	gtk_grid_attach (GTK_GRID(table4), TriggerButt, 4, row, 1, 1);
	gtk_widget_show(MasterButt);
	gtk_widget_show(TriggerButt);
	g_signal_connect (GTK_BUTTON(MasterButt), "clicked", G_CALLBACK (SetMasterMode), all_data);
	g_signal_connect (GTK_BUTTON(TriggerButt), "clicked", G_CALLBACK (SetTriggerMode), all_data);

	GtkWidget *label_r_gain;
	GtkWidget *label_g_gain;
	GtkWidget *label_b_gain;

	row++;

	//Buttons for VGA Modes
	GtkWidget *CroppedButt = gtk_button_new_with_label (_("Cropped VGA"));
	GtkWidget *BinnedButt = gtk_button_new_with_label (_("Binned VGA"));
	gtk_grid_attach (GTK_GRID(table4), CroppedButt, 2, row, 1, 1);
	gtk_grid_attach (GTK_GRID(table4), BinnedButt, 4, row, 1, 1);
	gtk_widget_show(CroppedButt);
	gtk_widget_show(BinnedButt);

	g_signal_connect (GTK_BUTTON(CroppedButt), "clicked", G_CALLBACK (SetCroppedVGAMode), all_data);
	g_signal_connect (GTK_BUTTON(BinnedButt), "clicked", G_CALLBACK (SetBinnedVGAMode), all_data);

	row++;

	//label "White Balance controls"
	label_w_balance_title=gtk_label_new(_("---- White Balance Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_w_balance_title), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_w_balance_title, 1, row, 5, 1);
	gtk_widget_show (label_w_balance_title);

	row++;

	//Label "White balance mode"
	label_w_balance=gtk_label_new(_("White Balance :"));
	gtk_misc_set_alignment (GTK_MISC (label_w_balance), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_w_balance, 1, row, 1, 1);
	gtk_widget_show (label_w_balance);

	//Combo box for selecting White Balance mode
	W_Balance = gtk_combo_box_text_new ();

	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(W_Balance),"Auto ");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(W_Balance),"Manual");
	gtk_widget_set_halign(W_Balance,GTK_ALIGN_FILL);
	gtk_widget_set_hexpand(W_Balance,TRUE);
	gtk_grid_attach(GTK_GRID(table4),W_Balance,2,row,3,1);
	gtk_widget_show(W_Balance);
	ret=GetWhiteBalanceMode(&white_balance_mode);
	if(ret == FALSE ) {
		printf("Unable to get White Balance Mode\n");
	} else {
		gtk_combo_box_set_active(GTK_COMBO_BOX(W_Balance),white_balance_mode-1);
	}
	g_signal_connect(GTK_COMBO_BOX_TEXT(W_Balance), "changed",G_CALLBACK (evt_W_Balance_changed), all_data);

	row++;

	//Label "Red gain"
	label_r_gain=gtk_label_new(_("Red gain :"));
	gtk_misc_set_alignment (GTK_MISC (label_r_gain), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_r_gain, 1, row, 1, 1);
	gtk_widget_show (label_r_gain);

	//Slider for Red gain value
	R_SliderAdjust=gtk_adjustment_new(0,0,256,1,1,1);
	R_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,R_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (R_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE(R_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), R_Slider, 2, row, 3, 1);
	gtk_widget_show (R_Slider);
	g_signal_connect (GTK_SCALE(R_Slider), "button-release-event",G_CALLBACK (evt_wb_slider_released), all_data);
	g_signal_connect (GTK_SCALE(R_Slider), "key-release-event",G_CALLBACK (evt_wb_slider_released), all_data);
	//Spin button for Red gain
	R_Spin=gtk_spin_button_new_with_range(0,255,1);
	gtk_editable_set_editable(GTK_EDITABLE(R_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), R_Spin, 5, row, 1, 1);
	gtk_widget_show (R_Spin);

	row++;

	//Label "Green gain"
	label_g_gain=gtk_label_new(_("Green gain :"));
	gtk_misc_set_alignment (GTK_MISC (label_g_gain), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_g_gain, 1, row, 1, 1);
	gtk_widget_show (label_g_gain);

	//Slider for Green gain values
	G_SliderAdjust=gtk_adjustment_new(0,0,256,1,1,1);
	G_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,G_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (G_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE(G_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), G_Slider, 2, row, 3, 1);
	gtk_widget_show (G_Slider);
	g_signal_connect (GTK_SCALE(G_Slider), "button-release-event",G_CALLBACK (evt_wb_slider_released), all_data);
	g_signal_connect (GTK_SCALE(G_Slider), "key-release-event",G_CALLBACK (evt_wb_slider_released), all_data);

	//Spin button for Green gain
	G_Spin=gtk_spin_button_new_with_range(0,255,1);
	gtk_editable_set_editable(GTK_EDITABLE(G_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), G_Spin, 5, row, 1, 1);
	gtk_widget_show (G_Spin);

	row++;

	//Label "Blue gain"
	label_b_gain=gtk_label_new(_("Blue gain :"));
	gtk_misc_set_alignment (GTK_MISC (label_b_gain), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_b_gain, 1, row, 1, 1);
	gtk_widget_show (label_b_gain);

	//Slider for Blue gain values
	B_SliderAdjust=gtk_adjustment_new(0,0,256,1,1,1);
	B_Slider=gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,B_SliderAdjust);
	gtk_scale_set_draw_value (GTK_SCALE (B_Slider), FALSE);
	gtk_range_set_round_digits(GTK_RANGE(B_Slider), 0);
	gtk_grid_attach (GTK_GRID(table4), B_Slider, 2, row, 3, 1);
	gtk_widget_show (B_Slider);
	g_signal_connect (GTK_SCALE(B_Slider), "button-release-event",G_CALLBACK (evt_wb_slider_released), all_data);
	g_signal_connect (GTK_SCALE(B_Slider), "key-release-event",G_CALLBACK (evt_wb_slider_released), all_data);

	//Spin buttons for Blue gain
	B_Spin=gtk_spin_button_new_with_range(0,255,1);
	gtk_editable_set_editable(GTK_EDITABLE(B_Spin),FALSE);
	gtk_grid_attach (GTK_GRID(table4), B_Spin, 5, row, 1, 1);
	gtk_widget_show (B_Spin);

	g_signal_connect (GTK_SCALE(R_Slider), "value-changed",G_CALLBACK (evt_update_wb_spin), all_data);
	g_signal_connect (GTK_SCALE(G_Slider), "value-changed",G_CALLBACK (evt_update_wb_spin), all_data);
	g_signal_connect (GTK_SCALE(B_Slider), "value-changed",G_CALLBACK (evt_update_wb_spin), all_data);

	//Get gain values from camera and update slider
	ret = GetWhiteBalanceGain(WB_RED,&g_white_bal_gain);
	if(ret == FALSE)
		printf("Unable to get White Balance Gain\n");
	else
		gtk_range_set_value((GtkRange*)R_Slider,g_white_bal_gain);
	ret = GetWhiteBalanceGain(WB_GREEN,&g_white_bal_gain);
	if(ret == FALSE)
		printf("Unable to get White Balance Gain\n");
	else
		gtk_range_set_value((GtkRange*)G_Slider,g_white_bal_gain);
	ret = GetWhiteBalanceGain(WB_BLUE,&g_white_bal_gain);
	if(ret == FALSE)
		printf("Unable to get White Balance Gain\n");
	else
		gtk_range_set_value((GtkRange*)B_Slider,g_white_bal_gain);

	if(white_balance_mode)
	{
		switch(white_balance_mode)
		{
			case 1:
				gtk_widget_set_sensitive((GtkWidget*)R_Slider,FALSE);
				gtk_widget_set_sensitive((GtkWidget*)G_Slider,FALSE);
				gtk_widget_set_sensitive((GtkWidget*)B_Slider,FALSE);
				gtk_widget_set_sensitive((GtkWidget*)R_Spin,FALSE);
				gtk_widget_set_sensitive((GtkWidget*)G_Spin,FALSE);
				gtk_widget_set_sensitive((GtkWidget*)B_Spin,FALSE);
				break;
			case 2:
				gtk_widget_set_sensitive((GtkWidget*)R_Slider,TRUE);
				gtk_widget_set_sensitive((GtkWidget*)G_Slider,TRUE);
				gtk_widget_set_sensitive((GtkWidget*)B_Slider,TRUE);
				gtk_widget_set_sensitive((GtkWidget*)R_Spin,TRUE);
				gtk_widget_set_sensitive((GtkWidget*)G_Spin,TRUE);
				gtk_widget_set_sensitive((GtkWidget*)B_Spin,TRUE);
				break;
		}
	}
	row++;

	//Buton for default values
	DefaultButt = gtk_button_new_with_label (_("Default Values"));
	gtk_grid_attach(GTK_GRID(table4),DefaultButt,3,row,1,1);
	gtk_widget_show(DefaultButt);

	g_signal_connect (GTK_BUTTON(DefaultButt), "clicked", G_CALLBACK (evt_default_button_clicked), all_data);

	row++;

	//Label for Other Options
	label_other = gtk_label_new(_("---- Other Options ----"));
	gtk_misc_set_alignment (GTK_MISC (label_other), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_other, 1, row, 5, 1);
	gtk_widget_show (label_other);

	row++;

	//Buttons for F/W version and About
	GtkWidget *FwVersionButt = gtk_button_new_with_label (_("F/W Verison"));
	GtkWidget *AboutButt = gtk_button_new_with_label (_("About"));
	gtk_grid_attach(GTK_GRID(table4),FwVersionButt,2,row,1,1);
	gtk_grid_attach(GTK_GRID(table4),AboutButt,4,row,1,1);
	gtk_widget_show (FwVersionButt);
	gtk_widget_show (AboutButt);
	g_signal_connect (GTK_BUTTON(FwVersionButt), "clicked", G_CALLBACK (ShowFwVersion), all_data);
	g_signal_connect (GTK_BUTTON(AboutButt), "clicked", G_CALLBACK (ShowAbout), all_data);

	return;

}

