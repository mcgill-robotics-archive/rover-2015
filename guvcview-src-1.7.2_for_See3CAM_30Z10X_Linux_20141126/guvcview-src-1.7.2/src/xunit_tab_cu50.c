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

GtkWidget *check_Flash;
GtkWidget *check_Torch;
GtkWidget *GPIO_Out;
GtkWidget *GPIO_In;
GtkWidget *GPIO_UpdateButt;
GtkWidget *GPIO_Radio1;
GtkWidget *GPIO_Radio2;
GtkWidget *label_GPIO_in_status;

UINT8 g_combobox_choice;

void evt_50_GPIO_radio_changed(GtkRadioButton *GPIO_Radio1, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 GpioValue = 0;
	g_combobox_choice=gtk_combo_box_get_active((GtkComboBox*)GPIO_Out);
	GpioValue=gtk_toggle_button_get_active((GtkToggleButton*)GPIO_Radio1);
	if(g_combobox_choice==0) {
		ret = SetGpioLevel(GPIO_50_OUT1,GpioValue);
	} else {
		ret = SetGpioLevel(GPIO_50_OUT2,GpioValue);
	}
	if(ret == FALSE) {
		ShowErrorDialog("Error in setting GPIO level",all_data);
		return;
	}
	return;
}
void evt_50_GPIO_out_changed(GtkComboBox *GPIO_Out, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 GpioValue = 0;
	g_combobox_choice=gtk_combo_box_get_active(GPIO_Out);
	if(g_combobox_choice==0) {
		ret = GetGpioLevel(GPIO_50_OUT1,&GpioValue);
	} else {
		ret = GetGpioLevel(GPIO_50_OUT2,&GpioValue);
	}
	if(ret == FALSE) {
		ShowErrorDialog("Error in getting GPIO level",all_data);
		return;
	}
	if(GpioValue == 1 ) {
		g_signal_handlers_block_by_func((GtkToggleButton*)GPIO_Radio1, G_CALLBACK (evt_50_GPIO_radio_changed), all_data);
		gtk_toggle_button_set_active((GtkToggleButton*)GPIO_Radio1,TRUE);
		g_signal_handlers_unblock_by_func((GtkToggleButton*)GPIO_Radio1, G_CALLBACK (evt_50_GPIO_radio_changed), all_data);
	} else {
		g_signal_handlers_block_by_func((GtkToggleButton*)GPIO_Radio1, G_CALLBACK (evt_50_GPIO_radio_changed), all_data);
		gtk_toggle_button_set_active((GtkToggleButton*)GPIO_Radio2,TRUE);
		g_signal_handlers_unblock_by_func((GtkToggleButton*)GPIO_Radio1, G_CALLBACK (evt_50_GPIO_radio_changed), all_data);
	}
	return;
}

void evt_50_GPIO_button_clicked(GtkButton *GPIO_UpdateButt, struct ALL_DATA *all_data)
{
	BOOL ret=FALSE;
	UINT8 GpioValue = 0;
	g_combobox_choice=gtk_combo_box_get_active((GtkComboBox*)GPIO_In);
	if(g_combobox_choice==0) {
		ret = GetGpioLevel(GPIO_50_IN1,&GpioValue);
	} else {
		ret = GetGpioLevel(GPIO_50_IN2,&GpioValue);
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
void evt_50_flash_toggled(GtkWidget *Widget, struct ALL_DATA *all_data)
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
void evt_50_torch_toggled(GtkWidget *Widget, struct ALL_DATA *all_data)
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

void DrawCU50Control(GtkWidget* table4,struct ALL_DATA *all_data)
{
	GtkWidget *HButtonBox2;
	GtkWidget *label_GPIO_controls;
	GtkWidget *label_GPIO_out_no;
	GtkWidget *label_GPIO_out_level;
	GtkWidget *label_GPIO_in_no;
	GtkWidget *label_GPIO_in_level;
	GtkWidget *label_flash_controls;
	GtkWidget *label_other;

	UINT8 GpioValue = 0, flash_level = 0, torch_level=0;
	UINT8 row = 0;
	BOOL ret = FALSE;

	row++;

	//label "GPIO controls"
	label_GPIO_controls=gtk_label_new(_("---- GPIO Controls ----"));
	gtk_misc_set_alignment (GTK_MISC (label_GPIO_controls), 0.5, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_GPIO_controls, 1, row, 3, 1);
	gtk_widget_show (label_GPIO_controls);

	row++;

	//Label "Output GPIO number"
	label_GPIO_out_no = gtk_label_new(_("GPO number :"));
	gtk_misc_set_alignment (GTK_MISC (label_GPIO_out_no), 1, 0.5);
	gtk_grid_attach (GTK_GRID(table4), label_GPIO_out_no, 1, row, 1, 1);
	gtk_widget_show (label_GPIO_out_no);

	//Combo box for selecting output GPIO
	GPIO_Out = gtk_combo_box_text_new ();
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(GPIO_Out),"1");
	gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(GPIO_Out),"2");
	gtk_combo_box_set_active(GTK_COMBO_BOX(GPIO_Out),0);
	gtk_grid_attach (GTK_GRID(table4), GPIO_Out, 2, row, 1, 1);
	gtk_widget_show (GPIO_Out);
	g_signal_connect(GTK_COMBO_BOX_TEXT(GPIO_Out), "changed",G_CALLBACK (evt_50_GPIO_out_changed), all_data);

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

	ret = GetGpioLevel(GPIO_50_OUT1,&GpioValue);
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
	g_signal_connect(GTK_CHECK_BUTTON(GPIO_Radio1), "toggled",G_CALLBACK (evt_50_GPIO_radio_changed), all_data);

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

	g_signal_connect(GTK_COMBO_BOX_TEXT(GPIO_In), "changed",G_CALLBACK (evt_50_GPIO_button_clicked), all_data);

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
	g_signal_connect(GTK_BUTTON(GPIO_UpdateButt), "clicked",G_CALLBACK (evt_50_GPIO_button_clicked), all_data);
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

	g_signal_connect (GTK_BUTTON(check_Flash), "toggled",G_CALLBACK (evt_50_flash_toggled), all_data);
	g_signal_connect (GTK_BUTTON(check_Torch), "toggled",G_CALLBACK (evt_50_torch_toggled), all_data);

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
