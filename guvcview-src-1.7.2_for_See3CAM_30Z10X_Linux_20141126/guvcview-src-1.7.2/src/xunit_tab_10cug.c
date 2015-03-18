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

void SetMasterMode(GtkButton *MasterButt, struct ALL_DATA *all_data)
{
	BOOL ret = FALSE;
	ret = EnableMasterMode();
	if(ret == FALSE)
	{
		ShowErrorDialog("Unable to switch to Master Mode",all_data);
		return;
	}
	printf("Master mode set successfully\n");
	return;
}

void SetTriggerMode(GtkButton *TriggerButt, struct ALL_DATA *all_data)
{
	BOOL ret = FALSE;
	ret = EnableTriggerMode();
	if(ret == FALSE)
	{
		ShowErrorDialog("Unable to switch to Trigger Mode",all_data);
		return;
	}
	printf("Trigger mode set successfully\n");
	return;
}

void Draw10CUGControl(GtkWidget* table4,struct ALL_DATA *all_data)
{
	GtkWidget *HButtonBox1;
	GtkWidget *HButtonBox2;
	/*------------------------------ Add Buttons ------------------------------*/

	HButtonBox1 = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
        gtk_widget_set_halign (HButtonBox1, GTK_ALIGN_FILL);
		gtk_widget_set_hexpand (HButtonBox1, TRUE);
        gtk_button_box_set_layout(GTK_BUTTON_BOX(HButtonBox1),GTK_BUTTONBOX_SPREAD);
        gtk_box_set_homogeneous(GTK_BOX(HButtonBox1),TRUE);

	HButtonBox2 = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
        gtk_widget_set_halign (HButtonBox2, GTK_ALIGN_FILL);
		gtk_widget_set_hexpand (HButtonBox2, TRUE);
        gtk_button_box_set_layout(GTK_BUTTON_BOX(HButtonBox2),GTK_BUTTONBOX_SPREAD);
        gtk_box_set_homogeneous(GTK_BOX(HButtonBox2),TRUE);


	//Master Mode button
	
	GtkWidget *MasterButt = gtk_button_new_with_label (_("Master Mode"));
	GtkWidget *TriggerButt = gtk_button_new_with_label (_("Trigger Mode"));
	GtkWidget *FwVersionButt = gtk_button_new_with_label (_("F/W Verison"));
	GtkWidget *AboutButt = gtk_button_new_with_label (_("About"));

	gtk_box_pack_start (GTK_BOX (HButtonBox1), MasterButt, TRUE, TRUE, 1);
	gtk_box_pack_start (GTK_BOX (HButtonBox1), TriggerButt, TRUE, TRUE, 1);
	gtk_box_pack_start (GTK_BOX (HButtonBox2), FwVersionButt, TRUE, TRUE, 1);
	gtk_box_pack_start (GTK_BOX (HButtonBox2), AboutButt, TRUE, TRUE, 1);

	gtk_widget_show_all (MasterButt);
	gtk_widget_show_all (TriggerButt);
	gtk_widget_show_all (FwVersionButt);
	gtk_widget_show_all (AboutButt);

	g_signal_connect (GTK_BUTTON(MasterButt), "clicked", G_CALLBACK (SetMasterMode), all_data);
	g_signal_connect (GTK_BUTTON(TriggerButt), "clicked", G_CALLBACK (SetTriggerMode), all_data);
	g_signal_connect (GTK_BUTTON(FwVersionButt), "clicked", G_CALLBACK (ShowFwVersion), all_data);
	g_signal_connect (GTK_BUTTON(AboutButt), "clicked", G_CALLBACK (ShowAbout), all_data);

	gtk_grid_attach (GTK_GRID(table4), HButtonBox1, 0, 1, 1, 50);
	gtk_grid_attach (GTK_GRID(table4), HButtonBox2, 0, 1, 1, 100);

	gtk_widget_show_all (HButtonBox1);
	gtk_widget_show_all (HButtonBox2);

	return;
}
