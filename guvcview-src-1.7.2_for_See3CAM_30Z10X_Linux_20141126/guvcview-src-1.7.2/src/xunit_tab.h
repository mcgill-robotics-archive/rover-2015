//added by E-con for its uvc xu control


/*******************************************************************************#
#           guvcview              http://guvcview.berlios.de                    #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#           Nobuhiro Iwamatsu <iwamatsu@nigauri.org>                            #
#                             Add UYVY color support(Macbook iSight)            #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 3 of the License, or             #
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
#ifndef XUNIT_TAB_H
#define XUNIT_TAB_H

#include "guvcview.h"
//------------------------- UVC XU Control Tab ---------------------------------



void xunit_tab(struct ALL_DATA *all_data);

void ShowErrorDialog(const char *err_msg, struct ALL_DATA *all_data);

void ShowFwVersion(GtkButton *FwVersionButt, struct ALL_DATA *all_data);

void ShowAbout(GtkButton *AboutButt, struct ALL_DATA *all_data);

void SetMasterMode(GtkButton *MasterButt, struct ALL_DATA *all_data);

void SetTriggerMode(GtkButton *TriggerButt, struct ALL_DATA *all_data);

void Draw80USBControl(GtkWidget* table4,struct ALL_DATA *all_data);

void Draw10CUGControl(GtkWidget* table4,struct ALL_DATA *all_data);

void Draw11CUGControl(GtkWidget* table4,struct ALL_DATA *all_data);

void DrawZ10XControl(GtkWidget* table4,struct ALL_DATA *all_data);

void DrawCU50Control(GtkWidget* table4,struct ALL_DATA *all_data);

#endif
