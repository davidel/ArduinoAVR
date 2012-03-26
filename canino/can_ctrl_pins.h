/*
 *  canino by Davide Libenzi (CAN bus interface software for AVR Arduino boards)
 *  Copyright (C) 2012  Davide Libenzi
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Davide Libenzi <davidel@xmailserver.org>
 *
 */

#pragma once

#if defined(__AVR_ATmega2560__)

#define P_MOSI  B,2
#define P_MISO  B,3
#define P_SCK   B,1

#define MCP2515_CS          B,0
#define MCP2515_INT         D,2
#define LED2_HIGH           B,7
#define LED2_LOW            B,7

#define SDCARD_CS           H,6

#else

#define P_MOSI  B,3
#define P_MISO  B,4
#define P_SCK   B,5

#define MCP2515_CS          B,2
#define MCP2515_INT         D,2
#define LED2_HIGH           B,0
#define LED2_LOW            B,0

#define SDCARD_CS           B,1

#endif

#define SDCARD_CS_PIN       9

