#ifndef SHIFT_H
#define SHIFT_H

#include <inttypes.h>

/*
Public Invention's Ox Project is an open source hardware design for an oxygen concentrator for use by field hospitals around the world. This design concept aims to design an oxygen concentrator that can be manufactured locally while overcoming challenges posed by human resources, hospital location (geographically), infrastructure and logistics; in addition, this project attempts minimum documentation of their design standards necessary for local approval whilst tackling regulatory requirements for medical devices. Copyright (C) 2021 Robert Read, Ben Coombs, and Darío Hereñú.

This program includes free Firmware/Hardware designs: you can redistribute, use, study it and/or modify it under the terms of the CERN Open Hardware License Version 2 as published here: https://ohwr.org/cern_ohl_s_v2.txt

See the CERN Open Hardware License Version 2 for more details.
You should have received a copy of the CERN Open Hardware License Version 2 along with this program.  If not, see https://ohwr.org/cern_ohl_s_v2.txt.

This program includes free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

See the GNU Affero General Public License for more details.
You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

/*#define DS 13 // 747HC pin 14 - serial data
#define ST_CP 12 // 747HC pin 12 - storage register clock (latch)
#define SH_CP 27 // 747HC pin 11 - shift register clock*/
struct shift_pins {
    int latch;
    int clock;
    int data;
};

void shiftInit();
void shiftOutValves(uint8_t data_out);

#endif
