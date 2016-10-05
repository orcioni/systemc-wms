// Electrothermal devices:
// Copyright (C) 2016 Giorgio Biagetti and Simone Orcioni
/*
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef ELC_THERMAL_H
#define ELC_THERMAL_H

#include "../analog_system"
#include "../wave_system"
#include "../sys/oneport"
#include "../sys/twoport"
#include "../nature/thermal"
#include "thermal.h"


// Declaration of electrothermal components

typedef P_load_th<electrical>		  R_load_th;
typedef P_load_var_th<electrical>     R_load_var_th;
typedef PIp_load_var_th<electrical>   RCp_load_var_th;
typedef PIp_load_th<electrical>		  RCp_load_th;
typedef Ps_2s_th<electrical>		  Rs_2s_th;
typedef PIp_2p_th<electrical>		  RCp_2p_th;


#endif //ELC_THERMAL_H