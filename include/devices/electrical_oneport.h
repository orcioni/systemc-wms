// electrical_devices:
// Copyright (C) 2013 Giorgio Biagetti and Simone Orcioni
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

#ifndef ELECTRICAL_ONEPORT_H
#define ELECTRICAL_ONEPORT_H

#include "../wave_system"
#include "../analog_system"
#include "../nature/electrical"
#include "../sys/oneport"
#include "../sys/nonlinear"

// Library of linear components and circuits:
//
// load
//
// R_load R1(sc_core::sc_module_name name, double R)    -- R  one port
// L_load L1(sc_core::sc_module_name name, double L)    -- L  one port
// C_load C1(sc_core::sc_module_name name, double C)    -- C  one port
// RCs_load RC1(sc_core::sc_module_name name, double R, double C) -- RC  series, one port
// RCp_load  -- RC parallel, one port
// RLs_load  -- RL  series, one port
// RLp_load  -- RL parallel, one port
// RLCs_load -- RLC series, one port
// RLCp_load -- RLC parallel, one port
// Coulomb_counter
typedef P_load<electrical>  R_load;
typedef D_load<electrical>  L_load;
typedef I_load<electrical>  C_load;
typedef PIs_load<electrical>  RCs_load;
typedef PIp_load<electrical>  RCp_load;
typedef PDs_load<electrical>  RLs_load;
typedef PDp_load<electrical>  RLp_load;
typedef PDIs_load<electrical>  RLCs_load;
typedef PDIp_load<electrical>  RLCp_load;
typedef integrator<electrical> Coulomb_counter;

// Non linear components and circuits:
//
// idiode D1(sc_core::sc_module_name name) -- Ideal diode, one port
// th_idiode D1(sc_core::sc_module_name name) -- Ideal diode with threshold, one port
// onoff_switch     --  Controlled ideal switch one port, without recycling diode
// onoff_switchd    --  Controlled ideal switch one port, with recycling diode
typedef rectifier<electrical> idiode;
typedef th_rectifier<electrical> th_idiode;
typedef controlled_switch<electrical> onoff_switch;
typedef controlled_switch_rect<electrical> onoff_switchd;

#endif  //ELECTRICAL_ONEPORT_H