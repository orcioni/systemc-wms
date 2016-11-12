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

#ifndef ELECTRICAL_TWOPORT_H
#define ELECTRICAL_TWOPORT_H

#include "../nature/electrical"
#include "../analog_system"
#include "../sys/nonlinear"
#include "../sys/twoport"
#include "../sys/ideal"

// Library of linear components and circuits:
//
// load
// transducer
// transformer
//
// Rs_2s     -- R series - two port in series between port1 and port2
// RCs_2s    -- RC series - two port in series between port1 and port2
// RCs_2p    -- RC series - two port in parallel with port1 and port2
// RCp_2s    -- RC parallel- two port in series between port1 and port2
// RCp_2p    -- RC parallel- two port in parallel with port1 and port2
// RLs_2s    -- RL series - two port in series between port1 and port2
// RLs_2p    -- RL series - two port in parallel with port1 and port2
// RLp_2s    -- RL parallel- two port in series between port1 and port2
// RLp_2p    -- RL parallel- two port in parallel with port1 and port2
// RLCs_2s   -- RLC series- two port in series between port1 and port2
// RLCs_2p   -- RLC series- two port in parallel with port1 and port2
// RLCp_2s   -- RLC parallel- two port in series between port1 and port2
// RLCp_2p   -- RLC parallel- two port in parallel with port1 and port2
typedef Ps_2s<electrical>   Rs_2s;
typedef PIs_2s<electrical>  RCs_2s;
typedef PIs_2p<electrical>  RCs_2p;
typedef PIp_2s<electrical>  RCp_2s;
typedef PIp_2p<electrical>  RCp_2p;
typedef PDs_2s<electrical>  RLs_2s;
typedef PDs_2p<electrical>  RLs_2p;
typedef PDp_2s<electrical>  RLp_2s;
typedef PDp_2p<electrical>  RLp_2p;
typedef PDIs_2s<electrical>  RLCs_2s;
typedef PDIs_2p<electrical>  RLCs_2p;
typedef PDIp_2s<electrical>  RLCp_2s;
typedef PDIp_2p<electrical>  RLCp_2p;

// LsCp_ladder -- second order low pass filter,2port L series and C parallel
// CCCS(sc_core::sc_module_name name, double constant) -- current controlled current source

typedef DsIp_ladder<electrical> LsCp_ladder;
typedef TCTS<electrical>  CCCS;


//
// Non linear components and circuits:
//
// diode_2s(port1, port2)  --  Ideal diode connected in series
//                             between port1 (anode) and port2 (cathode)
// diode_bridge  --  Ideal Graetz' bridge
// onoff_switch_2s(port1, port2, control)  --  Controlled ideal switch 2 ports,
//                                             without recycling diode(symmetrical)

// onoff_switchd_2s(port1, port2, control) --  Controlled ideal switch 2 ports,
//                                             with recycling diode connected in
//											   inverse between
//                                             port2 (cathode) and port1 (anode)

typedef rectifier_2s<electrical> diode_2s;
typedef ideal_rectifier<electrical> diode_bridge;
typedef controlled_switch_2s<electrical> onoff_switch_2s;
typedef controlled_switch_rect_2s<electrical> onoff_switchd_2s;
typedef controlled_switch_loss_rect_2s<electrical> Ron_switchd_2s;
typedef controlled_switch_loss_2s<electrical> Ron_switch_2s;


#endif //ELECTRICAL_TWOPORT_H
