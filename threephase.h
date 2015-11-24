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

#ifndef THREEPHASE_H
#define THREEPHASE_H

#include "../analog_system"
#include "../wave_system"
#include "../nature/threephase"
#include "../nature/electrical"
#include "../sys/oneport"
#include "../sys/twoport"
#include "../sys/nonlinear"

//
// Library of threephase circuits:
// All values of the components should be given as the circuits would wye connected

// R_load_tph(sc_core::sc_module_name name, double R)
// R - one port

// RCs_load_tph
// RC series - one port

// RLCs_2s_tph(sc_core::sc_module_name name, double R, double L, double C)
// RLS series - two port in series between port1 and port2

// Rs_2s_tph(sc_core::sc_module_name name, double R)
// R - two port in series between port1 and port2

// RCs_2p_tph(sc_core::sc_module_name name, double R, double C)
// RC series - two port in series between port1 and port2

// RCs_2p_tph(sc_core::sc_module_name name, double R, double C)
// RC series - two port in parallel between port1 and port2

// RLs_load_tph (sc_core::sc_module_name name, double R, double L)
// RL series - two port in series between port1 and port2

// RLs_2s_tph(sc_core::sc_module_name name, double R, double L)
// RL series - two port in series between port1 and port2

// NTC_2s_tph(sc_core::sc_module_name name, double R, double tau)
// NTC - two port in series between port1 and port2

// LsCp_ladder_tph(sc_core::sc_module_name name, double L, double C)
// L series and C parallel - two port ladder between port1 and port2

// clarke2wye (sc_core::sc_module_name name)
// A module to connect a 3phase channel, Clarke transformed in complex double, to 3 single phase wye-connected channels

// wye2delta (sc_core::sc_module_name name)
// A module to connect a Clarke transformed 3 phase wye system to a a Clarke transformed 3 phase delta system; still not working

// c2vect(sc_core::sc_module_name name)
// A module to convert a complex channel to two double channel; useful to convert a Clarke complex in the two vector components.


typedef P_load<threephase>  R_load_tph;
typedef Ps_2s<threephase>  Rs_2s_tph;
typedef controlled_switch_2s<threephase> switch_2s_tph;

//	Declaration of class RCseries_load:
//  In initial condition, ics, V0 represents the delta voltage on one of the three
//  capacitors, taken as they would delta connected.

struct RCs_load_tph : wave_module<1, threephase>, analog_module
{
	SC_HAS_PROCESS(RCs_load_tph);
	RCs_load_tph (sc_core::sc_module_name name, double proportional_element, double derivative_element);
	
public:
	void ics(double VC0);
private:
    void calculus ();
    void field (double *var) const;
    const double R, C;
};

// Class RLCs_2s_tph
//   Declaration of class RLC_series_2_ports_series with equal norm. resistance:

struct RLCs_2s_tph : wave_module <2, threephase>, analog_module
{
	SC_HAS_PROCESS(RLCs_2s_tph);
	RLCs_2s_tph (sc_core::sc_module_name name, double proportional_element, double derivative_element, double integrative_element);
	ab_port <threephase>  &p1, &p2;
private:
	void field (double *var) const;
	void calculus ();
	double P, D, I;
};

//	Declaration of class RCs_2p
//  In initial condition, ics, V0 represents the delta voltage on one of the three
//  capacitors, taken as they would delta connected.

struct RCs_2p_tph : wave_module<2, threephase>, analog_module
{
	SC_HAS_PROCESS(RCs_2p_tph);
	RCs_2p_tph (sc_core::sc_module_name name, double proportional_element, double integrative_element);
	
public:
	void ics(double VC0);
private:
	void calculus ();
	void field (double *var) const;
	const double R, C;
};


//	Declaration of class NTC_s2s_tph  with equal norm. resistance

struct NTC_2s_tph : wave_module<2, threephase>
{
	SC_HAS_PROCESS(NTC_2s_tph);
	NTC_2s_tph (sc_core::sc_module_name name, double proportional_element, double time_constant);

private:
	void calculus ();
	const double P, tau;
};


//	Declaration of class RLs_2s_tph

struct RLs_2s_tph : wave_module<2, threephase>, analog_module
{
	SC_HAS_PROCESS(RLs_2s_tph);
	RLs_2s_tph (sc_core::sc_module_name name, double proportional_element, double derivative_element
	);
	
private:
	void calculus ();
	void field (double *var) const;
	const double R, L;
};


//	Declaration of class LsCp_ladder_tph

struct LsCp_ladder_tph : wave_module<2, threephase>, analog_module
{
	SC_HAS_PROCESS(LsCp_ladder_tph);
	LsCp_ladder_tph (sc_core::sc_module_name name, double s_L, double p_C);
	ab_port <threephase> &s, &p;
private:
	void calculus ();
	void field (double *var) const;
	const double C,L;
};

//	Declaration of RLs_load_tph

struct RLs_load_tph : wave_module<1, threephase>, analog_module
{
	SC_HAS_PROCESS(RLs_load_tph);
	RLs_load_tph (sc_core::sc_module_name name, double s_R, double s_L);
	
private:
    void calculus ();
    void field (double *var) const;
    const double R,L;
};


//Declaration of clarke2wye; a module to connect a complex-Clarke 3phase channel to 3 single phase wye-connected channels:

struct clarke2wye : wave_module<4, electrical, electrical, electrical, threephase>
{
	SC_HAS_PROCESS(clarke2wye);
	clarke2wye (sc_core::sc_module_name name);
	ab_port <electrical> &a, &b, &c;
	ab_port <threephase> &clarke;
private:
	void calculus ();
};

//Declaration of wye2delta; a module to connect a Clarke transformed 3 phase wye system to a a Clarke transformed 3 phase delta system:

/* struct wye2delta : wave_module<2, threephase>
{
	SC_HAS_PROCESS(wye2delta);
	wye2delta (sc_core::sc_module_name name);
	ab_port <threephase> &wye, &delta;
private:
	void calculus ();
};

*/

//Declaration of complex Clarke to vector Clarke
struct c2vect : wave_module<3, threephase, electrical, electrical>
{
	SC_HAS_PROCESS(c2vect);
	c2vect(sc_core::sc_module_name name);
	ab_port <threephase> &clarke;
	ab_port <electrical> &v1, &v2;
private:
	void flowclarke ();
	void flowvect ();
	std::complex<double> tmp;
};

#endif //THREEPHASE_H