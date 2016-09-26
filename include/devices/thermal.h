// Thermal devices:
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

#ifndef THERMAL_H
#define THERMAL_H

#include "../analog_system"
#include "../wave_system"
#include "../sys/oneport"
#include "../sys/twoport"
#include "../nature/thermal"

//
// Library of thermal devices
//
//

// One port

// Thermal element representing thermal CONDUCTION: transport of heat without storing it.
// Thermal_resistance R1(sc_core::sc_module_name name, double R);

// Box system. Heat flows along the length
// Thermal conductance G = k*A/L [W/K]
// Thermal resistance  R = 1/G
// k: thermal conductivity (depending on material) [W/(m K)]
// A: Area   [m^2]
// L: length [m]

// Radial system. Heat flows from inside to outside
// Thermal conductance G = 2*pi*k*L/ln(r2/r1) [W/K]
// Thermal resistance  R = 1/G
// r2 = outer radius
// r1 = inner radius

// For a list of thermal conductivity please refer to
// https://en.wikipedia.org/wiki/List_of_thermal_conductivities

typedef P_load<thermal>    Thermal_resistance;

// Thermal element representing STORING of heat. It is also called heat mass.
//If a start value for initialization (i.e. T = 25 degree Celsius) is needed:
// Thermal_capacity C1(sc_core::sc_module_name name, double C);
// C1.ics(25.0 Celsius);

typedef I_load<thermal>  Thermal_capacitance;


// Two ports

// CONVECTION
// This thermal element represents a plate in contact with a fluid
// It can be represented as a 2 port with a resistance between the thermal conductor and the fluid
// The thermal conductance Gr is calculated as a function of h
// Gc = A*h [W/K]
// A = Area of radiator [m^2]
// h: coefficient of convective heat transfer [W/(m^2 K)]

// If thermal conductance is constant
// Thermal_convection radiator(sc_core::sc_module_name name, double h, double A)

struct Thermal_convector : wave_module <2, thermal>
{
	SC_HAS_PROCESS(Thermal_convector);
	Thermal_convector(sc_core::sc_module_name name, double area, double h);
private:
	void calculus ();
	const double A, H;
};


//Since the thermal conductance is never constant in practice
// it may be a function of the speed of a cooling fan
// it can be written as a function of a signal h
// R. Fischer: Elektrische Maschinen, 10th edition, Hanser-Verlag 1999, p. 378):

// h = 7.8*v^0.78 [W/(m^2 K)] (forced convection)
// h = 12         [W/(m^2 K)] (free convection)
// where
// v: Air velocity in [m/s]


struct Thermal_convector_var : wave_module <2, thermal>
{
	SC_HAS_PROCESS(Thermal_convector_var);
	Thermal_convector_var(sc_core::sc_module_name name, double area);
	sc_core::sc_in <typename thermal::wave_type> h;

private:
	void calculus ();
	const double A;
};

// RADIATION
// This thermal element represents thermal radiation
// TODO
// equation Q_flow = Gr*ssigma*(T1^4 - T2^4);
// Gr is the radiation conductance and ssigma is Stefan-Boltzmann constant;


// Sources
// components of type source can be used as an ideal source of heat or a body with fixed temperature
//
// ideal heat source:
// source <thermal> s1("S1", cfg::through);
//
// body with fixed temperature (can accept any heat flow without change its temperature)
// source <thermal> amb("amb", cfg::across);


//
// Library of electrical devices with a thermal port
//
//

// One template port and one thermal port


//	Declaration of class P_load_th
template <class T1>
struct P_load_th : wave_module<2, T1, thermal>
{
	typedef wave_module<> base_class;
	SC_HAS_PROCESS(P_load_th);
	P_load_th (sc_core::sc_module_name name, double proportional_element);
	ab_port <T1>      &tmpl_port;
	ab_port <thermal> &thrm_port;
private:
	void calculus ();
	const double P;
};

//	Implementation of class P_load_th:

template <class T1> P_load_th<T1>::P_load_th (sc_core::sc_module_name name, double proportional_element) : P(proportional_element), tmpl_port(base_class::port<T1>(1)), thrm_port(base_class::port<thermal>(2))
{
	SC_METHOD(calculus);
	this->sensitive << this->activation;
	tmpl_port <<= 5;
	thrm_port <<= 5;
}

template <class T1> void P_load_th<T1>::calculus ()
{
	const double reflection = (P - tmpl_port->get_normalization()) / (P + tmpl_port->get_normalization());
	typename T1::wave_type a = tmpl_port->read();
	typename T1::wave_type b = reflection * a;
	tmpl_port->write(b);
	thrm_port->write(thrm_port->read() + (a*a - b*b) * sqrt(thrm_port->get_normalization()));
}

//	Declaration of class PIparallel_load_th:

template <class T>
struct PIp_load_th : wave_module<2, T, thermal>, analog_module
{
	typedef wave_module<> base_class;
	SC_HAS_PROCESS(PIp_load_th);
	PIp_load_th (sc_core::sc_module_name name, double proportional_element, double integrative_element);
	ab_port <T> &tmpl_port;
	ab_port <thermal> &thrm_port;
public:
	void ics(double IC);
private:
	void calculus ();
	void field (double *var) const;
	const double P, I;
};


//	Implementation of class PIparallel_load_th:

template <class T> PIp_load_th<T>::PIp_load_th (sc_core::sc_module_name name, double proportional_element, double integrative_element) : analog_module(1), P(proportional_element), I(integrative_element), tmpl_port(base_class::port<T>(1)), thrm_port(base_class::port<thermal>(2))
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
}

template <class T> void PIp_load_th<T>::ics(double IC)
{
	this->ic(I*IC);
}

template <class T> void PIp_load_th<T>::field (double *var) const
{
	const double P0 = tmpl_port->get_normalization();
	const double sqrt_P0 = tmpl_port->get_normalization_sqrt();
	
	var[0] = 2 * tmpl_port->read() / sqrt_P0 -  state[0]*(P+P0)/(P*P0*I);
}

template <class T> void PIp_load_th<T>::calculus ()
{
	const double sqrt_P0 = tmpl_port->get_normalization_sqrt();
	if (!set_steplimits_used) {
		const double P0 = tmpl_port->get_normalization();
		set_steplimits(P0*I/100.0,P0/10.0);
	}
	while (step()) {
		double a = tmpl_port->read();
		double b = - (a - state[0]/(sqrt_P0*I));
		double vR = (a+b)*sqrt_P0;
		tmpl_port->write(b);
		thrm_port->write(thrm_port->read() + (vR*vR/P) * sqrt(thrm_port->get_normalization()));
		
	}
}

// Two template ports and one thermal port

//	Declaration of class Ps_2s

template <class T>
struct Ps_2s_th : wave_module<3, T, T, thermal>
{
	typedef wave_module<> base_class;
	SC_HAS_PROCESS(Ps_2s_th);
	Ps_2s_th (sc_core::sc_module_name name, double proportional_element);
	ab_port <T> &tmpl_port1;
	ab_port <T> &tmpl_port2;
	ab_port <thermal> &thrm_port;
	
private:
	void calculus ();
	const double P;
};


//	Implementation of class Pseries_2port_series_thermal  with equal norm. resistance

template <class T> Ps_2s_th<T>::Ps_2s_th (sc_core::sc_module_name name, double proportional_element) : P(proportional_element), tmpl_port1(base_class::port<T>(1)), tmpl_port2(base_class::port<T>(2)), thrm_port(base_class::port<thermal>(3))
{
	SC_METHOD(calculus);
	this->sensitive << this->activation;
	tmpl_port1 <<= 5;
	tmpl_port2 <<= 5;
}

template <class T> void Ps_2s_th<T>::calculus ()
{
	const double Pn = P / tmpl_port1->get_normalization();
	typename T::wave_type a1 = tmpl_port1->read();
	typename T::wave_type a2 = tmpl_port2->read();
	typename T::wave_type b1 = Pn*a1/(2+Pn) + 2.0*a2/(2+Pn);
	typename T::wave_type b2 = 2.0*a1/(2+Pn) + Pn*a2/(2+Pn);
	
	tmpl_port1->write(b1);
	tmpl_port2->write(b2);
	thrm_port->write(thrm_port->read() + (a1*a1-b1*b1 + a2*a2 - b2*b2));
}

//	Declaration of class PIp_2p_th

template <class T>
struct PIp_2p_th : wave_module<3, T, T, thermal>, analog_module
{
	typedef wave_module<> base_class;
	SC_HAS_PROCESS(PIp_2p_th);
	PIp_2p_th (sc_core::sc_module_name name, double proportional_element, double integrative_element);
	ab_port <T> &tmpl_port1;
	ab_port <T> &tmpl_port2;
	ab_port <thermal> &thrm_port;
public:
	void ics(double IC);
private:
	void calculus ();
	void field (double *var) const;
	const double P, I;
	double last_var;
};

//	Implementation of class PIp_2p_th with equal norm. resistance

template <class T> PIp_2p_th<T>::PIp_2p_th(sc_core::sc_module_name name, double proportional_element, double integrative_element) : analog_module(1), P(proportional_element),I (integrative_element), tmpl_port1(base_class::port<T>(1)), tmpl_port2(base_class::port<T>(2)), thrm_port(base_class::port<thermal>(3))
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	last_var = 0;
	tmpl_port1 <<= 5;
	tmpl_port2 <<= 5;
}

template <class T> void PIp_2p_th<T>::ics(double IC)
{
	this->ic(I*IC);
}

template <class T> void PIp_2p_th<T>::field (double *var) const
{
	typename T::wave_type a1 = tmpl_port1->read(), a2 = tmpl_port2->read();
	const double sqrt_P0 = tmpl_port1->get_normalization_sqrt();
	const double P0 = tmpl_port1->get_normalization();
	
	var[0] = (2 / sqrt_P0 * (a1+a2) - state[0] * (2*P+P0) / (I*P*P0) ) ;
	last_var = var[0];
}


template <class T> void PIp_2p_th<T>::calculus ()
{
	const double sqrt_P0 = tmpl_port1->get_normalization_sqrt();
	if (!set_steplimits_used) {
		const double tau = tmpl_port1->get_normalization()*I;
		set_steplimits(tau/100,tau/10);
	}
	
	while (step()) {
		typename T::wave_type a1 = tmpl_port1->read();
		typename T::wave_type a2 = tmpl_port2->read();
		typename T::wave_type b1 = (state[0] / (sqrt_P0 * I)- a1);
		typename T::wave_type b2 = (state[0] / (sqrt_P0 * I)- a2);
		typename thermal::wave_type cap_power = state[0]*last_var/this->dt/I;
		tmpl_port1->write(b1);
		tmpl_port2->write(b2);
		thrm_port->write(thrm_port->read() + (a1*a1+a2*a2-b1*b1-b2*b2-cap_power) * sqrt(thrm_port->get_normalization()));
	}
}



#endif //THERMAL_H