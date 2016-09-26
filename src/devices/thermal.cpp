// thermal.cpp:
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

#include "../../include/nature/thermal"
#include "../../include/units/constants"
#include "../../include/devices/thermal.h"

//  implementation of class Thermal_convection

Thermal_convector::Thermal_convector (sc_core::sc_module_name name, double area, double h) : A(area), H(h)
{
	SC_METHOD(calculus);
	this->sensitive << this->activation;
	this->port[0] <<= 1;
	this->port[1] <<= 1;
}

void Thermal_convector::calculus ()
{
	typename thermal::wave_type a1 = this->port[0]->read();
	typename thermal::wave_type a2 = this->port[1]->read();
	
	const double Pn = 1.0/A/H/this->port[0]->get_normalization();
	
	this->port[0]->write(Pn*a1/(2+Pn) + 2.0*a2/(2+Pn));
	this->port[1]->write(2.0*a1/(2+Pn) + Pn*a2/(2+Pn));
}

//  implementation of class Thermal_convection_var

Thermal_convector_var::Thermal_convector_var (sc_core::sc_module_name name, double area) : A(area)
{
	SC_METHOD(calculus);
	this->sensitive << this->activation;
	this->sensitive << h;
	this->port[0] <<= 1;
	this->port[1] <<= 1;
}

void Thermal_convector_var::calculus ()
{
	typename thermal::wave_type a1 = this->port[0]->read();
	typename thermal::wave_type a2 = this->port[1]->read();
	double ht = h.read();
	
	const double Pn = 1/A/ht/this->port[0]->get_normalization();
	
	this->port[0]->write(Pn*a1/(2+Pn) + 2.0*a2/(2+Pn));
	this->port[1]->write(2.0*a1/(2+Pn) + Pn*a2/(2+Pn));
}


