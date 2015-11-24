// threephase.cpp:
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

#include "../../include/units/constants"
#include "../../include/devices/threephase.h"

//	Implementation of class RCseries_load
// In the initial conditions the capacitors are supposed to be delta connected and one
// charged to VC0 so that the other two results charged to VC0 / 2
// the resulting voltages will be clarke * VC0*[0.5, 0, -0.5] = VC0*[ic_clk1, ic_clk2, 0]

const double  ic_clk1 = 6.12372435695795e-01;
const double  ic_clk2 = 3.53553390593274e-01;

RCs_load_tph::RCs_load_tph (sc_core::sc_module_name name, double proportional_element, double derivative_element) : analog_module(2, derivative_element / proportional_element /100, derivative_element / proportional_element/10), R(proportional_element), C(derivative_element)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	//	this->port[0] <<= 1.0;
}

void RCs_load_tph::ics(double VC0)
{
	double ICS[2] = {C*VC0*ic_clk1, C*VC0*ic_clk2};
	this->ic(ICS);
}

void RCs_load_tph::field (double *var) const
{
	const double R0 = this->port->get_normalization();
	const double sqrt_R0 = this->port->get_normalization_sqrt();
	std::complex <double> cvar = ( 2.0*this->port->read()*sqrt_R0 - std::complex <double>(state[0],state[1]) / C ) / (R + R0) ;
	var[0] = cvar.real();
	var[1] = cvar.imag();
}

void RCs_load_tph::calculus ()
{
	const double R0 = this->port->get_normalization();
	const double sqrt_R0 = this->port->get_normalization_sqrt();
	while (step()) {
		this->port->write( (this->port->read()*(R-R0) + std::complex <double>(state[0],state[1])*sqrt_R0 / C)/(R+R0) );
	}
}

//   Implementation of class RLCs_2s_tph:

RLCs_2s_tph::RLCs_2s_tph (sc_core::sc_module_name name, double proportional_element, double derivative_element, double integrative_element): analog_module(4, sqrt(derivative_element*integrative_element)/1000, sqrt(derivative_element*integrative_element)/10), P(proportional_element), D(derivative_element), I(integrative_element), p1(port(1)), p2(port(2))
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	p1 <<= 5.0;
	p2 <<= 5.0;
}

void RLCs_2s_tph::field (double *var) const
{
	const double sqrt_P0 = p1->get_normalization_sqrt();
	const double P0 = p1->get_normalization();
	std::complex <double> cvar1 = 2.0 * (p1->read() - p2->read()) * sqrt_P0 - std::complex <double>(state[0],state[1]) / D * (2.0 * P0 + P) - std::complex <double>(state[2],state[3]) / I;
	var[0] = cvar1.real();
	var[1] = cvar1.imag();
	var[2] = state[0] / D;
	var[3] = state[1] / D;
//	var[0] = 2 * (this->port[0]->read() - this->port[1]->read()) * sqrt_P0 - state[0] / D * (2 * P0 + P) - state[1] / I;
//	var[1] = state[0] / D;
}

void RLCs_2s_tph::calculus ()
{
	const double sqrt_P0 = p1->get_normalization_sqrt();
	while (step()) {
		p1->write(this->port[0]->read() - sqrt_P0 / D * std::complex <double>(state[0],state[1]));
		p2->write(this->port[1]->read() + sqrt_P0 / D * std::complex <double>(state[0],state[1]));
	}
}


//	Implementation of class RCs_2p with equal norm. resistance

RCs_2p_tph::RCs_2p_tph(sc_core::sc_module_name name, double proportional_element, double integrative_element) : analog_module(2, proportional_element * integrative_element /100, proportional_element * integrative_element/10), R(proportional_element), C(integrative_element)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	this->port[0] <<= 1.0;
	this->port[1] <<= 1.0;
}

void RCs_2p_tph::ics(double VC0)
{
	double ICS[2] = {C*VC0*ic_clk1, C*VC0*ic_clk2};
	this->ic(ICS);
}

void RCs_2p_tph::field (double *var) const
{
	std::complex <double> a1 = this->port[0]->read();
	std::complex <double> a2 = this->port[1]->read();
	const double sqrt_R0 = this->port[0]->get_normalization_sqrt();
	const double R0 = this->port[0]->get_normalization();
	std::complex <double> cvar = (2.0 * sqrt_R0 * (a1+a2) / (2.0*R+R0) - (2.0 * std::complex <double> (state[0],state[1])) / (2*R+R0) / C );
	var[0] =  cvar.real();
	var[1] =  cvar.imag();
}


void RCs_2p_tph::calculus ()
{
	const double sqrt_R0 = this->port[0]->get_normalization_sqrt();
	const double R0 = this->port[0]->get_normalization();
	while (step()) {
		std::complex <double> a1 = this->port[0]->read();
		std::complex <double> a2 = this->port[1]->read();
		std::complex <double> b1 = (2.0*R*a2 / (2.0*R+R0) - R0*a1 / (2.0*R+R0) + (sqrt_R0 * std::complex <double>(state[0],state[1])) / (2.0*R+R0) / C);
		std::complex <double> b2 = (2.0*R*a1 / (2.0*R+R0) - R0*a2 / (2.0*R+R0) + (sqrt_R0 * std::complex <double>(state[0],state[1])) / (2.0*R+R0) / C);
		this->port[0]->write(b1);
		this->port[1]->write(b2);
	}
}

//	Implementation of class NTC_2s_tph  with equal norm. resistance

NTC_2s_tph::NTC_2s_tph (sc_core::sc_module_name name, double proportional_element, double time_constant) : P(proportional_element), tau(time_constant)
{
	SC_METHOD(calculus);
	this->sensitive << this->activation;
	this->port[0] <<= 1;
	this->port[1] <<= 1;
}

void NTC_2s_tph::calculus ()
{
	double Pn = (P*exp(-sc_core::sc_time_stamp().to_seconds()/tau) + 100e-3) / this->port[0]->get_normalization();
//	double Pn = P / this->port[0]->get_normalization();
	std::complex <double>  a1 = this->port[0]->read();
	std::complex <double>  a2 = this->port[1]->read();

	
	this->port[0]->write(Pn*a1/(2+Pn) + 2.0*a2/(2+Pn));
	this->port[1]->write(2.0*a1/(2+Pn) + Pn*a2/(2+Pn));
}

//	Implementation of class RL_series_2_ports with equal norm. resistance

RLs_2s_tph::RLs_2s_tph(sc_core::sc_module_name name, double proportional_element, double derivative_element) : analog_module(2, derivative_element/proportional_element / 100, derivative_element/proportional_element/10), R(proportional_element), L(derivative_element)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	this->port[0]<<=5;
	this->port[1]<<=5;
}

void RLs_2s_tph::field (double *var) const
{
	const double sqrt_P0 = this->port[0]->get_normalization_sqrt();
	const double P0 = this->port[0]->get_normalization();
	
	std::complex <double> cvar = (2 * sqrt_P0 * (this->port[0]->read()-this->port[1]->read()) - std::complex <double>(state[0],state[1]) * (2*P0+R) / L);
	var[0] =  cvar.real();
	var[1] =  cvar.imag();
}


void RLs_2s_tph::calculus ()
{
	const double sqrt_P0 = this->port[0]->get_normalization_sqrt();
	state[0] = 0;
	while (step()) {
		this->port[0]->write((this->port[0]->read() - std::complex <double>(state[0],state[1]) * sqrt_P0 / L));
		this->port[1]->write((this->port[1]->read() + std::complex <double>(state[0],state[1]) * sqrt_P0 / L));
	}
}


//	Imnplementation of class LsCp_ladder_tph

LsCp_ladder_tph::LsCp_ladder_tph (sc_core::sc_module_name name, double s_L, double p_C) : 
	analog_module(4,sqrt(p_C*s_L)/1000, sqrt(p_C*s_L)/10),
	C(p_C),L(s_L),
	s(port(1)), p(port(2))
{
	SC_THREAD(calculus); sensitive << activation;
	s <<= 5.0;
	p <<= 5.0;
}


void LsCp_ladder_tph::field (double *var) const
{
	const double sqrt_R1 = s->get_normalization_sqrt();
	const double R1 = s->get_normalization();
	const double sqrt_R2 = p->get_normalization_sqrt();
	const double R2 = p->get_normalization();
	std::complex <double> v01 = std::complex<double>(state[2],state[3])/L + (2.0*p->read())/sqrt_R2 - std::complex<double>(state[0],state[1])/(R2*C);
	std::complex <double> v23 = 2.0*s->read()*sqrt_R1 - (R1/L)*std::complex<double>(state[2],state[3]) - (std::complex<double>(state[0],state[1]))/C;
	var[0] = v01.real();
	var[1] = v01.imag();
	var[2] = v23.real();
	var[3] = v23.imag();
}


void LsCp_ladder_tph::calculus ()
{
	const double sqrt_R1 = s->get_normalization_sqrt();
	const double sqrt_R2 = p->get_normalization_sqrt();
	
	state[0]=0;
	state[1]=0;
	state[2]=0;
	state[3]=0;
	while (step()) {
		s->write(s->read()-(sqrt_R1/L) * std::complex<double>(state[2],state[3]));
		p->write(-p->read()+std::complex<double>(state[0],state[1])/(sqrt_R2*C));
	}
}


//	Imnplementation of class RLs_load_tph

RLs_load_tph::RLs_load_tph(sc_core::sc_module_name name, double s_R, double s_L):analog_module(2, s_L / s_R /50,  s_L / s_R), R(s_R), L(s_L)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
}

void RLs_load_tph::field (double *var) const
{
	const double R0 = this->port->get_normalization();
  	const double sqrt_R0 = this->port->get_normalization_sqrt();
	
	std::complex <double> v01= 2.0*this->port->read()*sqrt_R0 - (std::complex<double>(state[0],state[1])*(R + R0) / L);
	
	var[0] = v01.real();
	var[1] = v01.imag();
}

void RLs_load_tph::calculus ()
{
	const double sqrt_R0 = this->port->get_normalization_sqrt();
	state[0] = 0;
	state[1] = 0;
	while (step()) {
		this->port->write(this->port->read() - std::complex<double>(state[0],state[1])*sqrt_R0/L);
	}
}


// implementation of class clarke2wye

clarke2wye::clarke2wye (sc_core::sc_module_name name) : a(port<electrical>(1)), b(port<electrical>(2)), c(port<electrical>(3)), clarke(port<threephase>(4))
{
	SC_METHOD(calculus);
	sensitive << activation;
	clarke <<= 1;
	a <<=1;
	b<<=1;
	c<<=1;

}

void clarke2wye::calculus ()
{
	double aa, ba, ca;
	std::complex<double> clarkea;
	const double S[5][5] ={
		{1.0/3.0,        1.0/3.0,        1.0/3.0,        +sqrt(2.0/3.0), 0},
 		{1.0/3.0,        1.0/3.0,        1.0/3.0,        -1.0/sqrt(6.0), +1.0/sqrt(2.0)},
		{1.0/3.0,        1.0/3.0,        1.0/3.0,        -1.0/sqrt(6.0), -1.0/sqrt(2.0)},
		{+sqrt(2.0/3.0), -1.0/sqrt(6.0), -1.0/sqrt(6.0), 0,              0},
		{0, 		     +1.0/sqrt(2.0), -1.0/sqrt(2.0), 0,              0}
	};

	aa=a->read();
	ba=b->read();
	ca=c->read();
	clarkea=clarke->read();
	a->write(S[0][0]*aa + S[0][1]*ba + S[0][2]*ca + S[0][3]*clarkea.real() + S[0][4]*clarkea.imag());
	b->write(S[1][0]*aa + S[1][1]*ba + S[1][2]*ca + S[1][3]*clarkea.real() + S[1][4]*clarkea.imag());
	c->write(S[2][0]*aa + S[2][1]*ba + S[2][2]*ca + S[2][3]*clarkea.real() + S[2][4]*clarkea.imag());
	clarke->write(std::complex<double>
			  (S[3][0]*aa + S[3][1]*ba + S[3][2]*ca + S[3][3]*clarkea.real() + S[3][4]*clarkea.imag(),
			   S[4][0]*aa + S[4][1]*ba + S[4][2]*ca + S[4][3]*clarkea.real() + S[4][4]*clarkea.imag()));
}


// implementation of class wye2delta

/*
wye2delta::wye2delta (sc_core::sc_module_name name) : wye(port(1)), delta(port(2))
{
	SC_METHOD(calculus);
	sensitive << activation;
	wye <<= 1.0;
	delta <<=1.0;
}

void wye2delta::calculus ()
{
	std::complex<double> A, a, Aa;

	const double N[2][2] = {
		{3.0/2.0, -sqrt(3)/2.0},
		{sqrt(3)/2.0, 3.0/2.0}
	};
	const double invN[2][2] = {
		{1.0/2.0, 1.0/(2*sqrt(3))},
		{-1.0/(2*sqrt(3)), 1.0/2.0}
	};
	
	a = wye->read();
	A = delta->read();
	Aa = 3.0*A - 2.0*a;
	wye->write(std::complex<double>(
									invN[0][0]*Aa.real() + invN[0][1]*Aa.imag(),
									invN[1][0]*Aa.real() + invN[1][1]*Aa.imag() ));
	delta->write(2.0*A - 4.0/3.0*a + std::complex<double>(
									    (N[0][0]*a.real() + N[0][1]*a.imag())/3.0,
									    (N[1][0]*a.real() + N[1][1]*a.imag())/3.0 ));
}

 */

// implementation of class c2vect

c2vect::c2vect (sc_core::sc_module_name name) : clarke(port<threephase>(1)), v1(port<electrical>(2)), v2(port<electrical>(3))
{
	SC_METHOD(flowclarke); sensitive << clarke;
	SC_METHOD(flowvect); sensitive << v1, v2;
	clarke <<= 1;
	v1 <<=1;
	v2<<=1;
}

void c2vect::flowclarke ()
{
	if (clarke->poll()) {
		v1->write((clarke->read()).real());
		v2->write((clarke->read()).imag());
	}
}

void c2vect::flowvect ()
{
	if (v1->poll()||v2->poll())  {
		tmp.real(v1->read());
		tmp.imag(v2->read());
		clarke->write(tmp);
	}
}