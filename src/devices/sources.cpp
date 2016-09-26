// sources.cpp:
// Copyright (C) 2004-2006 Giorgio Biagetti and Simone Orcioni
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


#include <string>

#include "../../include/sys/sources"
#include "../../include/units/constants"


// Implemetation of class "file":

file::file (const char *filename, int column) : column(column)
{
	datasource = new std::ifstream(filename);
	next_time = -1;
	next_value = 0;
	curr_value = 0;
}

file::file (file const &source) : column(source.column)
{
	datasource = source.datasource; // TODO: FIXME!
	curr_time = -2;
	next_time = -1;
	next_value = 0;
	curr_value = 0;
}

file::~file ()
{
	//TODO: copy constructor make the following not work... FIXME!
//	delete (std::ifstream *) datasource;
}

double file::operator () (double &t) const
{
	// TODO: the following does not work well for discontinuous data... FIX!
	for (std::string line; t >= next_time && std::getline(*(std::ifstream *) datasource, line); ) {
		if (line.empty() || line[0] == '#') continue;
		curr_time = next_time;
		curr_value = next_value;
		std::istringstream str(line);
		str >> next_time;
		for (int i = 0; i < column; ++i) str >> next_value;
	}
	double tau = (t - curr_time) / (next_time - curr_time);
	if (t >= next_time)
		t += 3600;
	else
		t = next_time;
	return curr_value * tau + next_value * (1 - tau);
}

// Implemetation of class "dc":

dc::dc (double amplitude)
{
	this->amplitude = amplitude;
}

double dc::operator () (double &t) const
{
	t += 1;
	return amplitude;
}

// Implemetation of class "sine":

sine::sine (double amplitude, double freq, double phase, int oversample)
{
	this->amplitude = amplitude;
	this->omega = 2 * pi * freq;
	this->phase = phase;
	this->tstep = 0.5 / oversample / freq;
}

double sine::operator () (double &t) const
{
	double y = amplitude * sin(omega * t + phase);
	t += tstep;
	return y;
}

// Implemetation of class "ramp":

ramp::ramp (double rate, double init, double delay, double tstep)
{
	this->rate = rate;
	this->init = init;
	this->delay = delay;
	this->tstep = tstep;
}

double ramp::operator () (double &t) const
{
	if (t < delay){
		t = delay;
		return 0;
	} else {
		double y = init + rate*t;
		t += tstep;
		return y;
	}
}

// Implemetation of class "step"

step::step (double amplitude, double delay)
{
	this->amplitude = amplitude;
	this->delay = delay;
}

double step::operator () (double &t) const
{
	if (t < delay){ 
		t = delay;
		return 0;
	} else {
		t += 3600;
		return amplitude;
	}
}

// Implemetation of class "bool_step"

bool_step::bool_step (double delay)
{
	this->delay = delay;
}

bool bool_step::operator () (double &t) const
{
	if (t < delay){
		t = delay;
		return false;
	} else {
		t += 3600;
		return true;
	}
}


// Implemetation of class "sine_threephase":

sine_threephase::sine_threephase (double amplitude, double freq, double phase)
{
	this->amplitude = sqrt(3.0/2.0)*amplitude;
	this->omega = 2 * pi * freq;
	this->phase = phase;
}

std::complex <double> sine_threephase::operator () (double &t) const
{
	std::complex <double> y(amplitude * cos(omega * t + phase), amplitude * sin(omega * t + phase));
	t += 0.01 / omega;
	return y;
}

// Implemetation of class "sine_threephase_d":

sine_threephase_d::sine_threephase_d (double amplitude, double freq, double phase)
{
	this->amplitude = sqrt(3.0/2.0)*amplitude;
	this->omega = 2 * pi * freq;
	this->phase = phase;
}

std::complex <double> sine_threephase_d::operator () (double &t) const
{
	std::complex <double> y(amplitude * cos(omega * t + phase), amplitude * sin(omega * t + phase));
	t += 1e-6;
	return y;
}

// Implementation of class "sawtooth":
sawtooth::sawtooth (double amplitude, double freq, double phase, double bias ,int oversample)
{
	this->amplitude = amplitude;
	this->_freq = freq;
	this->phase = phase+pi;
	this->_bias = bias;
	this->tstep = 0.5 / oversample / freq;
}

double sawtooth::operator () (double &t) const
{
	double y =amplitude * fmod(t*_freq+phase/(2*pi),1)-amplitude/2+_bias;
	t += tstep;
	return y;
}

//Implementation of class "triangular":
triangular::triangular (double amplitude, double freq, double phase, double bias ,int oversample)
{
	this->amplitude = amplitude;
	this->_freq = freq;
	this->phase = phase+pi/2;
	this->_bias = bias;
	this->tstep = 0.5 / oversample / freq;
}

double triangular::operator () (double &t) const
{ double y;
	if (fmod(t*_freq+(phase)/(2*pi),1) <= 0.5 )
		y = ( 2*amplitude * fmod(t*_freq+(phase)/(2*pi),1))-(amplitude/2)+_bias;
	else
		y = ( 2*amplitude * (1-fmod(t*_freq+(phase)/(2*pi),1)))-(amplitude/2)+_bias;
	t += tstep;
	return y;
}