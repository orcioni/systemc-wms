// noise.h:
// Copyright (C) 2007-2008 Giorgio Biagetti
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

#ifndef NOISE_H
#define NOISE_H

#include "sys/sources"

struct noise : function_base <double>
{	CLONABLE
	noise (double power, double frequency, double bandwidth);
	double operator () (double &t) const;
private:
	const double variance, omega, period;
	struct IIR_Filter_8
	{
		IIR_Filter_8 ();
		double operator () (double x);
	private:
		double za1, za2, zb1, zb2, zc1, zc2, zd1, zd2;
	} mutable filter1, filter2;
	mutable double t_sine;
	mutable double t_rand;
	mutable double i_rand, q_rand;
	mutable double i_old,  q_old;
	mutable int n;
};

#endif
