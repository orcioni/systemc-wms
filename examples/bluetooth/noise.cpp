// noise.cpp:
// Copyright (C) 2007-2008 Giorgio Biagetti and Mauro Ballicchia
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

#include "noise.h"
#include "mtrand.h"

#include "units/constants"

static MTRand_int32 rnd_uint32;

noise::noise (double power, double frequency, double bandwidth) :
	variance(power * 64),       // because of 0.125fs filter
	omega(frequency * 2 * pi),
	period(0.125 / bandwidth)   // because of 4x oversampling
{
	t_sine = 0.1 / omega;
	t_rand = 0;
	n = 0;
	i_old = i_rand = 0;
	q_old = q_rand = 0;
}

noise::IIR_Filter_8::IIR_Filter_8 ()
{
	za1 = za2 = zb1 = zb2 = zc1 = zc2 = zd1 = zd2 = 0;
}

double noise::IIR_Filter_8::operator () (double x)
{
	// 8th-order elliptic filter (fc=0.125, pass-ripple=0.1dB, stop-ripple=60dB)
	const double ba1 = -0.18817331904289, aa1 = -1.67636669743467, aa2 = +0.71284003477907, ga = 0.01990026467942;
	const double bb1 = -1.56588162146167, ab1 = -1.72987556119779, ab2 = +0.81668225349439, gb = 0.19996087838732;
	const double bc1 = -1.74582167183525, ac1 = -1.78298229774416, ac2 = +0.91605793207351, gc = 0.52355224495417;
	const double bd1 = -1.78728634932500, ad1 = -1.82131023237745, ad2 = +0.97708682198289, gd = 0.73233000849321;

	double ya = za1 + x;
	za1 = za2 + ba1 * x - aa1 * ya;
	za2 = x - aa2 * ya;
	ya *= ga;
	double yb = zb1 + ya;
	zb1 = zb2 + bb1 * ya - ab1 * yb;
	zb2 = ya - ab2 * yb;
	yb *= gb;
	double yc = zc1 + yb;
	zc1 = zc2 + bc1 * yb - ac1 * yc;
	zc2 = yb - ac2 * yc;
	yc *= gc;
	double yd = zd1 + yc;
	zd1 = zd2 + bd1 * yc - ad1 * yd;
	zd2 = yc - ad2 * yd;
	return yd * gd;
}

double noise::operator () (double &t) const
{
	while (t >= t_rand) { // time to get a new random sample
		i_old = i_rand;
		q_old = q_rand;
		if (n == 0) {
			double x, y, r;
			do {
				x = (double) rnd_uint32() / 2147483648U - 1;
				y = (double) rnd_uint32() / 2147483648U - 1;
				r = x * x + y * y;
			} while (r > 1 || r == 0);
			r = sqrt(-variance * log(r) / r);
			i_rand = filter1(x * r);
			q_rand = filter2(y * r);
		} else {
			i_rand = filter1(0);
			q_rand = filter2(0);
		}
		if (!(++n % 4)) n = 0;
		t_rand += period;
		// This uses a filter to do a 4x oversampling, and then a linear interpolation.
	}
	double tau = (t_rand - t) / period;
	double i = i_old * tau + i_rand * (1 - tau);
	double q = q_old * tau + q_rand * (1 - tau);
	double y = i * sin(omega * t) + q * cos(omega * t);
	t += t_sine;
	return y;
}
