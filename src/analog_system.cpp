// AnalogSys.cpp
// Copyright (C) 2003 Giorgio Biagetti and Marco Caldari
// Copyright (C) 2004-2006 Giorgio Biagetti
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

#include "systemc.h"
#include "analog_system"
#include <algorithm>
#include <cstring>

// ATTENTION: multistep_b must not be shorter than multistep_c!
#ifndef ODE_METHOD
#error No ODE solver method specified in ODE_METHOD
#elif ODE_METHOD==1 // Euler ODE solver coefficients:
const double analog_module::multistep_a[] = {1};
const double analog_module::multistep_b[] = {1};
const double analog_module::multistep_c[] = {};
#elif ODE_METHOD==2 // Central Differences ODE solver coefficients:
const double analog_module::multistep_a[] = {0, 1};
const double analog_module::multistep_b[] = {2};
const double analog_module::multistep_c[] = {};
#elif ODE_METHOD==4 // Adams-Bashforth multistep ODE solver coefficients:
const double analog_module::multistep_a[] = {1};
const double analog_module::multistep_b[] = {+55/24.0, -59/24.0, +37/24.0, -9/24.0};
const double analog_module::multistep_c[] = {};
#elif ODE_METHOD==8 // Adams-Moulton predictor-corrector ODE solver coefficients:
const double analog_module::multistep_a[] = {1};
const double analog_module::multistep_b[] = {+55/24.0, -59/24.0, +37/24.0, -9/24.0};
const double analog_module::multistep_c[] = { +9/24.0, +19/24.0,  -5/24.0, +1/24.0};
#elif ODE_METHOD==16 // Bulirsch-Stoer adaptive step ODE solver
#else
#error Unknown ODE_METHOD specified
#endif

namespace {
template <typename T> T *init_array (int size, T val)
{
	T *array = new T[size];
	for (int i = 0; i < size; ++i) array[i] = val;
	return array;
}
} // namespace



#ifndef ODE_METHOD
#error No ODE solver method specified in ODE_METHOD
#elif ODE_METHOD==1||ODE_METHOD==2||ODE_METHOD==4||ODE_METHOD==8

const int analog_module::multistep_order_a = sizeof analog_module::multistep_a / sizeof (double);
const int analog_module::multistep_order_b = sizeof analog_module::multistep_b / sizeof (double);
const int analog_module::multistep_order_c = sizeof analog_module::multistep_c / sizeof (double);

analog_module::analog_module (int size, double min, double max) :
	abstol(init_array(size, 1e-12)), reltol(1e-6), mintol(1e-1), size(size)
{
  	dt_factor = 1.1; // factor must be > 1, otherwise may loop forever!
	direction = init_array(size * multistep_order_b, 0.0);
	state = init_array(size * (multistep_order_a + !!multistep_order_c), 0.0);
	set_steplimits(min, max);
	set_steplimits_used = false;
	for(int i = 0; i < size; ++i) state[i] = 0.0;
}

analog_module::~analog_module ()
{
	delete [] state;
	delete [] direction;
	delete [] abstol;
}

void analog_module::set_steplimits (double min, double max)
{
	dt_min = min;
	dt_max = min > max ? min : max;
	dt_aux = dt_min;
	dt = -dt_min;
	set_steplimits_used = true;
}

void analog_module::set_tolerances (double const *abstol, double reltol, double mintol)
{
	for (int i = 0; i < size; ++i)
		this->abstol[i] = abstol[i];
	this->reltol = reltol;
	this->mintol = mintol;
}

void analog_module::set_tolerances (double abstol, double reltol, double mintol)
{
	for (int i = 0; i < size; ++i)
		this->abstol[i] = abstol;
	this->reltol = reltol;
	this->mintol = mintol;
}

void analog_module::ic (double const *icvect)
{
	for (int i = 0; i < size; ++i)
		this->state[i] = icvect[i];
}

void analog_module::ic (double const icval)
{
		this->state[0] = icval;
}

bool analog_module::step ()
{	
	if (dt < 0) {
	  //	  dt = 0;
	            dt=dt_min;
	  //  dt=dt_aux=50e-9;
		return true;
	}

	// Evaluate direction of state change:
	field(direction);

	// Compute appropriate step size:

	enum amount {low = -1, ok = 0, high = 1};
	for (amount variation = high; variation > ok; )
	{
		dt = dt_aux;
		variation = ok;
		for (int i = 0; i < size; ++i)
		{
			bool over  = fabs(direction[i] * dt) > (fabs(state[i]) * reltol + abstol[i]);
			bool under = fabs(direction[i] * dt) < (fabs(state[i]) * reltol + abstol[i]) * mintol;
			if (over) variation = high;
			if (under && variation < high) variation = low;
		}
		switch (variation)
		{
		case ok:
			// let things stay as they are...
			continue;
		case low:
			if (dt < dt_max && (dt_aux *= dt_factor) > dt_max) dt_aux = dt_max;
			break;
		case high:
			if (dt > dt_min && (dt_aux /= dt_factor) < dt_min) dt_aux = dt_min;
			break;
		}
		if (dt == dt_aux) variation = ok;
	}


	// Sleep:
	sc_core::sc_time t1 = sc_core::sc_time_stamp();
	sc_core::wait(dt, sc_core::SC_SEC, activation);
	sc_core::sc_time t2 = sc_core::sc_time_stamp();
	if (t2 == t1) return true;
	double elapsed_dt = (t2 - t1).to_seconds(); // may be different than dt if wakened up by activation event
	if (elapsed_dt < dt / dt_factor) dt_aux = dt_min;

	// Update state:
	if (multistep_order_a > 1) // otherwise assume multistep_a[0] == 1;
	{	// WARNING: currently it is not very stable!
		int len = (multistep_order_a - 1) * size;
		for (int i = 0; i < size; ++i) state[i + len] *= multistep_a[multistep_order_a - 1];
		for (int j = 0; j < multistep_order_a - 1; ++j)
			for (int i = 0; i < size; ++i)
				state[i + len] += state[i + j * size] * multistep_a[j];
		// roll memory:
		for (int j = multistep_order_a - 1; j > 0; --j)
			for (int i = 0; i < size; ++i)
				std::swap(state[i + j * size], state[i + (j - 1) * size]);
	}
	// Backup state if corrector is enabled:
	if (multistep_order_c > 0)
		for (int i = 0; i < size; ++i)
			state[i + multistep_order_a * size] = state[i];
	for (int i = 0; i < size; ++i)
		for (int j = 0; j < multistep_order_b; ++j)
			state[i] += direction[i + j * size] * multistep_b[j] * elapsed_dt;

	// Roll past field vector:
	memmove(direction + size, direction, sizeof (double) * size * (multistep_order_b - 1));
	// Apply corrector:
	if (multistep_order_c > 0)
	{
		field(direction);
		for (int i = 0; i < size; ++i)
		{
			state[i] = state[i + multistep_order_a * size];
			for (int j = 0; j < multistep_order_c; ++j)
				state[i] += direction[i + j * size] * multistep_c[j] * elapsed_dt;
		}
	}


	return true;
}

#elif ODE_METHOD==16

/***********************************************/
/*** Bulirsch-Stoer adaptive step ODE solver ***/
/***********************************************/
#include "bulirsch_stoer"

analog_module::analog_module (int size, double min, double max):
	abstol(init_array(size, 1e-12)), reltol(1e-6), size(size),h(1e-6) 
{
	state = init_array(size , 0.0);
	K = gsl_odeiv_step_bsimp;	//set step type to Bulirsch-Stoer 
	s = gsl_odeiv_step_alloc (K, size);	//allocate step type
	c = gsl_odeiv_control_standard_new (1e-13, 0.0, 1, 1); 	//create the control system
	e = gsl_odeiv_evolve_alloc (size);
}

analog_module::~analog_module ()
{
	delete [] state;
	delete [] abstol;
	gsl_odeiv_evolve_free (e);
	gsl_odeiv_control_free (c);
	gsl_odeiv_step_free (s);
}

int analog_module::func (double t, const double y[], double f[])
	{
		field(f);
		return GSL_SUCCESS;
	}

int analog_module::jac (double t, const double y[], double *dfdy, double dfdt[])
	{
		gsl_matrix_view dfdy_mat = gsl_matrix_view_array (dfdy, size, size);
		gsl_matrix * m = &dfdy_mat.matrix;
		jacobian(m, dfdt);
		return GSL_SUCCESS;
	}
	

void analog_module::set_steplimits (double min, double max) {}

void analog_module::set_tolerances (double const *abstol, double reltol, double mintol) {}

void analog_module::set_tolerances (double abstol, double reltol, double mintol) {}

void analog_module::set_control_param (double abstol, double reltol, double a_y, double a_dydt){
 
	  std_control_state_t * s = (std_control_state_t *) c;
	  s->eps_rel = reltol;
	  s->eps_abs = abstol;
	  s->a_y = a_y;
	  s->a_dydt = a_dydt;
}

void analog_module::set_step (double step){
	h=step;
}

#endif
