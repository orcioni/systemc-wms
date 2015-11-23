// self_oscillating_HB.cpp:
// Copyright (C) 2005-2006 Giorgio Biagetti and Simone Orcioni
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

#include <systemc.h>
#include "devices/electrical_oneport.h"
#include "devices/electrical_twoport.h"
#include "sys/sources"
#include "sys/probes"
#include "nature/electrical"
#include "units/electrical"
#include "units/constants"

#include "tab_trace"


//Inverter module

SC_MODULE(inverter)
{		
  sc_core::sc_in  <bool> in;	//input bit
  sc_core::sc_out <bool> out;	//output bit

	void doing()
	{ 		
		out.write(!(in.read()));
	}

	SC_CTOR(inverter) 
	{
		SC_METHOD(doing);	
		this->sensitive << in;
	}
};


// Implementation of a modified version of RLCs_2s
// to simulate a suddenly change in the inductance, as required by the paper example

// RLCs_2s   -- RLC series- two-port in series between port1 and port2


template <class T1>
struct RLCs_2sm : wave_module <2, T1>, analog_module
{
  SC_HAS_PROCESS(RLCs_2sm);
  RLCs_2sm (sc_core::sc_module_name name, double resistance, double inductance, double capacitance); 
private:
  void field (double *var) const;
  void calculus ();
  double R, L, C;           
};

//   Implementation of the module RLC series 2-ports:

template <class T> RLCs_2sm<T>::RLCs_2sm (sc_core::sc_module_name name,double resistance, double inductance, double capacitance): analog_module(2, sqrt(inductance*capacitance)/100, sqrt(inductance*capacitance)/10),R(resistance), L(inductance), C(capacitance)
{
  SC_THREAD(calculus);
  this->sensitive << activation;
}

template <class T> void RLCs_2sm<T>::field (double *var) const
{
  double a1 = this->port[0]->read();
  double a2 = this->port[1]->read();
  static const double sqrt_R0 = this->port[0]->get_normalization_sqrt();
  static const double R0 = this->port[0]->get_normalization();
  var[0] = 2 * (a1 - a2) * sqrt_R0 - state[0] / L * (2 * R0 + R) - state[1] / C;
  var[1] = state[0] / L;
}

template <class T> void RLCs_2sm<T>::calculus ()
{
  const double sqrt_R0 = this->port[0]->get_normalization_sqrt();
  const double R0 = this->port[0]->get_normalization();
  state[0] = 0;
  state[1] = 0;
  while (step()) {
    if(sc_core::sc_time_stamp().to_seconds()>200e-06)
      //	  L=80e-06*(1-(sc_core::sc_time_stamp().to_seconds()-200e-06)/200e-06);
      L=40e-06;
    double a1 = this->port[0]->read();
    double a2 = this->port[1]->read();
    double b1 = a1 - sqrt_R0 / L * state[0];
    double b2 = a2 + sqrt_R0 / L * state[0];
    this->port[0]->write(b1);
    this->port[1]->write(b2);
  }
}


// main program:

int sc_main (int argc, char *argv[])
{
  sc_core::sc_set_time_resolution(1.0, sc_core::SC_NS);
  sc_core::sc_signal <bool> compout, pulse1, pulse2;
  ab_signal <electrical, series> shunt2;
  sc_core::sc_signal <electrical::wave_type> in;
  ab_signal <electrical, parallel> mains(10 ohm), rectified(10 ohm);

  sc_core::sc_trace_file *f = create_tab_trace_file("Load_traces");
//  sc_core::sc_trace_file *g = create_tab_trace_file("mains");
  //f->delta_cycles(true);

//  mains.trace(g,"MAINS");

  //sc_core::sc_trace(g, compout, "Output");

  generator <electrical::wave_type> signal_source("SOURCE1", sine(sqrt(2) * 230, 50 Hz, pi/2));
  signal_source(in);

  source <electrical> wave_source("SOURCE2", cfg::across);
  wave_source(mains, in);

  ideal_rectifier <electrical> bridge("BRIDGE");
  bridge(mains, rectified);

  RCs_load line_filter("FILTER", 1 ohm, 5 uF);
  line_filter(rectified);

  ab_signal <electrical, half_bridge> ponte;

  ponte.trace(f, "CHOPPED");
  ab_connector <electrical> vdd("VDD");
  vdd(ponte->mains, rectified);

  onoff_switchd switch1("SWITCH1");
  switch1(-ponte->up, pulse1);

  onoff_switchd switch2("SWITCH2");
  switch2(-ponte->down, compout);

//  ab_connector <electrical> load("LOAD");
//  load(ponte->load, chopped);

  RLCs_2s load1("LOAD1", 4 ohm, 80 uH, 740 nF);
  load1(ponte->load, shunt2);

//  RCs_load snubber("SNUBBER",10 ohm, 10 nF);
//  snubber(chopped);

  comparator <electrical, delayed> cmp("CMP",2e-8,cfg::through);

  //    icomp_st <electrical> cmp("CMP",0, 2e-8);
  //  icomp_st_hst <electrical> cmp("CMP",-0.1,0.1, 2e-7);

  cmp(shunt2, compout);

  inverter inv1("INV1");
  inv1(compout,pulse1);


  //  onoff_switchd_2s switch2("SWITCH2");
  //  switch2 (shunt1, chopped, compout);


  //  onoff_switchd_2s switch1("SWITCH1");
  //  switch1(chopped, rectified, pulse1);

  //  onoff_switchd_2s switch2("SWITCH2");
  //  switch2 (shunt1, chopped, compout);


  sc_core::sc_start(sc_core::sc_time(600,sc_core::SC_US));
  close_tab_trace_file(f);
//  close_tab_trace_file(g);
  return 0;
}
