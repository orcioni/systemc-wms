// sigmadelta.cpp:
// Copyright (C) 2005-2016 Giorgio Biagetti and Simone Orcioni
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
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1601 USA.
 */

#define SC_INCLUDE_FX
#include <systemc.h>


// scaled integrator
struct sca_integrator : sc_core::sc_module
{
	sca_integrator (sc_core::sc_module_name instname, sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> scale);
	sc_in<sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > input;
	sc_in<bool> clk;
	sc_out<sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > output;
	
	sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> scale;
	adder adder1;
	delayff mem1;
	amp amp1, amp2;
	
	sc_signal<sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > s1, s2, s3;
};

sca_integrator::sca_integrator (sc_core::sc_module_name instname, sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> scale) :  adder1("ADDER1"), mem1("MEM1"), amp1("AMP1", scale),amp2("AMP2",scale)
{
	//Bindings
	amp1.in(input);
	amp1.out(s1);
	
	adder1.in_a(s1);
	adder1.in_b(s2);
	adder1.out(s3);
	
	mem1.in(s3);
	mem1.out(s2);
	mem1.clk(clk);
	
	amp2.in(s3);
	amp2.out(output);
}

// third order sigma delta - DC gain = 1

SC_MODULE(sigmadelta3rd)
{
	sc_core::sc_in<sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > inputSD;
	sc_core::sc_in<bool> clkSD;
	sc_core::sc_out<bool> control;
	sc_core::sc_out<bool> controlz;
	
	sca_integrator inta, intb, intc;
	subtractor suba, subb, subc;
	comp cmp;
	amp amp1, amp2;
	delayff ff1;
	
	sc_core::sc_signal<sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > delayed_fb,feedback,s1,s2,s3,s4,s5,s6,s7,s8;
	
	SC_CTOR(sigmadelta3rd) : inta("INTA", 0.5), intb("INTB", 0.5), intc("INTC", 1), suba("SUBa"), subb("SUBb"), subc("SUBc"), cmp("COMP1"), amp1("AMP1", 0.5), amp2("AMP2", 0.5),ff1("FF1")
	{
		
		//sc_core::sc_trace(f3, inputSD, "SD_IN");
		//sc_core::sc_trace(f3, s8, "COMP_IN");
		//sc_core::sc_trace(f3, control, "COMP_OUT");
		//sc_core::sc_trace(f3, feedback, "COMP_OUT");

		amp1.in(inputSD);
		amp1.out(s1);
		
		amp2.in(delayed_fb);
		amp2.out(s2);
		
		suba.in_plus(s1);
		suba.in_minus(s2);
		suba.out(s3);
		
		inta.input(s3);
		inta.output(s4);
		inta.clk(clkSD);
		
		subb.in_plus(s4);
		subb.in_minus(delayed_fb);
		subb.out(s5);
		
		intb.input(s5);
		intb.output(s6);
		intb.clk(clkSD);
		
		subc.in_plus(s6);
		subc.in_minus(delayed_fb);
		subc.out(s7);
		
		intc.input(s7);
		intc.output(s8);
		intc.clk(clkSD);
		cmp.input(s8);
		cmp.output(control);
		cmp.output_z(controlz);
		cmp.output_fx(feedback);
		ff1.in(feedback);
		ff1.out(delayed_fb);
		ff1.clk(clkSD);
		
	}
};