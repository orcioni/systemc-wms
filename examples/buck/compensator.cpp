// compensator.cpp:
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

SC_MODULE(adder4in)
{
	sc_core::sc_in<  sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > a, b, c, d;
	sc_core::sc_out< sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > out;
	
	SC_CTOR(adder4in)
	{
		SC_METHOD(doing);
		sensitive << a << b << c << d;
	}
	
	void doing()
	{
		sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> in_a = a.read();
		sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> in_b = b.read();
		sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> in_c = c.read();
		sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> in_d = d.read();
		
		out.write(in_a + in_b + in_c + in_d);
	}
	
};

SC_MODULE(compensator)
{
	sc_in <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > input;
	sc_in <bool> clock;
	sc_out <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > output;
	
	amp amp1, amp2, amp3;
	delayff a, b, c, d;
	adder4in adder;
	
	sc_signal <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > s1, s2, s3, s4, s5, s6, s7;
	
	SC_CTOR(compensator) :
	amp1("AMP1", sc_fixed_fast<wl,iwl,SC_RND, SC_SAT>(0.25)),
	amp2("AMP2", sc_fixed_fast<wl,iwl,SC_RND, SC_SAT>(-0.48388671875)),
	amp3("AMP3", sc_fixed_fast<wl,iwl,SC_RND, SC_SAT>(0.235595703125)),
	a("A"), b("B"), c("C"), d("D"),
	adder("ADDER")
	{ //bindings:
		amp1.in(input);
		amp1.out(s1);
		
		amp2.in(input);
		amp2.out(s2);
		
		amp3.in(input);
		amp3.out(s3);
		
		a.in(s2);
		a.out(s4);
		a.clk(clock);
		
		b.in(s3);
		b.out(s5);
		b.clk(clock);
		
		c.in(s5);
		c.out(s6);
		c.clk(clock);
		
		adder.a(s1);
		adder.b(s4);
		adder.c(s6);
		adder.d(s7);
		adder.out(output);
		
		d.in(output);
		d.out(s7);
		d.clk(clock);
		
	}
};
