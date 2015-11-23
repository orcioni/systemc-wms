// RF_modules.h:
// Copyright (C) 2007-2008 Mauro Ballicchia
// Copyright (C) 2014 Simone Orcioni
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

#ifndef RF_MODULES_H
#define RF_MODULES_H

#include <systemc.h>
#include <iostream>
#include <cmath>
#include "sys/probes"
#include "nature/electrical"
#include "devices/electrical_twoport.h"
#include "devices/electrical_oneport.h"
#include "units/constants"
#include "units/decibel"
#include "noise.h"

#include "tab_trace"


// Instantaneous model of an RF amplifier:

struct amplifier_scatter : wave_module<2, electrical>
{
	SC_HAS_PROCESS(amplifier_scatter);
	amplifier_scatter (sc_core::sc_module_name name,
		scatter S11, scatter S12, scatter S21, scatter S22,
		double IIP3, double F_noise, double f0, double band,
		double R01 = 50 ohm, double R02 = 50 ohm
	) :
		S11(S11), S12(S12), S21(S21), S22(S22),
		a(0.5 * sqrt(2 * IIP3) * S21), F_noise(F_noise),
		n1(k * 290 * band * (F_noise - 1), f0, band)
	{
		SC_METHOD(calculus); sensitive << activation;
		port[0] <<= R01; // set normalization resistance on  input port
		port[1] <<= R02; // set normalization resistance on output port
	}
private:
	const double S11, S12, S21, S22, F_noise;
	const electrical::wave_type a;
	noise n1;
	void calculus ()
	{
		double t = sc_core::sc_time_stamp().to_seconds();
		double a1 = port[0]->read() + (F_noise == 1 ? 0 : n1(t));
		double a2 = port[1]->read();
		double y1 = S11 * a1 + S12 * a2;
		double y2 = S21 * a1 + S22 * a2;
		port[0]->write(y1);
		port[1]->write(a * tanh(y2 / a));
	}
};


// Istantaneous model of Mixer

struct mixer_scatter : wave_module<2, electrical>
{
	SC_HAS_PROCESS(mixer_scatter);
	mixer_scatter (sc_core::sc_module_name name,
		scatter S11, scatter S12, scatter c_gain, scatter S22, scatter SLO,
		double POWLO, double IIP3, double F_noise, double f0, double band,
		double R01 = 50 ohm, double R02 = 50 ohm
	) :
		S11(S11), S12(S12), K(c_gain + 6.0 dB), S22(S22), SLO(SLO), ALO(sqrt(2 * POWLO)),
		// The increase of 6dB in ''c_gain'' has been introduced in order to have 
		// the requested power conversion gain in one-half of the output signal bandwidth.
		a(0.5 * sqrt(2 * IIP3) * c_gain), F_noise(F_noise),
		n1(k * 290 * band * (F_noise - 1), f0, band)
	{
		SC_METHOD(calculus); sensitive << activation << LO;
		port[0] <<= R01;
		port[1] <<= R02;
	}
	sc_core::sc_in <electrical::wave_type> LO;
private:
	const double S11, S12, K, S22, SLO, ALO, F_noise;
	const electrical::wave_type a;
	noise n1;
	void calculus ()
	{
		double t = sc_core::sc_time_stamp().to_seconds();
		double a1 = port[0]->read() + (F_noise == 1 ? 0 : n1(t));
		double a2 = port[1]->read();
		double aLO = LO->read();
		double y1 = S11 * a1 + SLO * aLO + S12 * a2;
		double y2 = (1.0 + S22) * K * (a1 + S11 * a1 + SLO * aLO + S12 * a2) * aLO / ALO + S22 * a2;
		port[0]->write(y1);
		port[1]->write(a * tanh(y2 / a));
	}
};


// Instantaneous model of a noisy radio channel:

struct radio_channel : wave_module<2, electrical>
{
	SC_HAS_PROCESS(radio_channel);
	radio_channel (sc_core::sc_module_name name,
		double distance, double antenna_gain,
		double F_noise1, double F_noise2,
		double f0, double band
	) :
		wave_length(c / f0),
		attenuation(1 / (4 * pi * distance / wave_length)),
		S11(0), S12(antenna_gain * attenuation), S21(S12), S22(0),
		n1(k * 290 * band * (F_noise1 - 1), f0, band),
		n2(k * 290 * band * (F_noise2 - 1), f0, band)
	{
		SC_METHOD(calculus); sensitive << activation;
		port[0] <<= 50 ohm;
		port[1] <<= 50 ohm;
	}
private:
	const double wave_length, attenuation;
	const double S11, S12, S21, S22;
	noise n1, n2;
	void calculus ()
	{
		double t = sc_core::sc_time_stamp().to_seconds();
		double a1 = port[0]->read();
		double a2 = port[1]->read();
		double y1 = S11 * a1 + S12 * a2;
		double y2 = S21 * a1 + S22 * a2;
		port[0]->write(y1 + n1(t));
		port[1]->write(y2 + n2(t));
	}
};


// Class TX:

struct TX : sc_core::sc_module
{
	// input/output ports:
	ab_port <electrical> load;
	sc_core::sc_in <electrical::wave_type> in_I, in_Q;

	// internal constants:
	const double LOpower;

	// internal signals:
	sc_core::sc_signal <electrical::wave_type> LO1, LO2;
	ab_signal <electrical, parallel> wave_I, wave_Q, lpffilt_I, lpffilt_Q, sum;

	// internal modules:
	source <electrical> wave_source_I, wave_source_Q;
	generator <electrical::wave_type> vco1, vco2;
	LsCp_ladder lpf1, lpf2;
	mixer_scatter mixer1, mixer2;
	amplifier_scatter pa;

	// implementation:
	TX (sc_core::sc_module_name name) :
		LOpower(27.0 dBm),
		wave_I(50 ohm, 1e-6, 0),
		wave_Q(50 ohm, 1e-6, 0),
		lpffilt_I(50 ohm, 1e-6, 0),
		lpffilt_Q(50 ohm, 1e-6, 0),
		sum(50 ohm, 6e-6, 0),
		wave_source_I("SOURCE11", cfg::wave),
		wave_source_Q("SOURCE12", cfg::wave),
		lpf1("LPF1", 2.814 uH, 1.125 nF),
		lpf2("LPF2", 2.814 uH, 1.125 nF),
		vco1("LO1", sine(sqrt(2 * LOpower), 2.450 GHz, 0.0 * pi, 20)),
		vco2("LO2", sine(sqrt(2 * LOpower), 2.450 GHz, 0.5 * pi, 20)),
		mixer1("MIXER1", -15.0 dB, -90.0 dB, 2.0 dB, -15.0 dB, -86.0 dB, LOpower, 15.0 dBm, 0.0 dB, 2.450 GHz, 1 MHz),
		mixer2("MIXER2", -15.0 dB, -90.0 dB, 2.0 dB, -15.0 dB, -86.0 dB, LOpower, 15.0 dBm, 0.0 dB, 2.450 GHz, 1 MHz),
		pa("PA", -15.0 dB, -40.0 dB, 18.0 dB, -15.0 dB, 18.0 dBm, 0.0 dB, 2.450 GHz, 1 MHz)
	{
		sc_core::sc_trace_file *f = create_tab_trace_file("TX-trace", 1e-10);
		sum.trace(f, "Upconverted");
		wave_I.trace(f, "InputI");
		wave_Q.trace(f, "InputQ");
		lpffilt_I.trace(f, "FilteredI");
		lpffilt_Q.trace(f, "FilteredQ");

		wave_source_I(wave_I, in_I);
		wave_source_Q(wave_Q, in_Q);
		vco1(LO1);
		vco2(LO2);
		lpf1(wave_I, lpffilt_I);
		lpf2(wave_Q, lpffilt_Q);
		mixer1(lpffilt_I, sum, LO1);
		mixer2(lpffilt_Q, sum, LO2);
		pa(sum, load);
	}
};


// Class RX:

struct RX : sc_core::sc_module
{
	// input/output ports:
	ab_port <electrical> source;
	sc_core::sc_out <electrical::wave_type> out_I, out_Q;

	// internal constants:
	const double LOpower;

	// internal signals:
	sc_core::sc_signal <electrical::wave_type> LO1, LO2;
	ab_signal <electrical, parallel> splitter;
	ab_signal <electrical, parallel> downconv_I, downconv_Q;
	ab_signal <electrical, parallel> filtered_I, filtered_Q;
	ab_signal <electrical, series> amplified_I, amplified_Q;

	// internal modules:
	iprobe <electrical> DSPsign_I, DSPsign_Q;
	generator <electrical::wave_type> vco1, vco2;
	amplifier_scatter lna;
	mixer_scatter mixerlpf1, mixerlpf2;
	LsCp_ladder lpf1,lpf2;
	amplifier_scatter pga_I, pga_Q;
	C_load CI, CQ;

	// implementation:
	RX (sc_core::sc_module_name name) :
		LOpower(27.0 dBm),
		splitter(50 ohm, 5e-7, 0),
		downconv_I(50 ohm, 1.41e-6, 0), downconv_Q(50 ohm, 1.41e-6, 0),
		filtered_I(50 ohm, 6.3e-7, 0), filtered_Q(50 ohm, 6.3e-7, 0),
		amplified_I(50 ohm, 2.5e-6, 0), amplified_Q(50 ohm, 2.5e-6, 0),
		lna("LNA", -15.0 dB, -40.0 dB, 18.0 dB, -15.0 dB, 0.0 dBm, 4.0 dB, 2.450 GHz, 1 MHz),
		vco1("LO1", sine(sqrt(2 * LOpower), 2.448 GHz, 0.0 * pi, 20)),
		vco2("LO2", sine(sqrt(2 * LOpower), 2.448 GHz, 0.5 * pi, 20)),
		mixerlpf1("MIXER1", -15.0 dB, -90.0 dB, 12.0 dB, -15.0 dB, -90.0 dB, LOpower, 12.0 dBm, 24.0 dB, 2.450 GHz, 1 MHz),
		mixerlpf2("MIXER2", -15.0 dB, -90.0 dB, 12.0 dB, -15.0 dB, -90.0 dB, LOpower, 12.0 dBm, 24.0 dB, 2.450 GHz, 1 MHz),
		lpf1("LPF1", 2.814 uH, 1.125 nF),
		lpf2("LPF2", 2.814 uH, 1.125 nF),
		pga_I("PGA1", -150.0 dB, -150.0 dB, 12.0 dB, -150.0 dB, 140.0 dBm, 34.0 dB, 2 MHz, 1 MHz),
		pga_Q("PGA2", -150.0 dB, -150.0 dB, 12.0 dB, -150.0 dB, 140.0 dBm, 34.0 dB, 2 MHz, 1 MHz),
		CI("CI", 1 nF),
		CQ("CQ", 1 nF),
		DSPsign_I("PRB_I", cfg::wave),
		DSPsign_Q("PRB_Q", cfg::wave)
	{
		sc_core::sc_trace_file *f = create_tab_trace_file("RX-trace", 1e-10);
		splitter.trace(f, "Splitter");
		downconv_I.trace(f, "DownconvertedI");
		filtered_I.trace(f, "FilteredI");
		amplified_I.trace(f, "PGA_outI");
		amplified_Q.trace(f, "PGA_outQ");

		lna(source, splitter);
		vco1(LO1);
		vco2(LO2);
		mixerlpf1(splitter, downconv_I, LO1);
		mixerlpf2(splitter, downconv_Q, LO2);
		lpf1(downconv_I, filtered_I);
		lpf2(downconv_Q, filtered_Q);
		pga_I(filtered_I, amplified_I);
		pga_Q(filtered_Q, amplified_Q);
		CI(amplified_I);
		CQ(amplified_Q);
		DSPsign_I(amplified_I, out_I);
		DSPsign_Q(amplified_Q, out_Q);
	}
};

#endif
