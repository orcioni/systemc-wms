// battery.cpp:
// Copyright (C) 2016 Simone Orcioni
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
#include <iostream>
#include <string>
#include <random>
#include <complex>

#include "sys/probes"
#include "units/constants"
#include "sys/sources"
#include "devices/electrical_oneport.h"
#include "devices/electrical_twoport.h"
#include "devices/electrothermal.h"
#include "tab_trace"

#include "cell.h"


// main program:
int sc_main (int argc, char *argv[])
{
	// Command-line parameters:
	double sim_time;
	double sim_current, soc_i, TEMP, seed, perc1;
	char fileName[25], current_file[25];
	
	if (argc == 8) {
		sscanf(argv[1], "%lf", &sim_time);
		sscanf(argv[2], "%lf", &soc_i);
		sscanf(argv[3], "%lf", &TEMP);
		sscanf(argv[4], "%lf", &seed);
		sscanf(argv[5], "%lf", &perc1);
		sscanf(argv[6], "%s", current_file);
		sscanf(argv[7], "%s",  fileName);
	}  else {
		std::cout << "Usage: " << argv[0] << " <sim_time> <initial soc> <current_file> <out_file>\n\n";
		std::cout << "sim_time       -> Duration of simulation [s];\n";
		std::cout << "soc_i          -> Initial SOC [0-1]\n";
		std::cout << "TEMP           -> Ambiente temperature [°C]\n";
		std::cout << "seed           -> Seed of random generator\n";
		std::cout << "perc           -> StdDev = Percentage of nominal value\n";
		std::cout << "current_file   -> Name of input current file;\n";
		std::cout << "fileName       -> Name of trace file\n";
		std::cout << std::endl;
		return 1;
	}

	sc_core::sc_set_time_resolution(1.0, sc_core::SC_NS);
	const sc_time t_PERIOD (1.0e-1, SC_SEC);
	sc_core::sc_clock clk("clk", t_PERIOD, 0.5);
	
	// Random generators
	
	// Mersenne-twister random number generator:
	std::mt19937_64 engine;
	//	engine.seed(std::mt19937_64::default_seed);
	engine.seed(seed);
	
	
	// names of parameters files
	const std::vector<string> param_names =	{"../data/Ri.csv", "../data/R1.csv", "../data/C1.csv", "../data/R2.csv", "../data/C2.csv", "../data/OCV.csv"};
	
	// cell  parameters
	const double Qnom = 180000; // 50Ah = 180000 Coulomb
	const double mass = 1.4;
	const double L = 37e-3;
	const double W = 0.101;
	const double H = 0.192;
	const double d = 2e-3;
	const double A = W*H;
	const double heat_sink_A1 = 2*L*(W + H);
	const double heat_sink_A2 = heat_sink_A1 + A;
	const double ki = 1.1;
	const double ks = 58;
	const double cp = 830.0;
	const double heat_sink_h = 38.953;
	const double Rin_val = L/(ki*A);
	const double Cap_val   = cp*mass;
	const double Rcnt_val = 2*d/(ks*A);
	const double Rcnt1_num = 2*d/(ks*heat_sink_A1);
	const double Rcnt2_num = 2*d/(ks*heat_sink_A2);

	
	double perc = 10.0/3.0;
	// Ri statistical distribution:
	const double Ri_stddev =  perc1*perc*0.0024;
	std::normal_distribution<double> Ri_n(0.0, Ri_stddev);

	// R1 statistical distribution:
	const double R1_stddev =  perc1*perc*0.0012;
	std::normal_distribution<double> R1_n(0.0, R1_stddev);
	
	// C1 statistical distribution:
	const double C1_stddev =  perc1*perc*41667;
	std::normal_distribution<double> C1_n(0.0, C1_stddev);

	// Qnom statistical distribution:
	const double Qnom_stddev =  perc1*perc*Qnom;
	std::normal_distribution<double> Qnom_n(0.0, Qnom_stddev);

	// R2 statistical distribution:
	const double R2_stddev =  perc1*perc*0.0002;
	std::normal_distribution<double> R2_n(0.0, R2_stddev);
	
	// C2 statistical distribution:
	const double C2_stddev =  perc1*perc*15000;
	std::normal_distribution<double> C2_n(0.0, C2_stddev);

	// Vocv statistical distribution:
	const double Vocv_stddev =  perc1*3.28;
	std::normal_distribution<double> Vocv_n(0.0, Vocv_stddev);

	// Vocv statistical distribution:
	const double soc_stddev =  perc1*perc*soc_i;
	std::normal_distribution<double> soc_n(0.0, soc_stddev);



	// The battery architecture is limited to 1 modules
	// consisitng in the series of sub-modules of parallel cells.
	
	// Number of sub-modules
	const unsigned M = 1;
	// Number of cells per module
	const unsigned N = 6;

// assign statistical parameter variations
	std::vector<double> *params[M][N];
	
	for(unsigned i = 0; i < M; i++)
		for(unsigned j=0; j < N; j++)
			{
			params[i][j] = new std::vector<double>(6);
			(*params[i][j])[0]=Ri_n(engine);
			(*params[i][j])[1]=R1_n(engine);
			(*params[i][j])[2]=C1_n(engine);
			(*params[i][j])[3]=R2_n(engine);
			(*params[i][j])[4]=C2_n(engine);
			(*params[i][j])[5]=Vocv_n(engine);
			}
	
	// Modules instantiation and connection:
	
	ab_signal <electrical, series>     B_ch_se;
	ab_signal <electrical, parallel>   Cell_ch_sh[M];
	
	ab_signal <thermal, parallel>   Cell_th_ch_int[M][N], Cell_th_ch_ext[M][N];
	

	Thermal_capacitance *Cap[M][N];
	Thermal_resistance *Rint[M][N], *Rcnt[M-1][N];
	Thermal_convector *heat_sink[M][N];
	Li_cell_itrp *B_cell[M][N];
	ab_connector <electrical> *parallel2series[M];
	
	// The ambient, numeric value of ambient temperature
	ab_signal <thermal, parallel> cov_amb;
	sc_core::sc_signal <double> amb_ch;
	generator <double > amb_num("amb_num", dc(TEMP));
	amb_num(amb_ch);
	// The ambient, modelled as an ideal "Temperature generator"
	source <thermal> amb("amb", cfg::across);
	amb.input(amb_ch);
	amb.port(cov_amb);

	
	for(unsigned i = 0; i < M; i++){      //series
		for(unsigned j = 0; j < N; j++){  // parallel
			
			// Cells
			std::string name = "B_cell_" + std::to_string(i)+ std::to_string(j);
			B_cell[i][j] = new Li_cell(name.c_str(), param_names, soc_i+soc_n(engine), Qnom+Qnom_n(engine), *params[i][j]);
			B_cell[i][j]->e_port(Cell_ch_sh[i]);
			B_cell[i][j]->th_port(Cell_th_ch_int[i][j]);
			B_cell[i][j]->clock_port(clk);
			B_cell[i][j]->set_steplimits(1.0e-2, 1.0);
			B_cell[i][j]->set_tolerances (1e-9, 1e-3, 1e-1);

			// Thermal resistance from inner of the cell to outside
			name = "Rint_" + std::to_string(i)+ std::to_string(j);
			Rint[i][j]   = new Thermal_resistance(name.c_str(), Rin_val);
			(*Rint[i][j])(Cell_th_ch_int[i][j],Cell_th_ch_ext[i][j]);
			
			// Thermal resistence between a couple of cell
			if ((i*N+j != M*N/2-1)&&(i*N+j != M*N-1))
				if (j<N-1) {
					name = "Rcnt_" + std::to_string(i)+ std::to_string(j);
					Rcnt[i][j]   = new Thermal_resistance(name.c_str(), Rcnt_val);
					(*Rcnt[i][j])(Cell_th_ch_ext[i][j],Cell_th_ch_ext[i][j+1]);
				} else {
					name = "Rcnt_" + std::to_string(i)+ std::to_string(j);
					Rcnt[i][j]   = new Thermal_resistance(name.c_str(), Rcnt_val);
					(*Rcnt[i][j])(Cell_th_ch_ext[i+1][0],Cell_th_ch_ext[i][j]);
				};

			// Thermal capacitance of the cell
			name = "C_" + std::to_string(i)+ std::to_string(j);
			Cap[i][j]   = new Thermal_capacitance(name.c_str(), Cap_val);
			(*Cap[i][j])(Cell_th_ch_int[i][j]);
			Cap[i][j]->ics(TEMP);
			Cap[i][j]->set_steplimits (1.0e-2, 1);

			// Thermal exchange with ambient
			name = "heat_sink_" + std::to_string(i)+ std::to_string(j);
			switch (i*N+j) {
				case 0:
				case M*N/2-1:
				case M*N/2:
				case M*N-1:
					heat_sink[i][j] = new Thermal_convector(name.c_str(), heat_sink_A2, heat_sink_h);
					break;
				default:
					heat_sink[i][j] = new Thermal_convector(name.c_str(), heat_sink_A1, heat_sink_h);
					break;
			}
			(*heat_sink[i][j])(Cell_th_ch_ext[i][j],cov_amb);
		}
		std::string name = "sh2seCONN" + std::to_string(i);
		parallel2series[i] = new ab_connector<electrical> (name.c_str());
		(*parallel2series[i])(Cell_ch_sh[i], B_ch_se);
	}


// Load current
	sc_core::sc_signal <double> I_ch;
	generator <double > I_num("I_num", file(current_file));
	I_num(I_ch);
	source <electrical> I0("I0", cfg::through);
	I0.input(I_ch);
	I0.port(-B_ch_se);

	
	

	// Output files:
	std::string ele_name     = string(fileName) + "_ele";
	std::string thermal_name = string(fileName) + "_therm";

	sc_core::sc_trace_file *ef = create_tab_trace_file(ele_name.c_str(), 1.0);
//	sc_core::sc_trace_file *tf = create_tab_trace_file(thermal_name.c_str(), 1.0);

	//B_ch_se.trace(ef, "MODULE");
	Cell_ch_sh[0].trace(ef, "M0");
	
//	Cell_th_ch_int[0][0].trace(tf, "IC00");
//	Cell_th_ch_int[0][1].trace(tf, "IC01");
//	Cell_th_ch_int[0][2].trace(tf, "IC02");
//	Cell_th_ch_int[0][3].trace(tf, "IC03");
//	Cell_th_ch_int[0][4].trace(tf, "IC04");
//	Cell_th_ch_int[0][5].trace(tf, "IC05");


//	Cell_th_ch_ext[0][0].trace(tf, "EC00");
//	Cell_th_ch_ext[0][1].trace(tf, "EC01");
//	Cell_th_ch_ext[0][2].trace(tf, "EC02");
//	Cell_th_ch_ext[0][3].trace(tf, "EC03");
//	Cell_th_ch_ext[0][4].trace(tf, "EC04");
//	Cell_th_ch_ext[0][5].trace(tf, "EC05");
	
	sc_core::sc_trace(ef, B_cell[0][5]->soc, "soc_05");
	sc_core::sc_trace(ef, B_cell[0][4]->soc, "soc_04");
	sc_core::sc_trace(ef, B_cell[0][3]->soc, "soc_03");
	sc_core::sc_trace(ef, B_cell[0][2]->soc, "soc_02");
	sc_core::sc_trace(ef, B_cell[0][1]->soc, "soc_01");
	sc_core::sc_trace(ef, B_cell[0][0]->soc, "soc_00");

	sc_core::sc_trace(ef, B_cell[0][5]->Vocv, "OCV_05");
	sc_core::sc_trace(ef, B_cell[0][4]->Vocv, "OCV_04");
	sc_core::sc_trace(ef, B_cell[0][3]->Vocv, "OCV_03");
	sc_core::sc_trace(ef, B_cell[0][2]->Vocv, "OCV_02");
	sc_core::sc_trace(ef, B_cell[0][1]->Vocv, "OCV_01");
	sc_core::sc_trace(ef, B_cell[0][0]->Vocv, "OCV_00");

	sc_core::sc_start(sc_core::sc_time(sim_time, sc_core::SC_SEC));
	
	close_tab_trace_file(ef);
//	close_tab_trace_file(tf);
	return 0;
}
