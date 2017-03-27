// cell.cpp:
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

#include "./cell.h"

//  Implementation of class Li_cell with variable parameters
//  shunt diode on R2-C2
//  and methods for interpolation of parameters as functions of internal T and Soc

// Constructor for deterministic parameters

Li_cell::Li_cell (sc_core::sc_module_name name, const std::vector<string> &param_names, double soc_i, double Qn_) : analog_module(3), e_port(base_class::port<electrical>(1)), th_port(base_class::port<thermal>(2)), rv(6, 0.0), soc_i(soc_i), Qn(Qn_)
{
	tie(Ri_in1,  Ri_in2,  Ri_out)  = load2DRecords(param_names[0]);
	tie(R1_in1,  R1_in2,  R1_out)  = load2DRecords(param_names[1]);
	tie(C1_in1,  C1_in2,  C2_out)  = load2DRecords(param_names[2]);
	tie(R2_in1,  R2_in2,  R2_out)  = load2DRecords(param_names[3]);
	tie(C2_in1,  C2_in2,  C2_out)  = load2DRecords(param_names[4]);
	tie(OCV_in1, OCV_in2, OCV_out) = load2DRecords(param_names[5]);
	
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	SC_METHOD(set_params);
	this->sensitive << this->clock_port;
}

// Constructor for parameters with added random values

Li_cell::Li_cell (sc_core::sc_module_name name, const std::vector<string> &param_names, double soc_i, double Qn_, std::vector<double> rv) : analog_module(3), e_port(base_class::port<electrical>(1)), th_port(base_class::port<thermal>(2)), soc_i(soc_i), Qn(Qn_), rv(rv)
{
	tie(Ri_in1,  Ri_in2,  Ri_out)  = load2DRecords(param_names[0]);
	tie(R1_in1,  R1_in2,  R1_out)  = load2DRecords(param_names[1]);
	tie(C1_in1,  C1_in2,  C1_out)  = load2DRecords(param_names[2]);
	tie(R2_in1,  R2_in2,  R2_out)  = load2DRecords(param_names[3]);
	tie(C2_in1,  C2_in2,  C2_out)  = load2DRecords(param_names[4]);
	tie(OCV_in1, OCV_in2, OCV_out) = load2DRecords(param_names[5]);
	
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	SC_METHOD(set_params);
	this->sensitive << this->clock_port;
}

void Li_cell::set_params()
{
	Ri   = interpol2D(T, soc,  Ri_in1, Ri_in2,  Ri_out)  + rv[0];
	R1   = interpol2D(T, soc,  R1_in1, R1_in2,  R1_out)  + rv[1];
	C1   = interpol2D(T, soc,  C1_in1, C1_in2,  C1_out)  + rv[2];
	R2   = interpol2D(T, soc,  R2_in1, R2_in2,  R2_out)  + rv[3];
	C2   = interpol2D(T, soc,  C2_in1, C2_in2,  C2_out)  + rv[4];
	Vocv = interpol2D(T, soc, OCV_in1, OCV_in2, OCV_out) + rv[5];
	Rth  = Ri + R1 + R2;
}

void Li_cell::ics(double VC1, double VC2)
{
	//  the state of the Li_cell are the voltages on the two capacitor
	//  and the derivative of the current
	//   so the Initial Condition will be their voltages
	double ICS[3] = {VC1, VC2, 0};
	this->ic(ICS);
}

void Li_cell::field (double *var) const
{
	const double R0 = this->e_port->get_normalization();
	const double sqrt_R0 = this->e_port->get_normalization_sqrt();
	double P1 =  R1/(Ri+R0);
	double P2 =  R2/(Ri+R0);
	double a = e_port->read();
	var[0] = (-state[0] + 2.0*a*P1*sqrt_R0 - P1*(state[0] + state[1] + Vocv))/R1/C1;
	var[1] = (-state[1] + 2.0*a*P2*sqrt_R0 - P2*(state[0] + state[1] + Vocv))/R2/C2;
	var[2] = iR;
}

void Li_cell::calculus ()
{
	bool not_changed = true;
	const double R0 = this->e_port->get_normalization();
	const double sqrt_R0 = this->e_port->get_normalization_sqrt();
	const double sqrt_R1 = this->th_port->get_normalization_sqrt();
	while (step()) {
		// electrical part
		double a = e_port->read();
		double b = (a*(Ri-R0) + (Vocv+state[0]+state[1])*sqrt_R0)/(Ri+R0);
		iR = (a-b)/sqrt_R0;
		if (state[1]>0) {
			R2 = RON;
			not_changed = false;
		}
		if (state[1]<0 && not_changed) {
			R2   = interpol2D(T, soc,  R2_in1, R2_in2,  R2_out)  + rv[3];
			not_changed = true;
		}
		
		e_port->write(b);
		
		// thermal part
		a = th_port->read();
		b = a + (iR*iR*Rth) * sqrt_R1;
		T = (a+b)*sqrt_R1;
		th_port->write(b);
		// SOC calculation
		soc = soc_i + state[2]/Qn;
	}
}

tuple<vector<double>*,vector<double>*, Mat<double>* > Li_cell::load2DRecords(const std::string &cvs_fileName)
{
	vector<Record2D> records;
	string line;
	string in1, in2;
	string V;
	int i = 0;
	ifstream myfile(cvs_fileName);
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			int pos = line.find_first_of(',');	// ',' position
			V = line.substr(pos + 1);
			in1 = line.substr(0, pos);
			pos = V.find_first_of(',');
			in2 = V.substr(0, pos);
			V = V.substr(pos + 1);
			
			Record2D record;
			record.in1 = ::atof(in1.c_str());
			record.in2 = ::atof(in2.c_str());
			record.out = ::atof(V.c_str());
			records.push_back(record);
			i++;
		}
		myfile.close();
		order2DRecords(&records);
		
		// Make input vector vin1 from records
		vector<double> *vin1, *vin2;
		vin1 = new vector<double>;
		vin2 = new vector<double>;
		for(unsigned i=0, j=0; j<records.size(); i++){
			(*vin1).push_back(records[j].in1);
			for(;(records[j].in1 <= (*vin1)[i])&&(j<records.size()); j++);
		}
		
		// Make input vector vin2 from records
		for(unsigned i=0, j=0; j<records.size(); i++){
			(*vin2).push_back(records[j].in2);
			for(;(records[j].in2 <= (*vin2)[i])&&(j<records.size()); j++);
		}
		
		/*		std::cout << "vin1 contains:";
		 for (std::vector<double>::iterator it = (*vin1).begin() ; it != (*vin1).end(); ++it)
			std::cout << ' ' << *it;
		 std::cout << '\n';
		 
		 std::cout << "vin2 contains:";
		 for (std::vector<double>::iterator it = (*vin2).begin() ; it != (*vin2).end(); ++it)
		 std::cout << ' ' << *it;
		 std::cout << '\n';
		 */
		// Make the output matrix from records
		
		Mat<double>* vout;
		vout = new Mat<double>((*vin1).size(), (*vin2).size());
		for(unsigned k=0; k<records.size(); k++){
			unsigned j = k % (*vin2).size();
			unsigned i = k / (*vin2).size();
			(*vout)(i, j) = records.at(k).out;
		}
		
		return make_tuple(vin1, vin2, vout);
	}
	else
	{	//If error
		Record2D record;
		record.in1 = {};
		record.in2 = {};
		record.out = {};
		records.push_back(record);
		cout << "Error reading file " << cvs_fileName << endl;
	}
}


// Method for 2D interpolation

double Li_cell::interpol2D(double input1, double input2, vector<double>* vin1, vector<double>* vin2, Mat<double>* vout)
{
	double out, Z1, Z2;
	unsigned i = 0;
	for(; (i<(*vin1).size())&&(input1 > (*vin1)[i]); i++);
	unsigned j = 0;
	for(; (j<(*vin2).size())&&(input2 > (*vin2)[j]); ++j);
	if ((i>0)&&(i<(*vin1).size())&&(j>0)&&(j<(*vin2).size()))
	{
		//Interpolation along in2
		Z1 =  ((*vout)(i-1,j) - (*vout)(i-1,j-1))/((*vin2)[j]-(*vin2)[j-1])*(input2-(*vin2)[j-1]) + (*vout)(i-1,j-1);
		Z2 =  ((*vout)(i,j)   - (*vout)(i,j-1))  /((*vin2)[j]-(*vin2)[j-1])*(input2-(*vin2)[j-1]) + (*vout)(i,j-1);
		//Interpolation along in1
		out =   (Z2 - Z1)/((*vin1)[i] - (*vin1)[i-1])*(input1 - (*vin1)[i-1]) + Z1;
	}
	else if( (j==0)&&(i==0) )
	{
		out = (*vout)(i,j);
	}
	else if( (j==(*vin2).size())&&(i==(*vin1).size()) )
	{
		out = (*vout)(i-1,j-1);
	}
	else if( (j==0)&&(i==(*vin1).size()) )
	{
		out = (*vout)(i-1,j);
	}
	else if(j==0)
	{
		Z1 = (*vout)(i-1,j);
		Z2 = (*vout)(i,j);
		//Interpolation along in1
		out =   (Z2 - Z1)/((*vin1)[i] - (*vin1)[i-1])*(input1 - (*vin1)[i-1]) + Z1;
	}
	else if( (i==0)&&(j==(*vin2).size()) )
	{
		out = (*vout)(i,j-1);
	}
	else if(j==(*vin2).size())
	{
		Z1 = (*vout)(i-1,j-1);
		Z2 = (*vout)(i,j-1);
		//Interpolation along in1
		out =   (Z2 - Z1)/((*vin1)[i] - (*vin1)[i-1])*(input1 - (*vin1)[i-1]) + Z1;
	}
	else if (i==0)
	{
		out =  ((*vout)(i,j)   - (*vout)(i,j-1))  /((*vin2)[j]-(*vin2)[j-1])*(input2-(*vin2)[j-1]) + (*vout)(i,j-1);
	}
	else if(i==(*vin1).size())
	{
		out =  ((*vout)(i-1,j)   - (*vout)(i-1,j-1))  /((*vin2)[j]-(*vin2)[j-1])*(input2-(*vin2)[j-1]) + (*vout)(i-1,j-1);
	}
	return out;
}

void Li_cell::order2DRecords(vector<Record2D>* records)
{
	int size = (*records).size();
	
	for (int j = 0; j < size; j++)
	{
		for (int k = 0; k < size - j - 1; k++)
		{
			Record2D current = (*records).at(k);
			Record2D next = (*records).at(k + 1);
			if ( (current.in1 >= next.in1) && (current.in2 >= next.in2) )	//order in ascending order
			{
				(*records).at(k) = next;
				(*records).at(k + 1) = current;
			}
		}
	}
}


//  implementation of class Li_cell
// with fixed parameters

Li_cell_x::Li_cell_x (sc_core::sc_module_name name, double Ri_, double R1_, double C1_, double R2_, double C2_, double Vocv_, double Rth_) : analog_module(2), Ri(Ri_), R1(R1_), C1(C1_), R2(R2_), C2(C2_), Rth(Rth_), Vocv(Vocv_), e_port(port<electrical>(1)), th_port(port<thermal>(2))
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
}

void Li_cell_x::ics(double VC1, double VC2)
{
     //  the state of the Li_cell are the voltages on the two capacitor
	//   so the Initial Condition will be their voltages
	double ICS[2] = {VC1, VC2};
	this->ic(ICS);
}

void Li_cell_x::field (double *var) const
{
	const double R0 = e_port->get_normalization();
	const double sqrt_R0 = e_port->get_normalization_sqrt();
	const double P1 =  R1/(Ri+R0);
	const double P2 =  R2/(Ri+R0);
	double a = e_port->read();
	var[0] = (-state[0] + 2.0*a*P1*sqrt_R0 - P1*(state[0] + state[1] + Vocv))/R1/C1;
	var[1] = (-state[1] + 2.0*a*P2*sqrt_R0 - P2*(state[0] + state[1] + Vocv))/R2/C2;
}

void Li_cell_x::calculus ()
{
	const double R0 = this->e_port->get_normalization();
	const double sqrt_R0 = this->e_port->get_normalization_sqrt();
	const double sqrt_R1 = this->th_port->get_normalization_sqrt();

	while (step()) {
		double a = e_port->read();
		double b = (a*(Ri-R0) + (Vocv+state[0]+state[1])*sqrt_R0)/(Ri+R0);
		double iR = (a-b)/sqrt_R0;
		e_port->write(b);
	    th_port->write(th_port->read() + (iR*iR*Rth) * sqrt_R1);
}
}

//  implementation of class Li_cell with variable parameters

// Constructor for deterministic parameters
Li_cell_var::Li_cell_var (sc_core::sc_module_name name, double soc_i_, double Qn_) : analog_module(3), e_port(base_class::port<electrical>(1)), th_port(base_class::port<thermal>(2)), rv(8, 0.0), Qn(Qn_), soc_i(soc_i_)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	SC_METHOD(set_params);
	this->sensitive << this->C1_port << this->Rth_port << this->Ri_port << this->R1_port << this->R2_port << this->C2_port << this->Vocv_port;
}

// Constructor for parameters with added random values
Li_cell_var::Li_cell_var (sc_core::sc_module_name name, double soc_i_, double Qn_, std::vector<double> rv_) : analog_module(3), e_port(base_class::port<electrical>(1)), th_port(base_class::port<thermal>(2)), rv(rv_), Qn(Qn_),soc_i(soc_i_)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	SC_METHOD(set_params);
	this->sensitive << this->C1_port << this->Rth_port << this->Ri_port << this->R1_port << this->R2_port << this->C2_port << this->Vocv_port;
}


void Li_cell_var::set_params()
{
	Ri  = Ri_port.read()+ rv[0];
	R1  = R1_port.read()+ rv[1];
	C1  = C1_port.read()+ rv[2];
	R2  = R2_port.read()+ rv[3];
	C2  = C2_port.read()+ rv[4];
	Vocv = Vocv_port.read()+ rv[5];
	Rth = Rth_port.read() + rv[6];
	Qn += rv[7];
//	std::cout << "Ri " << rv[0] << "; R1 " << rv[1] << "; C1 "<< rv[2] << "; Vocv" << rv[5] << "; Rth "<< rv[6] << std::endl;
}
void Li_cell_var::ics(double VC1, double VC2)
{
	//  the state of the Li_cell are the voltages on the two capacitor
	//  and the derivative of the current
	//   so the Initial Condition will be their voltages
	double ICS[3] = {VC1, VC2, 0};
	this->ic(ICS);
}

void Li_cell_var::field (double *var) const
{
	const double R0 = this->e_port->get_normalization();
	const double sqrt_R0 = this->e_port->get_normalization_sqrt();
	double P1 =  R1/(Ri+R0);
	double P2 =  R2/(Ri+R0);
	double a = e_port->read();
	var[0] = (-state[0] + 2.0*a*P1*sqrt_R0 - P1*(state[0] + state[1] + Vocv))/R1/C1;
	var[1] = (-state[1] + 2.0*a*P2*sqrt_R0 - P2*(state[0] + state[1] + Vocv))/R2/C2;
	var[2] = iR;
}

void Li_cell_var::calculus ()
{
	const double R0 = this->e_port->get_normalization();
	const double sqrt_R0 = this->e_port->get_normalization_sqrt();
	const double sqrt_R1 = this->th_port->get_normalization_sqrt();
	
	while (step()) {
		double a = e_port->read();
		double b = (a*(Ri-R0) + (Vocv+state[0]+state[1])*sqrt_R0)/(Ri+R0);
		iR = (a-b)/sqrt_R0;
		e_port->write(b);
		th_port->write(th_port->read() + (iR*iR*Rth) * sqrt_R1);
		soc.write(soc_i + state[2]/Qn);
	}
}

//  implementation of class Li_cell with variable parameters
//  and shunt diode on R2-C2

// Constructor for deterministic parameters
Li_cell_d::Li_cell_d (sc_core::sc_module_name name, double soc_i_, double Qn_) : analog_module(3), e_port(base_class::port<electrical>(1)), th_port(base_class::port<thermal>(2)), rv(8, 0.0), Qn(Qn_), soc_i(soc_i_)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	SC_METHOD(set_params);
	this->sensitive << this->C1_port << this->Rth_port << this->Ri_port << this->R1_port << this->R2_port << this->C2_port << this->Vocv_port;
}

// Constructor for parameters with added random values
Li_cell_d::Li_cell_d (sc_core::sc_module_name name, double soc_i_, double Qn_, std::vector<double> rv_) : analog_module(3), e_port(base_class::port<electrical>(1)), th_port(base_class::port<thermal>(2)), rv(rv_), Qn(Qn_),soc_i(soc_i_)
{
	SC_THREAD(calculus);
	this->sensitive << this->activation;
	SC_METHOD(set_params);
	this->sensitive << this->C1_port << this->Rth_port << this->Ri_port << this->R1_port << this->R2_port << this->C2_port << this->Vocv_port;
}


void Li_cell_d::set_params()
{
	Ri  = Ri_port.read()+ rv[0];
	R1  = R1_port.read()+ rv[1];
	C1  = C1_port.read()+ rv[2];
	R2  = R2_port.read()+ rv[3];
	C2  = C2_port.read()+ rv[4];
	Vocv = Vocv_port.read()+ rv[5];
	Rth = Rth_port.read() + rv[6];
	Qn += rv[7];
	//	std::cout << "Ri " << rv[0] << "; R1 " << rv[1] << "; C1 "<< rv[2] << "; Vocv" << rv[5] << "; Rth "<< rv[6] << std::endl;
}
void Li_cell_d::ics(double VC1, double VC2)
{
	//  the state of the Li_cell are the voltages on the two capacitor
	//  and the derivative of the current
	//   so the Initial Condition will be their voltages
	double ICS[3] = {VC1, VC2, 0};
	this->ic(ICS);
}

void Li_cell_d::field (double *var) const
{
	const double R0 = this->e_port->get_normalization();
	const double sqrt_R0 = this->e_port->get_normalization_sqrt();
	double P1 =  R1/(Ri+R0);
	double P2 =  R2/(Ri+R0);
	double a = e_port->read();
	var[0] = (-state[0] + 2.0*a*P1*sqrt_R0 - P1*(state[0] + state[1] + Vocv))/R1/C1;
	var[1] = (-state[1] + 2.0*a*P2*sqrt_R0 - P2*(state[0] + state[1] + Vocv))/R2/C2;
	var[2] = iR;
}

void Li_cell_d::calculus ()
{
	const double R0 = this->e_port->get_normalization();
	const double sqrt_R0 = this->e_port->get_normalization_sqrt();
	const double sqrt_R1 = this->th_port->get_normalization_sqrt();
	
	while (step()) {
		double a = e_port->read();
		double b = (a*(Ri-R0) + (Vocv+state[0]+state[1])*sqrt_R0)/(Ri+R0);
		iR = (a-b)/sqrt_R0;
		if (state[1]>0)
			R2 = RON;
		else
			R2  = R2_port.read()+ rv[3];
		e_port->write(b);
		th_port->write(th_port->read() + (iR*iR*Rth) * sqrt_R1);
		soc.write(soc_i + state[2]/Qn);
	}
}
