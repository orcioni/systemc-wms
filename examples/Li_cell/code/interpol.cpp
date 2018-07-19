// interpol.cpp:
// Copyright (C) 2018 Simone Orcioni
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

#include "./interpol.h"

//  Implementation of class interpol for 2D interpolation of data coming from file.

tuple<vector<double>*,vector<double>*, Mat<double>* > interpol::load2DRecords(const std::string &cvs_fileName)
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

double interpol::interpol2D(double input1, double input2, vector<double>* vin1, vector<double>* vin2, Mat<double>* vout)
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

void interpol::order2DRecords(vector<Record2D>* records)
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
