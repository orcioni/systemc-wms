// tab_trace.cpp:
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


#include "tab_trace"
#include <vector>
#include <string>

class trace_devirtualizer : public sc_core::sc_trace_file
{
	void trace(const bool& object, const std::string& name) {}
	void trace(const sc_dt::sc_bit& object, const std::string& name) {}
	void trace(const sc_dt::sc_logic& object, const std::string& name) {}
	void trace(const unsigned char& object, const std::string& name, int width) {}
	void trace(const unsigned short& object, const std::string& name, int width) {}
	void trace(const unsigned int& object, const std::string& name, int width) {}
	void trace(const unsigned long& object, const std::string& name, int width) {}
	void trace(const char& object, const std::string& name, int width) {}
	void trace(const short& object, const std::string& name, int width) {}
	void trace(const int& object, const std::string& name, int width) {}
	void trace(const long& object, const std::string& name, int width) {}
	void trace(const sc_dt::int64& object, const std::string& name, int width) {}
	void trace(const sc_dt::uint64& object, const std::string& name, int width) {}
	void trace(const float& object, const std::string& name) {}
	void trace(const double& object, const std::string& name) {}
	void trace(const sc_dt::sc_uint_base& object, const std::string& name) {}
	void trace(const sc_dt::sc_int_base& object, const std::string& name) {}
	void trace(const sc_dt::sc_unsigned& object, const std::string& name) {}
	void trace(const sc_dt::sc_signed& object, const std::string& name) {}
	void trace(const sc_dt::sc_fxval& object, const std::string& name ) {}
	void trace(const sc_dt::sc_fxval_fast& object, const std::string& name ) {}
	void trace(const sc_dt::sc_fxnum& object, const std::string& name ) {}
	void trace(const sc_dt::sc_fxnum_fast& object, const std::string& name ) {}
	void trace(const sc_dt::sc_bv_base& object, const std::string& name) {}
	void trace(const sc_dt::sc_lv_base& object, const std::string& name) {}
	void trace(const unsigned& object, const std::string& name, const char** enum_literals) {}
};

class tab_trace_file : public trace_devirtualizer
{
public:
	tab_trace_file (const char *name, double minstep);
	void set_time_unit (double, sc_core::sc_time_unit) {}
	~tab_trace_file ();
protected:
	void trace (double const &object, std::string const &name);
	void write_comment (std::string const &comment);
	void delta_cycles (bool flag) {trace_deltas = flag;}
	void cycle (bool delta_cycle);
private:
	FILE *f;
	bool trace_deltas, initialized;
	std::vector <const double *> traces;
	std::vector <std::string> names;
	double next_t, output_step;
	void initialize ();
};

tab_trace_file::tab_trace_file (const char *name, double minstep) : trace_deltas(false), initialized(false)
{
	std::string filename = name;
	filename += ".txt";
	if (!(f = fopen(filename.c_str(), "w"))) {
		std::cerr << "ERROR: Cannot open trace file \"" << filename << "\".\n";
		throw;
	}
	next_t = -1;
	output_step = minstep;
}

tab_trace_file::~tab_trace_file ()
{
	if (f) fclose(f);
}

void tab_trace_file::initialize ()
{
	for (unsigned i = 0; i < names.size(); ++i)
		fprintf(f, "# % 3d: %s\n", i + 2, names[i].c_str());
	initialized = true;
}

void tab_trace_file::write_comment (std::string const &comment)
{
	fprintf(f, "# %s\n", comment.c_str());
}

void tab_trace_file::trace (double const &object, std::string const &name)
{
	traces.push_back(&object);
	names.push_back(name.c_str());
}

void tab_trace_file::cycle (bool this_is_a_delta_cycle)
{
	if (this_is_a_delta_cycle && !trace_deltas) return;
	if (!initialized) initialize();
	double t = sc_core::sc_time_stamp().to_seconds();
	if (output_step > 0) {
		if (t < next_t) return;
		next_t = t + output_step;
	}
	fprintf(f, "%e", sc_core::sc_time_stamp().to_seconds());
	for (unsigned i = 0, j = traces.size(); i < j; ++i)
		fprintf(f, "\t%e", *traces[i]);
	fprintf(f, "\n");
}


// auxiliary functions:

sc_core::sc_trace_file *create_tab_trace_file (const char *name, double minstep)
{
	sc_core::sc_trace_file *t = new tab_trace_file(name, minstep);
	// simcontext is deprecated in version 2.1, is there another way?
	sc_core::sc_get_curr_simcontext()->add_trace_file(t);
	return t;
}

void close_tab_trace_file (sc_core::sc_trace_file *f)
{
	tab_trace_file *t = (tab_trace_file *) f;
	delete t;
}
