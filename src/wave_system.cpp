// wave_system.cpp:
// Copyright (C) 2006 Giorgio Biagetti
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

#include "wave_system"

#ifdef HAVE_LAPACK
// This file depends on LAPACK solvers
extern "C" void dgesv_ ( // general solver of linear equations
	const int &n, const int &nrhs,
	double a[], const int &lda,
	int ipiv[],
	double b[], const int &ldb,
	int &info
);
#endif

// Implementation of other classes:

ab_signal_memory ab_signal_memory::state = 0;

// Implementation of class scatter_junction:

scatter_junction::scatter_junction ()
{
	for (int i = 0; i < max_dim; ++i)
		for (int j = 0; j < max_dim; ++j)
			scatter[i][j] = 0;
}

int scatter_junction::compute_scattering (double const *norms)
{
	int na = 0, nt = 0;
	const short *matrix = incidence_matrix(na, nt);
	double system[max_dim][max_dim];
	int n = na + nt;
	for (unsigned i = 0; i < n; ++i) {
		for (unsigned j = 0; j < na; ++j)
			scatter[i][j] = -(system[i][j] = matrix[j * n + i] * norms[i]);
		for (unsigned j = na; j < n; ++j)
			scatter[i][j] = +(system[i][j] = matrix[j * n + i] / norms[i]);
	}
	int info = 0;
#ifdef HAVE_LAPACK
	int pivot[max_dim];
	dgesv_(n, n, &system[0][0], max_dim, &pivot[0], &scatter[0][0], max_dim, info);
#else
	for (unsigned k = 0; k < n - 1; ++k) {
		double max = 0;
		int p = k;
		for (unsigned i = k; i < n; ++i)
			if (std::abs(system[k][i]) > max) {
				max = std::abs(system[k][i]);
				p = i;
			}
		if (max == 0) { // singular matrix
			info = k + 1;
			break;
		}
		for (unsigned i = 0; i < n; ++i) {
			std::swap(system[i][k], system[i][p]);
			std::swap(scatter[i][k], scatter[i][p]);
		}
		for (unsigned i = k + 1; i < n; ++i) {
			system[k][i] /= system[k][k];
			for (unsigned j = k + 1; j < n; ++j)
				system[j][i] -= system[k][i] * system[j][k];
		}
	}
	if (!info) {
		for (unsigned k = 0; k < n; ++k) {
			for (unsigned i = 0; i < n; ++i) {
				for (unsigned j = 0; j < i; ++j)
					scatter[k][i] -= system[j][i] * scatter[k][j];
			}
			for (unsigned i = n - 1; i < n; --i) {
				for (unsigned j = i + 1; j < n; ++j)
					scatter[k][i] -= system[j][i] * scatter[k][j];
				scatter[k][i] /= system[i][i];
			}
		}
	}
#endif
	if (info < 0) {
		std::cerr << "Error in scatter junction: invalid argument " << -info << std::endl;
	} else if (info > 0) {
		std::cerr << "Error in scatter junction: singularity detected!" << std::endl;
	} else return n;
	return 0;
}

