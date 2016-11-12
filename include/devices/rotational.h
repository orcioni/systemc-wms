// rotational devices:
// Copyright (C) 2014 Giorgio Biagetti and Simone Orcioni
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

#ifndef ROTATIONAL_H
#define ROTATIONAL_H

#include "../analog_system"
#include "../nature/mechnical"

//
// Library of mechanical devices
//
//
// typedef I_load<rotational>  inertia_moment
// moment of inertia, along a principal axis of the object or angular mass

// typedef D_load<rotational>  torsion_spring
// the load applied to a torsion spring is a torque or twisting force, and the end of the spring rotates through an angle as the load is applied


typedef P_load<rotational>  viscous_friction;
typedef D_load<rotational>  torsion_spring;
typedef I_load<rotational>  inertia_moment;
//typedef PDp_load<mechanical>  dumped_spring;
//typedef transformer<mechanical> lever;
//typedef TSTC<mechanical>  friction;


#endif //ROTATIONAL_H
