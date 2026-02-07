/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */


    //This is a configuration for upgraded GTS controllers, with upgraded current measurement and/or voltage measurement depending on what you comment. 

    #ifndef HW_GTS_R_H_
    #define HW_GTS_R_H_
    
    #define DISABLE_HW_LIMITS //not bothering to add ifdefs for 150v/150a limits. if you modified a controller, you dont need guardrails. 

    
    //GTS150V for a divider with (x)ohm r2. 
    //GTS150A for 150a current sensors, ie CC6922SG-5FB150. 

    #define GTS150A
    //#define GTS150V
    
    
    #define GTS
    #define HW_NAME					"GTS-R"

    #include "hw_gt_core.h"
    
    #endif /* HW_GTS_R_H_ */
    