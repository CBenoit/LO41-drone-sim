///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef DRONE_SIM_PARSER
#define DRONE_SIM_PARSER

#include "parser_structs.h"

void load(parser_data* data, const char* file_path);

void unload(parser_data* data);

void load_simulation_data(parser_data* input, sim_data* output);

void unload_simulation_data(sim_data* data);

void load_simulation(sim_data* data, const char* file_path);

void unload_simulation(sim_data* data);

void print_parsed_data(parser_data* data);

void print_simulation_data(sim_data* data);

#endif // DRONE_SIM_PARSER
