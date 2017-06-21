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

#define AIRWAY_SIZE ((double) 0.2) // in radians

/**
 * @brief  Loads a file.
 * @detail Parsed datas are writen to 'data'. Allocation may occur.
 *
 * @param[out] data      After a call to this method, contains data parsed in a 'to-be-processed' state.
 * @param[in]  file_path Path to the data to parse.
 */
void load(parser_data* data, const char* file_path);

/**
 * @brief Cleans previoulsy allocated data.
 *
 * @param[in] data Data to unload.
 */
void unload(parser_data* data);

/**
 * @brief Loads the simulation's data from pre-processed data.
 * @detail Parsed data are writen to 'data'. Allocation may occur.
 *
 * @param[in] input   Data previoulsy allocated via a call to 'load'.
 * @param[out] output After the call, contains processed data, ready for the simulation.
 */
void load_simulation_data(parser_data* input, sim_data* output);

/**
 * @brief Cleans previoulsy loaded simulation.
 *
 * @param[in] data Data to unload.
 */
void unload_simulation_data(sim_data* data);

/**
 * @brief Loads the simulation's data from a file. Allocation may occur.
 *
 * @param[out] data      After a call to this method, contains the simulation's datax
 * @param[in]  file_path Path to the data to parse.
 */
void load_simulation(sim_data* data, const char* file_path);

/**
 * @brief Cleans previoulsy allocated data
 *
 * @param[in] data Data to unload.
 */
void unload_simulation(sim_data* data);

/**
 * @brief prints info about pre-processed data
 */
void print_parsed_data(parser_data* data);

/**
 * @brief prints info about a simulation data.
 */
void print_simulation_data(sim_data* data);

#endif // DRONE_SIM_PARSER
