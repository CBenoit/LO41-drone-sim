///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_DRONE
#define DRONE_SIM_DRONE

#include "structs.h"
#include "parser_structs.h"

/**
 * @brief Launches a drone.
 * 
 * @param drone             Informations about the launched drone.
 * @param client_pipes      File descriptor's id linked to the clients.
 * @param number_of_clients Total number of clients.
 * @param msqid             ID of the mothership's message queue
 * @param data              Data of the simulation.
 */
void drone_main(drone_t drone, int* client_pipes, unsigned int number_of_clients, int msqid, sim_data* data);

#endif /* ifndef DRONE_SIM_DRONE */

