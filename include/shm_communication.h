///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_SHM_COMMUNICATION
#define DRONE_SIM_SHM_COMMUNICATION

#include <unistd.h>
#include <stdbool.h>

void initialize_shared_memory(size_t number_of_drones);

void clean_shared_memory(void);

void map_shared_memory(void);

void unmap_shared_memory(void);

// everything is thread safe thanks to a semaphore
void remove_flying_drone(pid_t drone_pid);

void add_flying_drone(pid_t drone_pid);

bool drone_is_flying(pid_t drone_pid);

pid_t* get_flying_drones(void);

size_t get_number_of_flying_drones(void);

#endif

