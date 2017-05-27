///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>

#include <parser.h>

int main(int argc, char *argv[]) {
    parser_data data;
    sim_data sim;
    load(&data, "test.csv");
    load_simulation_data(&data, &sim);

    printf("\n-------- Parsed data --------\n\n");
    print_parsed_data(&data);
    printf("\n\n----- Simulation data -----\n\n");
    print_simulation_data(&sim);

    unload_simulation_data(&sim);
    unload(&data);
    return EXIT_SUCCESS;
}

