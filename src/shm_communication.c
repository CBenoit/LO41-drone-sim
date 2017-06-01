///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <sys/mman.h>
#include <fcntl.h> // For O_* constants
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <stdint.h>

#include "shm_communication.h"
#include "utility.h"

#define BAD_SIZE ((size_t) 9999)
#define BAD_FILDES -1

// to be mapped to the shared memory
size_t* flying_drone_nbr;
pid_t* flying_drones;
void* data;

// just static fields
static size_t drones_nb = BAD_SIZE;
static int fildes = BAD_FILDES;

// === helpers ===

static void check_initialized(void);
static void check_fildes_opened(void);
static void open_fildes(void);
static void close_fildes(void);

void check_initialized() {
    if (drones_nb == BAD_SIZE) {
        printf("error: shared memory hasn't been initialized.\n");
        exit(EXIT_FAILURE);
    }
}

void check_fildes_opened() {
    if (fildes == BAD_FILDES) {
        printf("error: shared memory hasn't been mapped.\n");
        exit(EXIT_FAILURE);
    }
}

void open_fildes() {
    fildes = shm_open(SHARED_MEM_NAME, O_RDWR, 0644);
    if (fildes == -1) {
        perror("open_fildes shm_open");
        exit(EXIT_FAILURE);
    }
}

void close_fildes() {
    if (close(fildes) == -1) {
        perror("close");
        exit(EXIT_FAILURE);
    }
    fildes = BAD_FILDES;
}

// === end helpers ===

void initialize_shared_memory(size_t number_of_drones) {
    drones_nb = number_of_drones;

    // ==== create shared memory ====
    fildes = shm_open(SHARED_MEM_NAME, O_CREAT | O_TRUNC | O_RDWR, 0644);
    if (fildes == -1) {
        perror("initialize_shared_memory shm_open");
        exit(EXIT_FAILURE);
    }

    if (ftruncate(fildes, (off_t) (sizeof(size_t) + sizeof(pid_t) * drones_nb)) == -1) {
        perror("initialize_shared_memory ftruncate");
        exit(EXIT_FAILURE);
    }

    close_fildes();
    // ==============================

    // map local variables to the shared memory
    map_shared_memory();

    // initialize data (initially, zero flying drone)
    *flying_drone_nbr = 0;

    // unmap local variables
    unmap_shared_memory();
}

void clean_shared_memory() {
    //check_initialized();
    shm_unlink(SHARED_MEM_NAME);
}

void map_shared_memory() {
    check_initialized();
    open_fildes();

    data = mmap(NULL, sizeof(size_t) + sizeof(pid_t) * drones_nb, PROT_READ | PROT_WRITE, MAP_SHARED, fildes, 0);
    if (flying_drone_nbr == MAP_FAILED) {
        perror("map_shared_memory mmap");
        exit(EXIT_FAILURE);
    }

    flying_drone_nbr = data;
    flying_drones = (pid_t*) ((size_t*) data + 1);
}

void unmap_shared_memory() {
    check_fildes_opened();

    if (munmap(data, sizeof(size_t) + sizeof(pid_t) * drones_nb) == -1) {
        perror("unmap_shared_memory munmap");
        exit(EXIT_FAILURE);
    }

    close_fildes();
}

// everything is thread safe thanks to a semaphore
void remove_flying_drone(pid_t drone_pid) {
    check_fildes_opened();

    for (size_t i = *flying_drone_nbr; i--;) {
        if (flying_drones[i] == drone_pid) {
            --*flying_drone_nbr;
            flying_drones[i] = flying_drones[*flying_drone_nbr];
            return;
        }
    }
}

void add_flying_drone(pid_t drone_pid) {
    check_fildes_opened();

    flying_drones[*flying_drone_nbr] = drone_pid;
    ++*flying_drone_nbr;
}

bool drone_is_flying(pid_t drone_pid) {
    check_fildes_opened();

    for (size_t i = *flying_drone_nbr; i--;) {
        if (flying_drones[i] == drone_pid) {
            return true;
        }
    }
    return false;
}

pid_t* get_flying_drones() {
    check_fildes_opened();
    return flying_drones;
}

size_t get_number_of_flying_drones() {
    check_fildes_opened();
    return *flying_drone_nbr;
}

