///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/msg.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <semaphore.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <math.h>
#include <signal.h>
#include <string.h>

#include "colors.h"
#include "mothership.h"
#include "utility.h"
#include "mq_communication.h"
#include "shm_communication.h"
#include "parser.h"
#include "typedefs.h"
#include "stats.h"

#define TICK_MIN_TIME_MSEC 500
#define BAD_ID ((identity_t) -1)

// === private functions ===

static void init_timer(void);
static void wait_timer(void);
static void send_sig_to_list(int sig, pid_t* list, size_t len);
static void send_sig_to_all(int sig);
static void tick(void);
static void fail_fast(const char* message);
static void wait_for(unsigned long);
static void clean(void);
static bool find_drone(identity_t id, drone_t* drone_found);
static bool find_appropriate_package_for_drone(identity_t drone_id, identity_t* package_id_found);
static identity_t find_drone_id_by_pid(pid_t drone_pid);

static void interruption_handler(int);
static void sigchild_handler(int);

// === private variables

static mothership_t* this;

static sim_data* m_sdata  = NULL;
static pid_t* m_drones_p  = NULL;
static pid_t* m_clients_p = NULL;
static pid_t* m_hunters_p = NULL;
static int m_msqid;

static size_t m_remaining_power_loading_slots;

struct timeval m_beg_time;

static bool m_is_drone_turn = false;
static bool m_is_hunter_turn = false;
static bool m_is_client_turn = false;
static size_t m_remaining_drone_nbr;
static size_t m_remaining_client_nbr;
static size_t m_remaining_hunter_nbr;

static identity_t* m_package_id_by_drone_id;
static bool* m_drones_going_to_client;
static bool* m_remaining_packages;
static bool* m_busy_clients;

static sim_stats m_stats;

// === implementations

void mothership_main(sim_data* sdata, pid_t* drones_p, pid_t* clients_p, pid_t* hunters_p, int msqid) {
    // initializing
    signal(SIGINT, &interruption_handler);
    signal(SIGCHLD, &sigchild_handler);
    map_shared_memory();

    this = &sdata->mothership;

    m_sdata = sdata;
    m_drones_p = drones_p;
    m_clients_p = clients_p;
    m_hunters_p = hunters_p;
    m_msqid = msqid;

    m_remaining_power_loading_slots = m_sdata->mothership.power_loading_slots;
    m_remaining_drone_nbr = m_sdata->drone_nbr;
    m_remaining_client_nbr = this->client_nbr;
    m_remaining_hunter_nbr = m_sdata->hunter_nbr;

    m_package_id_by_drone_id = (identity_t*) malloc(sizeof(identity_t) * sdata->drone_nbr);
    for (size_t i = sdata->drone_nbr; i--;) {
        m_package_id_by_drone_id[i] = BAD_ID;
    }

    m_drones_going_to_client = (bool*) malloc(sizeof(bool) * sdata->drone_nbr);
    for (size_t i = sdata->drone_nbr; i--;) {
        m_drones_going_to_client[i] = false;
    }

    m_remaining_packages = (bool*) malloc(sizeof(bool) * this->package_nbr);
    for (size_t i = this->package_nbr; i--;) {
        m_remaining_packages[i] = true;
    }

    m_busy_clients = (bool*) malloc(sizeof(bool) * this->client_nbr);
    for (size_t i = this->client_nbr; i--;) {
        m_busy_clients[i] = false;
    }

    size_t nb_airways = (size_t) ceil(2 * M_PI / AIRWAY_SIZE);
    bool used_airway_this_turn[nb_airways];

    // init stats
    m_stats.initial_nb_drone = sdata->drone_nbr;
    m_stats.initial_nb_package = this->package_nbr;
    m_stats.nb_package_still_in_mothership = this->package_nbr;

    // wait for drones, hunters and clients.
    wait_for(m_remaining_drone_nbr + m_remaining_hunter_nbr + m_remaining_client_nbr);

    unsigned long tick_count = 0;
    bool had_msg = false;

    forever {
        init_timer();

        // check messages from drones
        for (size_t i = nb_airways; i--;) {
            used_airway_this_turn[i] = false;
        }
        message_t message;
        unsigned long int nbr_of_packages_that_can_be_loaded = this->package_throughput;
        while (msgrcv(m_msqid, &message, sizeof(message_t), getpid(), IPC_NOWAIT) != -1) {
            if (!had_msg) {
                char header[256];
                sprintf(header, "Tick %lu", tick_count);
                size_t len = strlen(header);

                printf("#------------------------------------------#\n");
                printf("|");
                for (size_t i = len / 2 ; i < 21 ; ++i) {
                    printf(" ");
                }

                for(size_t i = 0 ; i < len ; ++i) {
                    putchar(header[i]);
                }

                for (size_t i = len / 2 + len % 2; i < 21 ; ++i) {
                    printf(" ");
                }

                printf("|\n");
                printf("#------------------------------------------#\n\n");
            }
            had_msg = true;
            identity_t drone_id = find_drone_id_by_pid(message.pid);
            if (drone_id == BAD_ID) {
                printf(FLRED"Found no drone corresponding to the pid %d.\n"RESET, message.pid);
            } else {
                switch (message.msg_id) {
                    case ASK_DEPARTURE_MSG: {
                        if (m_drones_going_to_client[drone_id]) {
                            message_t answer = make_message(message.pid, DEPART_DRONE_MSG);
                            if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                fail_fast("ASK_DEPARTURE_MSG: msgsnd failed!");
                            }

                            m_drones_going_to_client[drone_id] = false;
                            m_busy_clients[this->packages[m_package_id_by_drone_id[drone_id]].client_id] = false;
                            m_package_id_by_drone_id[drone_id] = BAD_ID;
                            printf(FLBLUE"Authorized drone "FYELLOW""BOLD"#D%lu"RESET""FLBLUE" to come back to the mothership.\n"RESET, drone_id);
                        } else {
                            size_t airway_idx =
                                (size_t) this->clients[this->packages[m_package_id_by_drone_id[drone_id]].client_id].airway
                                + nb_airways / 2;
                            if (used_airway_this_turn[airway_idx]) {
                                //printf("Did not authorized drone %lu to leave the mothership: airway not available.\n", drone_id);
                            } else {
                                message_t answer = make_message(message.pid, DEPART_DRONE_MSG);
                                if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                    fail_fast("ASK_DEPARTURE_MSG: msgsnd failed!");
                                }

                                used_airway_this_turn[airway_idx] = true;
                                m_drones_going_to_client[drone_id] = true;
                                printf(FLBLUE"Authorized drone "FYELLOW""BOLD"#D%lu"RESET""FLBLUE" to leave the mothership.\n"RESET, drone_id);
                                add_flying_drone(message.pid);
                            }
                        }
                        break;
                    }
                    case NOTIFY_ARRIVAL_MSG: {
                        printf(FMAGENTA"Received notify arrival message.\n"RESET);
                        if (drone_is_flying(message.pid)) {
                            if (m_drones_going_to_client[drone_id]) {
                                printf(FMAGENTA"\tDrone "FYELLOW""BOLD"#D%lu"RESET""FMAGENTA" arrived to its client.\n"RESET, drone_id);
                                ++m_stats.nb_delivered_package;
                            } else {
                                printf(FMAGENTA"\tDrone "FYELLOW""BOLD"#D%lu"RESET""FMAGENTA" came back to the mothership.\n"RESET, drone_id);
                                remove_flying_drone(message.pid);
                            }
                        }
                        break;
                    }
                    case ASK_PACKAGE_MSG: {
                        if (nbr_of_packages_that_can_be_loaded > 0) {
                            printf(FCYAN"Drone "FYELLOW""BOLD"#D%lu"RESET""FCYAN" asked a package.\n"RESET, drone_id);
                            identity_t package_id;
                            if (find_appropriate_package_for_drone(message.identity_value, &package_id)) {
                                printf(FCYAN"\tAppropriate package found (package "FYELLOW""BOLD"#P%lu"RESET""FCYAN").\n"RESET, package_id);
                                message_t answer = make_identity_message(message.pid, LOAD_PACKAGE_MSG, package_id);
                                if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                    fail_fast("ASK_PACKAGE_MSG: msgsnd failed!");
                                }
                                m_busy_clients[this->packages[package_id].client_id] = true;
                                m_package_id_by_drone_id[drone_id] = package_id;
                                --nbr_of_packages_that_can_be_loaded;
                                --m_stats.nb_package_still_in_mothership;
                            } else {
                                printf(FCYAN"\tNo appropriate package for it. Powering it off.\n"RESET);
                                message_t answer = make_message(message.pid, POWER_OFF_MSG);
                                if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                    fail_fast("ASK_PACKAGE_MSG: msgsnd failed!");
                                }
                                ++m_stats.nb_powered_off_drone;
                            }
                        } else {
                            printf(FCYAN"Drone "FYELLOW""BOLD"#D%lu"RESET""FCYAN" asked a package, but has to wait for other drones to be served.\n"RESET, drone_id);
                        }
                        break;
                    }
                    case ASK_POWER_MSG: {
                        if (m_remaining_power_loading_slots > 0) {
                            printf(FGREEN"Give a power loading slot to drone "BOLD""FYELLOW"#D%lu"RESET""FGREEN".\n"RESET, drone_id);
                            ticks_t required_ticks = (ticks_t) ceil(message.double_value / this->power_throughput);
                            message_t answer = make_ticks_message(message.pid, POWER_DRONE_MSG, required_ticks);
                            if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                fail_fast("ASK_POWER_MSG: msgsnd failed!");
                            }
                            --m_remaining_power_loading_slots;
                        } else {
                            printf(FGREEN"Drone "BOLD""FYELLOW"#D%lu"RESET""FGREEN" requested a power loading slot, but there is none available.\n"RESET, drone_id);
                        }
                        break;
                    }
                    case END_POWER_MSG: {
                            printf(FGREEN"Drone "BOLD""FYELLOW"#D%lu"RESET""FGREEN" left a power loading slot.\n"RESET, drone_id);
                        ++m_remaining_power_loading_slots;
                        break;
                    }
                    default:
                        fail_fast("Unexpected message received.\n");
                        // no need to break
                }
                printf("\n");
            }
        }

        if (errno != ENOMSG) {
            perror("msgrcv");
            fail_fast("Aborting...\n");
        }

        if (had_msg) {
            printf("--------------------------------------------\n\n");
            had_msg = false;
        }
        tick();

        wait_timer();

        if (m_remaining_drone_nbr == 0) {
            break;
        }
        ++tick_count;
    }

    printf("\n-------- End simulation... --------\n");

    print_stats(m_stats);

    clean();
}

void init_timer() {
    gettimeofday(&m_beg_time, NULL);
}

void wait_timer() {
    struct timeval diff;
    gettimeofday(&diff, NULL);
    diff.tv_sec -= m_beg_time.tv_sec;
    if (diff.tv_usec >= m_beg_time.tv_usec) {
        diff.tv_usec -= m_beg_time.tv_usec;
    } else {
        --diff.tv_sec;
        diff.tv_usec = m_beg_time.tv_usec - diff.tv_usec;
    }

    diff.tv_sec = TICK_MIN_TIME_MSEC / 1000 - diff.tv_sec;
    diff.tv_usec = (TICK_MIN_TIME_MSEC - diff.tv_sec * 1000) * 1000 - diff.tv_usec;
    if (diff.tv_sec * 1000000 + diff.tv_usec > 0) {
        usleep((useconds_t)(diff.tv_sec * 1000000 + diff.tv_usec));
    }
}

void send_sig_to_list(int sig, pid_t* list, size_t len) {
    size_t i;
    for (i = 0; i < len; ++i) {
        kill(list[i], sig);
    }
}

void send_sig_to_all(int sig) {
    send_sig_to_list(sig, m_drones_p, m_sdata->drone_nbr);
    send_sig_to_list(sig, m_clients_p, m_sdata->mothership.client_nbr);
    send_sig_to_list(sig, m_hunters_p, m_sdata->hunter_nbr);
}

void tick() {
    m_is_drone_turn = true;
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_drones_p, m_sdata->drone_nbr);
    wait_for(m_remaining_drone_nbr);
    m_is_drone_turn = false;
    m_is_client_turn = true;
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_clients_p, m_sdata->mothership.client_nbr);
    wait_for(m_remaining_client_nbr);
    m_is_client_turn = false;
    m_is_hunter_turn = true;
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_hunters_p, m_sdata->hunter_nbr);
    wait_for(m_remaining_hunter_nbr);
    m_is_hunter_turn = false;
}

void fail_fast(const char* message) {
    puts(message);
    clean();
    exit(EXIT_FAILURE);
}

void wait_for(unsigned long value) {
    while(value--) {
        sem_wait(mother_sem);
    }
}

void clean() {
    signal(SIGCHLD, SIG_DFL);

    send_sig_to_all(SIGKILL);

    free(m_package_id_by_drone_id);
    free(m_remaining_packages);
    free(m_busy_clients);
    free(m_drones_going_to_client);
    free(m_drones_p);
    free(m_clients_p);
    free(m_hunters_p);

    sem_unlink(MOTHER_SEM_NAME);
    sem_close(mother_sem);
    sem_destroy(mother_sem);

    unmap_shared_memory();
    clean_shared_memory();

    msgctl(m_msqid, IPC_RMID, NULL);

    unload_simulation(m_sdata);
}

bool find_drone(identity_t id, drone_t* drone_found) {
    bool found = false;
    size_t i;
    for (i = m_sdata->drone_nbr; i--;) {
        if (m_sdata->drones[i].id == id) {
            found = true;
            *drone_found = m_sdata->drones[i];
            break;
        }
    }
    return found;
}

bool find_appropriate_package_for_drone(identity_t drone_id, identity_t* package_id_found) {
    drone_t drone;
    if (find_drone(drone_id, &drone)) {
        bool found = false;
        size_t i;
        for (i = this->package_nbr; i--;) {
            if (m_remaining_packages[i]
                    && this->packages[i].weight <= drone.max_package_weight
                    && this->packages[i].volume <= drone.max_package_volume
                    && drone.max_fuel * drone.speed > this->clients[this->packages[i].client_id].mothership_distance * 2
                    && !m_busy_clients[this->packages[i].client_id]) {
                found = true;
                *package_id_found = i;
                m_remaining_packages[i] = false;
                break;
            }
        }
        return found;
    } else {
        return false;
    }
}

// returns 0 if not found
identity_t find_drone_id_by_pid(pid_t drone_pid) {
    for (identity_t i = m_sdata->drone_nbr; i--;) {
        if (m_drones_p[i] == drone_pid) {
            return i;
        }
    }
    return BAD_ID;
}

void interruption_handler(int ignored) {
    fail_fast("Interruption handler...\n");
}

void sigchild_handler(int ignored) {
    int status;
    pid_t pid;
    unsigned int i;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        for (i = m_sdata->drone_nbr ; i-- ; ) {
            if (m_drones_p[i] == pid) {
                if (WIFEXITED(status)) {
                    int exit_status = WEXITSTATUS(status);
                    if (exit_status == GRACEFULLY_STOPPED) {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" gracefully stopped (pid %d).\n", find_drone_id_by_pid(pid), pid);
                    } else if (exit_status == EXPLODED) {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" exploded (pid %d).\n", find_drone_id_by_pid(pid), pid);
                    } else if (exit_status == UNEXPECTEDLY_STOPPED) {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" unexpectedly stopped (pid %d).\n", find_drone_id_by_pid(pid), pid);
                    } else if (exit_status == DIED) {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" died (pid %d).\n", find_drone_id_by_pid(pid), pid);
                    } else {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" exited for an unknown reason with status %d (pid %d).\n",
                                find_drone_id_by_pid(pid), exit_status, pid);
                    }
                } else if (WIFSIGNALED(status)) {
                    int sig = WTERMSIG(status);
                    if (sig == SIGKILL) {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" was killed (pid %d).\n", find_drone_id_by_pid(pid), pid);
                    } else {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" unexpectedly ended with sig %d (pid %d).\n",
                                find_drone_id_by_pid(pid), sig, pid);
                    }
                }

                if (drone_is_flying(pid)) {
                    remove_flying_drone(pid);
                }
                --m_remaining_drone_nbr;
                if (m_is_drone_turn) {
                    sem_post(mother_sem);
                }
                break;
            }
        }
        if (i == -1) {
            for (i = m_sdata->hunter_nbr ; i-- ;) {
                if (m_hunters_p[i] == pid) {
                    if (WIFEXITED(status)) {
                        int exit_status = WEXITSTATUS(status);
                        if (exit_status == GO_HOME) {
                            printf("/!\\ Hunter "BOLD""FYELLOW"#H%d"RESET" is going back home.\n", pid);
                        } else {
                            printf("/!\\ Hunter "BOLD""FYELLOW"#H%d"RESET" exited for an unknown reason with status %d.\n", pid, exit_status);
                        }
                    } else if (WIFSIGNALED(status)) {
                        printf("/!\\ Hunter "BOLD""FYELLOW"#H%d"RESET" unexpectedly ended with sig %d.\n", pid, WTERMSIG(status));
                    }

                    --m_remaining_hunter_nbr;
                    if (m_is_hunter_turn) {
                        sem_post(mother_sem);
                    }
                    break;
                }
            }

            if (i == -1) {
                for (i = m_sdata->mothership.client_nbr ; i-- ;) {
                    if (m_clients_p[i] == pid) {
                        printf("/!\\ Client died (pid %d).\n", pid);
                        --m_remaining_client_nbr;
                        if (m_is_client_turn) {
                            sem_post(mother_sem);
                        }
                        break;
                    }
                }

                if (i == -1) {
                    printf("Who died ?? its pid is %d\n", pid);
                }
            }
        }
        printf("\n");
    }
}

