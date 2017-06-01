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

#include "mothership.h"
#include "utility.h"
#include "mq_communication.h"
#include "parser.h"
#include "typedefs.h"

#define TICK_MIN_TIME_MSEC 500

static void init_timer(void);
static void wait_timer(void);
static void send_sig_to_list(int sig, pid_t* list, size_t len);
static void send_sig_to_all(int sig);
static void tick(void);
static void fail_fast(const char* message);
static void wait_for(unsigned long);
static void clean(void);
static void remove_available_drone(identity_t drone_id);
static void add_available_drone(identity_t drone_id);
static void swap_identity(identity_t* array, size_t i, size_t j);
static bool drone_is_available(identity_t drone_id);
static bool find_drone(identity_t id, drone_t* drone_found);
static bool find_appropriate_package_for_drone(identity_t drone_id, identity_t* package_id_found);
static identity_t find_drone_id_by_pid(pid_t drone_pid);

static void interruption_handler(int sig);

// OOP
static mothership_t* this;

static sim_data* m_sdata  = NULL;
static pid_t* m_drones_p  = NULL;
static pid_t* m_clients_p = NULL;
static pid_t* m_hunters_p = NULL;
static int m_msqid;

static unsigned long int m_remaining_power_loading_slots = 0;

static size_t m_available_drones_nbr;
static identity_t* m_available_drones;

struct timeval m_beg_time;

void sigchild_handler(int);

void sigchild_handler(int ignored) {
    int status;
    pid_t pid;
    unsigned int i;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        for (i = m_sdata->drone_nbr ; i-- ; ) {
            if (m_drones_p[i] == pid) {
                //TODO: something with status
                --m_sdata->drone_nbr;
                m_drones_p[i] = m_drones_p[m_sdata->drone_nbr];
                printf("Drone %d died.\n", pid);
                sem_post(mother_sem);
                break;
            }
        }
        if (i == -1) {
            for (i = m_sdata->hunter_nbr ; i-- ;) {
                if (m_hunters_p[i] == pid) {
                    // TODO: something with the status
                    --m_sdata->hunter_nbr;
                    m_hunters_p[i] = m_hunters_p[m_sdata->hunter_nbr];
                    printf("Client %d died.\n", pid);
                    sem_post(mother_sem);
                    break;
                }
            }

            if (i == -1) {
                for (i = m_sdata->mothership.client_nbr ; i-- ;) {
                    if (m_clients_p[i] == pid) {
                        printf("Hunter %d died.\n", pid);
                        // TODO: something with the status
                        --m_sdata->mothership.client_nbr;
                        m_clients_p[i] = m_clients_p[m_sdata->mothership.client_nbr];
                        sem_post(mother_sem);
                        break;
                    }
                }
                if (i == -1) {
                    printf("Who died ?? its pid is %d\n", pid);
                }
            }
        }
    }
}

void mothership_main(sim_data* sdata, pid_t* drones_p, pid_t* clients_p, pid_t* hunters_p, int msqid) {

    signal(SIGCHLD, &sigchild_handler);

    // initializing
    this = &sdata->mothership;

    m_sdata = sdata;
    m_drones_p = drones_p;
    m_clients_p = clients_p;
    m_hunters_p = hunters_p;
    m_msqid = msqid;

    m_remaining_power_loading_slots = m_sdata->mothership.power_loading_slots;

    m_available_drones_nbr = m_sdata->drone_nbr;
    m_available_drones = (identity_t*) malloc(sizeof(pid_t) * m_available_drones_nbr);

    wait_for(m_sdata->drone_nbr + m_sdata->hunter_nbr + this->client_nbr);
    signal(SIGINT, &interruption_handler);

    forever {
        init_timer();

        // check messages from drones
        printf("Mothership check messages.\n");
        message_t message;
        unsigned long int nbr_of_packages_that_can_be_loaded = this->package_throughput;
        while (msgrcv(m_msqid, &message, sizeof(message_t), getpid(), IPC_NOWAIT) != -1) {
            switch (message.msg_id) {
                case SHOOT_DRONE_MSG:
                    break;
                case ASK_DEPARTURE_MSG:
                    printf("Received ask departure message.\n");
                    break;
                case NOTIFY_ARRIVAL_MSG:
                    printf("Received notify arrival message.\n");
                    break;
                case ASK_PACKAGE_MSG:
                    printf("Received ask package message.\n");
                    if (nbr_of_packages_that_can_be_loaded > 0) {
                        identity_t package_id;
                        if (find_appropriate_package_for_drone(message.identity_value, &package_id)) {
                            message_t answer = make_identity_message(message.pid, LOAD_PACKAGE_MSG, package_id);
                            if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                fail_fast("ASK_PACKAGE_MSG: msgsnd failed!");
                            }
                            --nbr_of_packages_that_can_be_loaded;
                        } else {
                            message_t answer = make_message(message.pid, POWER_OFF_MSG);
                            if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                fail_fast("ASK_PACKAGE_MSG: msgsnd failed!");
                            }
                        }
                    }
                    break;
                case ASK_POWER_MSG:
                    printf("Received ask power message.\n");
                    if (m_remaining_power_loading_slots > 0) {
                        ticks_t required_ticks = (ticks_t) ceil(message.double_value / this->power_throughput);
                        message_t answer = make_ticks_message(message.pid, POWER_DRONE_MSG, required_ticks);
                        if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                            fail_fast("ASK_POWER_MSG: msgsnd failed!");
                        }
                        --m_remaining_power_loading_slots;
                    }
                    break;
                case END_POWER_MSG:
                    printf("Received end power message.\n");
                    ++m_remaining_power_loading_slots;
                    break;
                default:
                    fail_fast("Unexpected message received.\n");
                    // no need to break
            }
        }

        if (errno == ENOMSG) {
            printf("No more message.\n");
        } else {
            perror("msgrcv");
            fail_fast("Aborting...\n");
        }

        tick();

        wait_timer();
    }

    printf("exiting...\n");
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
    if (diff.tv_usec * 1000000 + diff.tv_usec > 0) {
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
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_drones_p, m_sdata->drone_nbr);
    wait_for(m_sdata->drone_nbr);
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_clients_p, m_sdata->mothership.client_nbr);
    wait_for(m_sdata->mothership.client_nbr);
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_hunters_p, m_sdata->hunter_nbr);
    wait_for(m_sdata->hunter_nbr);
}

void fail_fast(const char* message) {
    printf(message);
    clean();
    exit(EXIT_FAILURE);
}

void wait_for(unsigned long value) {
    while(value--) {
        sem_wait(mother_sem);
    }
}

void clean() {
    send_sig_to_all(SIGKILL);

    free(m_drones_p);
    free(m_clients_p);
    free(m_hunters_p);
    free(m_available_drones);

    sem_unlink(MOTHER_SEM_NAME);
    sem_close(mother_sem);
    sem_destroy(mother_sem);

    msgctl(m_msqid, IPC_RMID, NULL);

    unload_simulation(m_sdata);
}

void remove_available_drone(identity_t drone_id) {
    bool found = false;
    size_t i;
    for (i = 0; i < m_available_drones_nbr; i++) {
        if (m_available_drones[i] == drone_id) {
            found = true;
            swap_identity(m_available_drones, i, m_available_drones_nbr - 1);
        }
    }

    if (found)
        --m_available_drones_nbr;
}

void add_available_drone(identity_t drone_id) {
    m_available_drones[m_available_drones_nbr] = drone_id;
    ++m_available_drones_nbr;
}

void swap_identity(identity_t* array, size_t i, size_t j) {
    identity_t tmp = array[i];
    array[i] = array[j];
    array[j] = tmp;
}

bool drone_is_available(identity_t drone_id) {
    size_t i;
    for (i = m_available_drones_nbr; i--;) {
        if (m_available_drones[i] == drone_id) {
            return true;
        }
    }
    return false;
}

void interruption_handler(int sig) {
    fail_fast("Interruption handler...\n");
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
            if (this->packages[i].weight <= drone.max_package_weight
                    && this->packages[i].volume <= drone.max_package_volume) {
                found = true;
                *package_id_found = i;
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
    return 0;
}

