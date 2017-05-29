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

#include "mothership.h"
#include "utility.h"
#include "mq_communication.h"

#define TICK_MIN_TIME_MSEC 500

static void send_sig_to_list(int sig, pid_t* list, size_t len);
static void tick(void);
static void send_sig_to_all(int sig);
static void fail_fast(const char* message);
static void interruption_handler(int sig);
static void wait_for(unsigned long);

static void clean(void);

// OOP
static sim_data* m_sdata  = NULL;
static pid_t* m_drones_p  = NULL;
static pid_t* m_clients_p = NULL;
static pid_t* m_hunters_p = NULL;
static int m_msqid;

void mothership_main(sim_data* sdata, pid_t* drones_p, pid_t* clients_p, pid_t* hunters_p, int msqid) {
    // initializing
    m_sdata = sdata;
    m_drones_p = drones_p;
    m_clients_p = clients_p;
    m_hunters_p = hunters_p;
    m_msqid = msqid;

    wait_for(m_sdata->drone_nbr + m_sdata->hunter_nbr + m_sdata->mothership.client_nbr);
    signal(SIGINT, &interruption_handler);

    struct timeval beg_time, diff;
    for (;;) {
        gettimeofday(&beg_time, NULL);
        // 1 tick
        // check messages from drones
        printf("Mothership check messages.\n");
        bool continue_read = true;
        while (continue_read) {
            message_t message;
            if (msgrcv(m_msqid, &message, sizeof(message_t), getpid(), IPC_NOWAIT) == -1) {
                if (errno == ENOMSG) {
                    continue_read = false;
                    printf("No more message.\n");
                } else {
                    perror("msgrcv");
                    fail_fast("Aborting...\n");
                }
            } else {
                switch (message.msg_id) {
                    case DRONE_MSG:
                        printf("received drone message.\n");
                        break;
                    case HUNTER_MSG:
                        printf("received hunter message.\n");
                        break;
                    default:
                        fail_fast("unexpected message received.\n");
                        // no need to break
                }
            }
        }

        tick();

        gettimeofday(&diff, NULL);
        diff.tv_sec -= beg_time.tv_sec;
        if (diff.tv_usec >= beg_time.tv_usec) {
            diff.tv_usec -= beg_time.tv_usec;
        } else {
            --diff.tv_sec;
            diff.tv_usec = beg_time.tv_usec - diff.tv_usec;
        }

        diff.tv_sec = TICK_MIN_TIME_MSEC / 1000 - diff.tv_sec;
        diff.tv_usec = (TICK_MIN_TIME_MSEC - diff.tv_sec * 1000) * 1000 - diff.tv_usec;
        if (diff.tv_usec * 1000000 + diff.tv_usec > 0) {
            usleep((useconds_t)(diff.tv_sec * 1000000 + diff.tv_usec));
        }
    }

    printf("exiting...\n");
    clean();
}

void send_sig_to_list(int sig, pid_t* list, size_t len) {
    size_t i;
    for (i = 0; i < len; ++i) {
        kill(list[i], sig);
    }
}

void tick() {
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_drones_p, m_sdata->drone_nbr);
    wait_for(m_sdata->drone_nbr);
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_clients_p, m_sdata->mothership.client_nbr);
    wait_for(m_sdata->mothership.client_nbr);
    send_sig_to_list(MOTHERSHIP_SIGNAL, m_hunters_p, m_sdata->hunter_nbr);
    wait_for(m_sdata->hunter_nbr);
}

void send_sig_to_all(int sig) {
    send_sig_to_list(sig, m_drones_p, m_sdata->drone_nbr);
    send_sig_to_list(sig, m_clients_p, m_sdata->mothership.client_nbr);
    send_sig_to_list(sig, m_hunters_p, m_sdata->hunter_nbr);
}

void fail_fast(const char* message) {
    printf(message);
    clean();
    abort();
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
    sem_unlink(MOTHER_SEM_NAME);
    sem_close(mother_sem);
    sem_destroy(mother_sem);
    msgctl(m_msqid, IPC_RMID, NULL);
}

void interruption_handler(int sig) {
    fail_fast("Interruption handler...\n");
}

