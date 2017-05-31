///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/msg.h>
#include <stdbool.h>
#include <errno.h>

#include "parser.h"
#include "utility.h"
#include "drone.h"
#include "mq_communication.h"

struct state {
    bool going_to_client;
    void(*run)(void);
};

static void flying_state(void);
static void delivering_state(void);
static void refueling_state(void);
static void loading_state(void);
static void waiting_departure_auth_state(void);

static void clean(void);
static void tick(void);
static void send(message_t*);

static drone_t m_me;
static message_t m_last_message;
static int m_msqid;
static struct state m_state;
static int* m_clients_pipes;
static unsigned int m_clients_nbr;
static int m_pipes[2];
static sim_data* m_sdata;

void drone_main(drone_t me, int* clients_pipes, unsigned int number_of_clients, int my_pipes[2], int msqid, sim_data* sdata) {

    sigset_t mask;
    sigaddset(&mask, MOTHERSHIP_SIGNAL);
    sigprocmask(SIG_BLOCK, &mask, NULL);

    m_me = me;
    m_msqid = msqid;
    m_clients_nbr = number_of_clients;
    m_pipes[0] = my_pipes[0];
    m_pipes[1] = my_pipes[1];

    m_state.going_to_client = true;
    m_state.run = &loading_state;

    m_sdata = sdata;

    // I am ready
    tick();

    m_me.fuel = my_pipes[1];

    forever {
        m_state.run();
    }

    clean();
    exit(UNEXPECTEDLY_STOPPED);
}

void flying_state() {
    if (m_state.going_to_client) {
        while (m_me.client_distance > 0) {
            --m_me.client_distance;
            ++m_me.mothership_distance;
            --m_me.fuel;
            tick();
        }
        m_state.run = &delivering_state;
    } else {
        while (m_me.mothership_distance > 0) {
            --m_me.mothership_distance;
            ++m_me.client_distance;
            --m_me.fuel;
            tick();
        }
        m_state.run = &refueling_state;
    }

    if (m_me.fuel < 0) {
        clean();
        exit(DIED);
    }
}

void delivering_state() {

    dprintf(m_clients_pipes[m_me.package->client_id * 2 + 1], "%d;%lf\n", m_pipes[0], m_me.package->volume);
    tick();

    FILE* client = fdopen(m_clients_pipes[m_me.package->client_id * 2], "r");
    unsigned long waiting_time;
    printf("tick\n");
    fscanf(client, "%lu", &waiting_time);
    //todo: what if the client does not want the package
    printf("tack\n");
    while(waiting_time--) {
        tick();
    }
    m_me.package = NULL;
    m_state.run = &waiting_departure_auth_state;
    m_state.going_to_client = false;
}

void waiting_departure_auth_state() {
    message_t message = make_identity_message(getppid(), ASK_DEPARTURE_MSG, m_me.id);
    do {
        send(&message);
        tick();
    } while (m_last_message.msg_id != DEPART_DRONE_MSG);
    m_state.run = &flying_state;
}

void refueling_state() {
    message_t message = make_double_message(getppid(), ASK_POWER_MSG, m_me.max_fuel - m_me.fuel);
    do {
        send(&message);
        tick();
    } while (m_last_message.msg_id != POWER_DRONE_MSG);

    int waiting_time = m_last_message.int_value;

    while(waiting_time--) {
        tick();
    }

    message.msg_id = END_POWER_MSG;
    send(&message);

    m_state.run = &loading_state;
}

void loading_state() {
    message_t message = make_identity_message(getppid(), ASK_PACKAGE_MSG, m_me.id);
    do {
        send(&message);
        tick();
    } while(m_last_message.msg_id != LOAD_PACKAGE_MSG);

    m_me.package = m_sdata->mothership.packages + m_last_message.int_value;
    m_me.client_distance = m_sdata->mothership.clients[m_me.package->client_id].mothership_distance;
    m_state.run = &waiting_departure_auth_state;
    m_state.going_to_client = true;
}

void tick() {
    sem_post(mother_sem);
    wait_mothership_signal();

    // Reading messages.
    m_last_message.msg_id = INVALID;
    message_t msg;
    while (msgrcv(m_msqid, &msg, sizeof(message_t), getpid(), IPC_NOWAIT) != -1) {
        switch (msg.msg_id) {
            case EXPLOSE_MSG:
                clean();
                exit(EXPLODED);
            case POWER_OFF_MSG:
                clean();
                exit(GRACEFULLY_STOPPED);
            default:
                m_last_message = msg;
        }
    }
}

void send(message_t* msg) {
    if (msgsnd(m_msqid, msg, sizeof(message_t), IPC_NOWAIT) == -1) {
        perror("MSGSND");
    }
}

void clean() {
    close_pipes(m_clients_nbr, m_clients_pipes);
    close(m_pipes[1]);
    free(m_clients_pipes);
    unload_simulation(m_sdata);
}
