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

// Method we're in when flying
static void flying_state(void);

// Method we're in when we're delivering a client
static void delivering_state(void);

// Method we're in when we're refueling
static void refueling_state(void);

// Method we're in when we're trying to load a package
static void loading_state(void);

// Method we're in when we're waiting for the mothership's authorization to go.
static void waiting_departure_auth_state(void);

// Method to clean the drone's data
static void clean(void);

// Method to call to wait for the mothership's tick
static void tick(void);
// For the mothership's message queue
static void send(message_t*);

// last message we received
static message_t m_last_message;
// our current state
static struct state m_state;

static int* m_clients_pipes;
static unsigned int m_clients_nbr;
static sim_data* m_sdata;
static drone_t m_me;
static int m_msqid;

void drone_main(drone_t me, int* clients_pipes, unsigned int number_of_clients, int msqid, sim_data* sdata) {

    // initialisation
    sigset_t mask;
    sigaddset(&mask, MOTHERSHIP_SIGNAL);
    sigprocmask(SIG_BLOCK, &mask, NULL);

    m_me = me;
    m_msqid = msqid;
    m_clients_nbr = number_of_clients;

    m_state.going_to_client = true;
    m_state.run = &loading_state;
    m_clients_pipes = clients_pipes;

    m_sdata = sdata;

    // I am ready
    tick();

    // running until death (by crash, bullet in the knee, mothership's poweroff,...)
    forever {
        m_state.run();
    }

    clean();
    exit(UNEXPECTEDLY_STOPPED);
}

void flying_state() {
    // We're flying!
    if (m_state.going_to_client) {
        while (m_me.client_distance > 0) {
            m_me.client_distance -= m_me.speed;
            m_me.mothership_distance += m_me.speed;
            --m_me.fuel;
            tick();
            if (m_me.fuel < 0) {
                // We just crashed! :(
                printf("Crashing !\n");
                clean();
                exit(CRASHED);
            }
        }
        // updating the next state
        m_state.run = &delivering_state;
    } else {
        while (m_me.mothership_distance > 0) {
            m_me.mothership_distance -= m_me.speed;
            m_me.client_distance += m_me.speed;
            --m_me.fuel;
            tick();
            if (m_me.fuel < 0) {
                // We just crashed! :(
                printf("Crashing !\n");
                clean();
                exit(CRASHED);
            }
        }
        // updating the next state
        m_state.run = &refueling_state;
    }

    message_t message = make_message(getppid(), NOTIFY_ARRIVAL_MSG);
    send(&message);
}

void delivering_state() {

    // telling the client the size of our package
    dprintf(m_clients_pipes[m_me.package->client_id * 2 + 1], "%lf\n", m_me.package->volume);
    tick();

    char msg[256];
    read(m_clients_pipes[m_me.package->client_id * 2], msg, 256);
    unsigned long waiting_time = strtoul(msg, NULL, 10);

    // waiting until the client had enough time to take the package
    while(waiting_time--) {
        tick();
    }

    // updating next state
    m_state.run = &waiting_departure_auth_state;
    m_state.going_to_client = false;
    m_me.package = NULL;
}

void waiting_departure_auth_state() {
    // Sending a departure request to the mothership until it is fulfilled
    message_t message = make_identity_message(getppid(), ASK_DEPARTURE_MSG, m_me.id);
    do {
        send(&message);
        tick();
    } while (m_last_message.msg_id != DEPART_DRONE_MSG);

    // updating next state
    m_state.run = &flying_state;
}

void refueling_state() {

    // Sending a reloading request to the mothership until it is fulfilled
    message_t message = make_double_message(getppid(), ASK_POWER_MSG, m_me.max_fuel - m_me.fuel);
    do {
        send(&message);
        tick();
    } while (m_last_message.msg_id != POWER_DRONE_MSG);
    int waiting_time = m_last_message.int_value;

    // plugging
    tick();

    // refueling until full
    while(waiting_time--) {
        tick();
    }
    m_me.fuel = m_me.max_fuel;

    // one extra tick to unplug.
    tick();

    // we finished loading
    message.msg_id = END_POWER_MSG;
    send(&message);

    // updating next state
    m_state.run = &loading_state;
}

void loading_state() {

    // Asking a package to the mothership until it gives us one.
    message_t message = make_identity_message(getppid(), ASK_PACKAGE_MSG, m_me.id);
    do {
        send(&message);
        tick();
    } while(m_last_message.msg_id != LOAD_PACKAGE_MSG);

    // updating next state and package
    m_state.run = &waiting_departure_auth_state;
    m_state.going_to_client = true;
    m_me.package = m_sdata->mothership.packages + m_last_message.int_value;
    m_me.client_distance = m_sdata->mothership.clients[m_me.package->client_id].mothership_distance;
}

void tick() {
    // we finished our tick ; freeing resource for the mothership
    sem_post(mother_sem);

    // waiting for a mothership's signal.
    wait_mothership_signal();

    // Reading messages.
    m_last_message.msg_id = INVALID;
    message_t msg;
    while (msgrcv(m_msqid, &msg, sizeof(message_t), getpid(), IPC_NOWAIT) != -1) {
        switch (msg.msg_id) {
            case EXPLOSE_MSG:
                // a hunter killed us
                clean();
                exit(EXPLODED);
            case POWER_OFF_MSG:
                // the mothership powered us off
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
    // freeing resources
    close_pipes(m_clients_nbr, m_clients_pipes);
    free(m_clients_pipes);
    unload_simulation(m_sdata);

    // we technically finished our tick
    sem_close(mother_sem);
}
