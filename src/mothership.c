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

#include "mothership.h"
#include "utility.h"

static void send_sig_to_list(int sig, pid_t* list, size_t len);
static void send_sig_to_all(int sig);
static void fail_fast(const char* message);

static sim_data* m_sdata  = NULL;
static pid_t* m_drones_p  = NULL;
static pid_t* m_clients_p = NULL;
static pid_t* m_hunters_p = NULL;
static int m_msgid;

void interruption_handler(int sig) {
    fail_fast("Interruption handler...\n");
}

void mothership_main(sim_data* sdata, pid_t* drones_p, pid_t* clients_p, pid_t* hunters_p, int msgid) {
    // initializing
    m_sdata = sdata;
    m_drones_p = drones_p;
    m_clients_p = clients_p;
    m_hunters_p = hunters_p;
    m_msgid = msgid;

    signal(SIGINT, &interruption_handler);

    for (;;) {
        // 1 tick

        // TODO
        printf("mothership tick\n");
        send_sig_to_all(MOTHERSHIP_SIGNAL);
        sleep(1);
    }

    printf("exiting...\n");
    send_sig_to_all(SIGKILL);
    free(drones_p);
    free(clients_p);
    free(hunters_p);
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

void fail_fast(const char* message) {
    printf(message);
    send_sig_to_all(SIGKILL);
    msgctl(m_msgid, IPC_RMID, NULL);
    abort();
}

