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
#include <stdarg.h>

#include "colors.h"
#include "mothership.h"
#include "utility.h"
#include "mq_communication.h"
#include "shm_communication.h"
#include "parser.h"
#include "typedefs.h"
#include "stats.h"

#define COLUMNS 85
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
static size_t m_destroyed_pkg;

static identity_t* m_package_id_by_drone_id;
static bool* m_drones_going_to_client;
static bool* m_remaining_packages;
static bool* m_busy_clients;

static sim_stats m_stats;

// === implementations

void print_col(const char* format, ...);

void print_col(const char* format, ...) {
 	char text[COLUMNS];
	va_list valist;
	va_start(valist, format);
	int char_length;
	if ((char_length = vsprintf(text, format, valist)) < 0){
		va_end(valist);
		return;
	}
	va_end(valist);
    printf("║");
    long ignored_chars = 0;
    for (long i = 0 ; i < char_length ; ++i) {
        if (text[i] == '$') {
            ++i;
            ignored_chars += 2;
            switch (text[i]) {
                case 'C':
                    printf(FCYAN);
                    break;
                case 'G':
                    printf(FGREEN);
                    break;
                case 'R':
                    printf(FRED);
                    break;
                case 'Y':
                    printf(FYELLOW);
                    break;
                case 'B':
                    printf(FBLUE);
                    break;
                case 'M':
                    printf(FMAGENTA);
                    break;
                case 'c':
                    printf(FLCYAN);
                    break;
                case 'g':
                    printf(FLGREEN);
                    break;
                case 'r':
                    printf(FLRED);
                    break;
                case 'y':
                    printf(FLYELLOW);
                    break;
                case 'b':
                    printf(FLBLUE);
                    break;
                case 'm':
                    printf(FLMAGENTA);
                    break;
                case '-':
                    printf(RESET);
                    break;
                case '+':
                    printf(BOLD);
                    break;
                case '$':
                    putchar('$');
                    break;
                default:
                    break;
            }
        } else {
            putchar(text[i]);
        }
    }
    for (long i = char_length - ignored_chars ; i < COLUMNS - 2 ; ++i) {
        putchar(' ');
    }
    printf("║\n");
}

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
    m_stats.crashed_drones = 0;
    m_stats.shot_drones = 0;
    m_stats.initial_nb_package = this->package_nbr;
    m_stats.nb_package_still_in_mothership = this->package_nbr;
    m_stats.tick_count = 0;
    m_stats.power_consumption = 0;

    m_destroyed_pkg = 0;

    // wait for drones, hunters and clients.
    wait_for(m_remaining_drone_nbr + m_remaining_hunter_nbr + m_remaining_client_nbr);

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
                sprintf(header, "[%lu/%d/%d] Drones ─ Tick #%lu ─ [%u/%lu/%u] Packages",
                        m_remaining_drone_nbr,m_stats.initial_nb_drone - m_stats.crashed_drones - m_stats.shot_drones, m_stats.initial_nb_drone,
                        m_stats.tick_count,
                        m_stats.nb_package_still_in_mothership, m_stats.initial_nb_package - m_destroyed_pkg, m_stats.initial_nb_package);
                size_t len = strlen(header);

                printf("╔");
                for (size_t i = COLUMNS - 2 ; i-- ;) {
                    printf("═");
                }
                printf("╗\n");
                printf("║");
                for (size_t i = len / 2 ; i < COLUMNS / 2 + 1; ++i) {
                    printf(" ");
                }

                for(size_t i = 0 ; i < len ; ++i) {
                    putchar(header[i]);
                }

                for (size_t i = len / 2 + len % 2; i < COLUMNS / 2 + 2 ; ++i) {
                    printf(" ");
                }

                printf("║\n");
                printf("╟");
                for (size_t i = COLUMNS - 2 ; i-- ;) {
                    printf("─");
                }
                printf("╢\n");
                print_col("");
                print_col("");
            }
            had_msg = true;
            identity_t drone_id = find_drone_id_by_pid(message.pid);
            if (drone_id == BAD_ID) {
                print_col("$r    Found no drone corresponding to the pid %d."RESET, message.pid);
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
                            print_col("$b    Authorized drone $Y$+#D%lu$-$b to come back to the mothership.$-", drone_id);
                        } else {
                            size_t airway_idx =
                                (size_t) this->clients[this->packages[m_package_id_by_drone_id[drone_id]].client_id].airway
                                + nb_airways / 2;
                            if (used_airway_this_turn[airway_idx]) {
                                print_col("$b    Did not authorized drone $Y$+#D%lu$-$b to leave the mothership: airway not available.$-", drone_id);
                            } else {
                                message_t answer = make_message(message.pid, DEPART_DRONE_MSG);
                                if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                    fail_fast("ASK_DEPARTURE_MSG: msgsnd failed!");
                                }

                                used_airway_this_turn[airway_idx] = true;
                                m_drones_going_to_client[drone_id] = true;
                                print_col("$b    Authorized drone $Y$+#D%lu$-$b to leave the mothership.$-", drone_id);
                                add_flying_drone(message.pid);
                            }
                        }
                        break;
                    }
                    case NOTIFY_ARRIVAL_MSG: {
                        print_col("$M    Received notify arrival message.$-");
                        if (drone_is_flying(message.pid)) {
                            if (m_drones_going_to_client[drone_id]) {
                                print_col("$M        Drone $Y$+#D%lu$-$M arrived to its client.$-", drone_id);
                                ++m_stats.nb_delivered_package;
                            } else {
                                print_col("$M        Drone $Y$+#D%lu$-$M came back to the mothership.$-", drone_id);
                                remove_flying_drone(message.pid);
                            }
                        }
                        break;
                    }
                    case ASK_PACKAGE_MSG: {
                        if (nbr_of_packages_that_can_be_loaded > 0) {
                            print_col("$C    Drone $Y$+#D%lu$-$C asked a package.$-", drone_id);
                            identity_t package_id;
                            if (find_appropriate_package_for_drone(message.identity_value, &package_id)) {
                                print_col("$C        Appropriate package found (package $Y$+#P%lu$-$C).$-", package_id);
                                message_t answer = make_identity_message(message.pid, LOAD_PACKAGE_MSG, package_id);
                                if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                    fail_fast("ASK_PACKAGE_MSG: msgsnd failed!");
                                }
                                m_busy_clients[this->packages[package_id].client_id] = true;
                                m_package_id_by_drone_id[drone_id] = package_id;
                                --nbr_of_packages_that_can_be_loaded;
                                --m_stats.nb_package_still_in_mothership;
                            } else {
                                print_col("$C        No appropriate package for it. Powering it off.$-");
                                message_t answer = make_message(message.pid, POWER_OFF_MSG);
                                if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                    fail_fast("ASK_PACKAGE_MSG: msgsnd failed!");
                                }
                            }
                        } else {
                            print_col("$C    Drone $Y$+#D%lu$-$C asked a package, but has to wait for other drones to be served.$-", drone_id);
                        }
                        break;
                    }
                    case ASK_POWER_MSG: {
                        if (m_remaining_power_loading_slots > 0) {
                            print_col("$G    Give a power loading slot to drone $Y$+#D%lu$-$G.$-", drone_id);
                            ticks_t required_ticks = (ticks_t) ceil(message.double_value / this->power_throughput);
                            m_stats.power_consumption += message.double_value;
                            message_t answer = make_ticks_message(message.pid, POWER_DRONE_MSG, required_ticks);
                            if (msgsnd(msqid, &answer, sizeof(message_t), IPC_NOWAIT) == -1) {
                                fail_fast("ASK_POWER_MSG: msgsnd failed!");
                            }
                            --m_remaining_power_loading_slots;
                        } else {
                            print_col("$G    Drone $Y$+#D%lu$-$G requested a power loading slot, but there is none available.$-", drone_id);
                        }
                        break;
                    }
                    case END_POWER_MSG: {
                            print_col("$G    Drone $Y$+#D%lu$-$G left a power loading slot.$-", drone_id);
                        ++m_remaining_power_loading_slots;
                        break;
                    }
                    default:
                        fail_fast("Unexpected message received.\n");
                        // no need to break
                }
                print_col("");
            }
        }

        if (errno != ENOMSG) {
            perror("msgrcv");
            fail_fast("Aborting...\n");
        }

        if (had_msg) {
            print_col("");
            printf("╚");
            for (size_t i = COLUMNS - 2 ; i-- ;) {
               printf("═");
            }
            printf("╝\n\n");
            had_msg = false;
        }
        tick();

        wait_timer();

        if (m_remaining_drone_nbr == 0) {
            break;
        }
        ++m_stats.tick_count;
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

    diff.tv_sec = m_sdata->simulation_speed / 1000 - diff.tv_sec;
    diff.tv_usec = (m_sdata->simulation_speed - diff.tv_sec * 1000) * 1000 - diff.tv_usec;
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
        unsigned long priority = 0;
        bool found = false;
        size_t i;
        for (i = this->package_nbr; i--;) {
            if (m_remaining_packages[i]
                    && this->packages[i].weight <= drone.max_package_weight
                    && this->packages[i].volume <= drone.max_package_volume
                    && drone.max_fuel * drone.speed > this->clients[this->packages[i].client_id].mothership_distance * 2
                    && !m_busy_clients[this->packages[i].client_id]
                    && priority <= this->packages[i].priority) {
                priority = this->packages[i].priority;
                *package_id_found = i;
                found = true;
            }
        }
        if (found) {
            m_remaining_packages[*package_id_found] = false;
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
                    } else if (exit_status == CRASHED) {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" crashed (pid %d).\n", find_drone_id_by_pid(pid), pid);
                        ++m_stats.crashed_drones;
                    } else {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" exited for an unknown reason with status %d (pid %d).\n",
                                find_drone_id_by_pid(pid), exit_status, pid);
                    }
                } else if (WIFSIGNALED(status)) {
                    int sig = WTERMSIG(status);
                    if (sig == SIGKILL) {
                        ++m_stats.shot_drones;
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" was killed (pid %d).\n", find_drone_id_by_pid(pid), pid);
                    } else {
                        printf("/!\\ Drone "BOLD""FYELLOW"#D%lu"RESET" unexpectedly ended with sig %d (pid %d).\n",
                                find_drone_id_by_pid(pid), sig, pid);
                    }
                }

                if (drone_is_flying(pid)) {
                    remove_flying_drone(pid);
                    identity_t id = find_drone_id_by_pid(pid);
                    if (id != BAD_ID && m_drones_going_to_client[id]) {
                        ++m_destroyed_pkg;
                    }
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

