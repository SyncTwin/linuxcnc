/********************************************************************
* Description: halnats_main.cc
*   halnats - the Sparkplug node (devices hal and scope) of a VM that runs HAL
*   without LinuxCNC (automation nodes: HAL + soft-PLC). Same library as
*   milltask's publisher (halnats.hh); where LinuxCNC runs, the task publishes
*   and halnats refuses (one publisher per VM).
*
*   halnats -c <pins file> [-i <ini>]
*     connection from the environment (SYNCTWIN_NATS_URL/USER/PASS,
*     SYNCTWIN_BOX_ID, NODE_ID), the INI [NATS] keys override; the pins file
*     is the hal2spb conf line format. loadusr -W halnats -c ... or a unit.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#include "halnats.hh"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "hal.h"

static volatile sig_atomic_t done = 0;
static void on_sig(int) { done = 1; }

int main(int argc, char **argv)
{
    const char *pins = nullptr, *ini = nullptr;
    int o;
    while ((o = getopt(argc, argv, "c:i:")) != -1) {
        if (o == 'c') pins = optarg;
        else if (o == 'i') ini = optarg;
        else { fprintf(stderr, "usage: halnats -c <pins file> [-i <ini>]\n"); return 1; }
    }
    if (!pins && !ini) { fprintf(stderr, "usage: halnats -c <pins file> [-i <ini>]\n"); return 1; }

    halnats::Config c;
    c.tag = "halnats";
    int r = halnats::configure(c, ini, "", 0);   // period: INI PUBLISH_PERIOD, else the pins file's hz
    if (r) return r > 0 ? 0 : 1;
    if (pins) c.pins = pins;

    int comp = hal_init("halnats");
    if (comp < 0) { fprintf(stderr, "halnats: hal_init %d\n", comp); return 1; }
    hal_query_t q;
    memset(&q, 0, sizeof q);
    if (hal_comp_by_name("iocontrol.0", &q) == 0) {
        fprintf(stderr, "halnats: LinuxCNC runs on this HAL (iocontrol.0) - its task publishes ([NATS] PINS); "
                        "one publisher per VM, halnats refused\n");
        hal_exit(comp);
        return 1;
    }
    hal_ready(comp);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    if (halnats::start(c, comp, nullptr)) { hal_exit(comp); return 1; }
    while (!done) pause();
    halnats::stop();
    hal_exit(comp);
    return 0;
}
