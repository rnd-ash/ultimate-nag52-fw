#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "lib/egs52_ecus/src/GS.h"

using namespace std;

GS_218_EGS52 gs218;


#define LOOKUP_VALUE(v) \
    std::cout << #v << " is " << (int)v << std::endl;

int main() {
    gs218.raw = 0x0000225000000030;

    LOOKUP_VALUE(gs218.FEHLPRF_ST);
    LOOKUP_VALUE(gs218.MKRIECH);
    LOOKUP_VALUE(gs218.MOT_NAUS);
    LOOKUP_VALUE(gs218.MOT_NAUS_CNF);
    LOOKUP_VALUE(gs218.K_LSTFR);
    LOOKUP_VALUE(gs218.DYN0_AMR_EGS);
    LOOKUP_VALUE(gs218.DYN1_EGS);
    LOOKUP_VALUE(gs218.MPAR_EGS);
    LOOKUP_VALUE(gs218.FPC_AAD);
    LOOKUP_VALUE(gs218.KD);
    LOOKUP_VALUE(gs218.UEHITZ_GET);
    LOOKUP_VALUE(gs218.GS_NOTL);
    LOOKUP_VALUE(gs218.ALF);
    LOOKUP_VALUE(gs218.KS);
    LOOKUP_VALUE(gs218.GET_OK);
    LOOKUP_VALUE(gs218.HSM);
    LOOKUP_VALUE(gs218.SCHALT);
    LOOKUP_VALUE(gs218.FW_HOCH);
    LOOKUP_VALUE(gs218.GSP_OK);
    LOOKUP_VALUE(gs218.G_G);
    LOOKUP_VALUE(gs218.K_G_B);
    LOOKUP_VALUE(gs218.K_O_B);
    LOOKUP_VALUE(gs218.K_S_B);
    return 0;
}