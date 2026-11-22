#include "can_rules.h"
#include <string.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>

// serie_id выбранного авто — из имени /sd/f_01<SSSS><VV>.bin (fileId type=1).
uint16_t can_serie_from_sd(void) {
    DIR *dp = opendir("/sd");
    if (!dp) return 0;
    struct dirent *e; uint16_t serie = 0;
    while ((e = readdir(dp))) {
        if (!strncmp(e->d_name, "f_01", 4)) {
            unsigned long fid = strtoul(e->d_name + 2, NULL, 16);
            serie = (uint16_t)((fid >> 8) & 0xFFFF);
        }
    }
    closedir(dp);
    return serie;
}

const car_can_t *can_rules_find(uint16_t serie) {
    for (int i = 0; i < g_car_can_n; i++)
        if (g_car_can[i].serie == serie) return &g_car_can[i];
    return NULL;
}
