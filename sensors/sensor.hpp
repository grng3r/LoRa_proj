#pragma once

#include <cstdint>
struct sensorData_t {
    //GPS values
    float lat;//4
    float lon;//4
    //light
    float light;//4
    //moisture
    float moist;//4
    //temp&hum
    int temp;//4
    unsigned int hum;//4
    //RGB
    char rgb;//1
    //Acc x
    float acc_x;//4
    //Acc y
    //float acc_y;
};

void sensors_init();

void sensors_update(sensorData_t* data);
