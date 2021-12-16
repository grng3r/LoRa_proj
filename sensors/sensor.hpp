#pragma once

struct sensorData_t {
    //GPS values
    float lat;
    float lon;
    //light
    float light;
    //moisture
    float moist;
    //temp&hum
    int temp;
    unsigned int hum;
    //RGB
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void sensors_init();

void sensors_update(sensorData_t* data);
