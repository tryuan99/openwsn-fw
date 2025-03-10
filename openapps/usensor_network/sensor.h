#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>

// Sensor type enum.
typedef enum {
  SENSOR_TYPE_INVALID = -1,
  SENSOR_TYPE_POTENTIOMETRIC = 0,
  SENSOR_TYPE_RESISTIVE = 1,
  SENSOR_TYPE_PH = 2,
} sensor_type_e;

// Sensor config union.
typedef union {
} sensor_config_t;

// Sensor measurement union.
typedef union {
  // ADC output.
  uint16_t adc_output;
} sensor_measurement_t;

#endif // __SENSOR_H
