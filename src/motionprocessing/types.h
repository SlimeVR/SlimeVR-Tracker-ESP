#ifndef MOTIONPROCESSING_TYPES_H
#define MOTIONPROCESSING_TYPES_H

#if SENSORS_DOUBLE_PRECISION
typedef double sensor_real_t;
#else
typedef float sensor_real_t;
#define VQF_SINGLE_PRECISION
#endif

#endif
