#ifndef MOTIONPROCESSING_TYPES_H
#define MOTIONPROCESSING_TYPES_H

#if not defined(VQF_SINGLE_PRECISION) && BMI160_USE_VQF
    typedef double sensor_real_t;
#else
    typedef float sensor_real_t;
#endif

#endif