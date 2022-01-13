#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <vector>

using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;

using float32 = float;
using float64 = double;

struct ParametersIHM
{
    float32 pulleyRadius;
    float32 motorAccuracy;
    uint8 motorAccuracyDivider;
    uint16 linVelMotorMast;
    uint8 dutyCycle;
    uint8 restTime;
};

struct Velocities
{
    float64 vx;
    float64 vy;
    float64 theta_z;
};

struct Delay
{
    // en secondes
    uint8 restDelay;
    uint16 measureDelay;
    float32 periodDelay;
};

struct HeightsList
{
    std::vector<uint32> heights;
};

typedef struct ParametersIHM ParametersIHM;
typedef struct Velocities Velocities;
typedef struct HeightsList HeightsList;
typedef int socklen_t;

enum Index {
	VELOCITIES = 1, 
	PARAMETERS = 2, 
	HEIGHTS = 3, 
	DELAY = 4,
    INIT_SENSOR_POS = 5, 
	RPI_GO = 6, 
	U_IHM = 7, 
	EOM = 8,
	NEXT_MEASURE = 9,
	VEL_VOLT_OUT_TYPE = 10,
	STOP_MEASURE = 11
};

enum VelocityVoltageOutputType {
	TYPE_0_5_V = 0,
	TYPE_1_5_V = 1,
	TYPE_0_10_V = 2,
	TYPE_2_10_V = 3,
	TYPE_0_20_mA = 4,
	TYPE_4_20_mA = 5
};

namespace Havry{
    union Float32ConversionHelper
    {
        static_assert(sizeof(float32) == sizeof(uint32), "");
        float32 f;
        uint32 u;
    };

    union Float64ConversionHelper
    {
        static_assert(sizeof(float64) == sizeof(uint64), "");
        float64 f;
        uint64 u;
    };
}

#endif //TYPES_H