// default definitions for the GLOVE
#ifndef MAX_SENSORS_COUNT
#define MAX_SENSORS_COUNT 10
#endif
#ifndef TRACKER_TYPE
#define TRACKER_TYPE TrackerType::TRACKER_TYPE_SVR_GLOVE_LEFT
#endif
#ifndef GLOVE_SIDE
#define GLOVE_SIDE GLOVE_LEFT
#endif
#ifndef PRIMARY_IMU_ADDRESS_ONE
#define PRIMARY_IMU_ADDRESS_ONE 0x4a
#endif
#ifndef SECONDARY_IMU_ADDRESS_TWO
#define SECONDARY_IMU_ADDRESS_TWO 0x4b
#endif

#ifndef SENSOR_DESC_LIST
#define SENSOR_DESC_LIST                                 \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		(PRIMARY_IMU_ADDRESS_ONE ^ 0x02),                \
		IMU_ROTATION,                                    \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA),           \
		false,                                           \
		MCP_PIN(MCP_GPA6),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		(SECONDARY_IMU_ADDRESS_TWO ^ 0x02),              \
		IMU_ROTATION,                                    \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA),           \
		true,                                            \
		MCP_PIN(MCP_GPA5),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 0), \
		true,                                            \
		MCP_PIN(MCP_GPB0),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 0), \
		true,                                            \
		MCP_PIN(MCP_GPB1),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 1), \
		true,                                            \
		MCP_PIN(MCP_GPB2),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 1), \
		true,                                            \
		MCP_PIN(MCP_GPB3),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 2), \
		true,                                            \
		MCP_PIN(MCP_GPB4),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 2), \
		true,                                            \
		MCP_PIN(MCP_GPB5),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 3), \
		true,                                            \
		MCP_PIN(MCP_GPB6),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 3), \
		true,                                            \
		MCP_PIN(MCP_GPA1),                               \
		0                                                \
	)
#endif

#ifndef SENSOR_INFO_LIST
#if GLOVE_SIDE == GLOVE_LEFT
#define SENSOR_INFO_LIST                                                    \
	SENSOR_INFO_ENTRY(0, SensorPosition::POSITION_LEFT_HAND)                \
	SENSOR_INFO_ENTRY(1, SensorPosition::POSITION_LEFT_LITTLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(2, SensorPosition::POSITION_LEFT_RING_INTERMEDIATE)   \
	SENSOR_INFO_ENTRY(3, SensorPosition::POSITION_LEFT_RING_DISTAL)         \
	SENSOR_INFO_ENTRY(4, SensorPosition::POSITION_LEFT_MIDDLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(5, SensorPosition::POSITION_LEFT_MIDDLE_DISTAL)       \
	SENSOR_INFO_ENTRY(6, SensorPosition::POSITION_LEFT_INDEX_INTERMEDIATE)  \
	SENSOR_INFO_ENTRY(7, SensorPosition::POSITION_LEFT_INDEX_DISTAL)        \
	SENSOR_INFO_ENTRY(8, SensorPosition::POSITION_LEFT_THUMB_PROXIMAL)      \
	SENSOR_INFO_ENTRY(9, SensorPosition::POSITION_LEFT_THUMB_DISTAL)
#elif GLOVE_SDIE == GLOVE_RIGHT
#define SENSOR_INFO_LIST                                                     \
	SENSOR_INFO_ENTRY(0, SensorPosition::POSITION_RIGHT_HAND)                \
	SENSOR_INFO_ENTRY(1, SensorPosition::POSITION_RIGHT_LITTLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(2, SensorPosition::POSITION_RIGHT_RING_INTERMEDIATE)   \
	SENSOR_INFO_ENTRY(3, SensorPosition::POSITION_RIGHT_RING_DISTAL)         \
	SENSOR_INFO_ENTRY(4, SensorPosition::POSITION_RIGHT_MIDDLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(5, SensorPosition::POSITION_RIGHT_MIDDLE_DISTAL)       \
	SENSOR_INFO_ENTRY(6, SensorPosition::POSITION_RIGHT_INDEX_INTERMEDIATE)  \
	SENSOR_INFO_ENTRY(7, SensorPosition::POSITION_RIGHT_INDEX_DISTAL)        \
	SENSOR_INFO_ENTRY(8, SensorPosition::POSITION_RIGHT_THUMB_PROXIMAL)      \
	SENSOR_INFO_ENTRY(9, SensorPosition::POSITION_RIGHT_THUMB_DISTAL)
#else  // GLOVE_SIDE
#error "Glove side not defined"
#endif  // GLOVE_SIDE
#endif
