Header header

# identification of running internal algorithm
uint8 algorithm

# OQI: algorithm specific quality indicator
uint8 operational_quality_indicator

uint8 ALGO_NULL = 0
uint8 ALGO_PASS_THROUGH = 1
uint8 ALGO_EXTRAPOLATE_GNSS = 2
uint8 ALGO_FUSE_GNSS_IMU = 3
uint8 ALGO_FUSE_GNSS_IMU_ODOM = 4

uint8 OQI_UNKNOWN = 0
uint8 OQI_FAULT = 1      # failure, incorrect results
uint8 OQI_HICCUPS = 128  # temporary outages (e.g. lower output rate)
uint8 OQI_BEST = 255     # operating as specified
