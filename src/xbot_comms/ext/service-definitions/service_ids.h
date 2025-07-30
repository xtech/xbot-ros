#ifndef SERVICE_IDS_H
#define SERVICE_IDS_H

#include <cstdint>

namespace xbot::service_ids {
enum : uint16_t { EMERGENCY = 1, DIFF_DRIVE = 2, MOWER = 3, IMU = 4, POWER = 5, GPS = 6, INPUT = 7, BUMPER = 9 };
}

#endif  // SERVICE_IDS_H
