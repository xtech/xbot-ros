//
// Created by clemens on 26.07.24.
//

#include "ImuServiceInterface.h"

void ImuServiceInterface::OnAxesChanged(const double* new_value, uint32_t length) {
  return;
}

bool ImuServiceInterface::validateAxisConfig() {


  return true;
}

bool ImuServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  std::array<int8_t, 3> axis_remap = {1, -2, -3};  // Default (YardForce mainboard) mapping: X=1, Y=-2, Z=-3

  StartTransaction(true);

  if (validateAxisConfig()) {
    // Parse config string into axis_remap[] and axis_sign[]
    for (int i = 0; i < 3; ++i) {
      const size_t pos = i * 2;                                 // Position within config string (0, 2, 4)
      const int8_t sign = (axis_config_[pos] == '+') ? 1 : -1;  // Axis sign

      // ASCII-based axis calculation
      const char axis = axis_config_[pos + 1];
      const int8_t axis_num = static_cast<int8_t>((axis - 'X') + 1);  // X->1, Y->2, Z->3
      assert(axis_num >= 1 && axis_num <= 3);

      axis_remap[i] = sign * axis_num;
    }
  } else {

  }
  SetRegisterAxisRemap(axis_remap.data(), axis_remap.size());
  CommitTransaction();

  return true;
}
