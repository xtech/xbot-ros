//
// Created by clemens on 26.07.24.
//

#ifndef IMUSERVICEINTERFACE_H
#define IMUSERVICEINTERFACE_H


#include <ImuServiceInterfaceBase.hpp>

class ImuServiceInterface : public ImuServiceInterfaceBase {
 public:
  ImuServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx)
      : ImuServiceInterfaceBase(service_id, ctx) {
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnAxesChanged(const double* new_value, uint32_t length) override;

 private:
  std::string axis_config_;
bool validateAxisConfig();
};

#endif  // IMUSERVICEINTERFACE_H
