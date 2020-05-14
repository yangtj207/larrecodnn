#ifndef IWaveformRecog_H
#define IWaveformRecog_H

#include "canvas/Utilities/Exception.h"
#include "fhiclcpp/ParameterSet.h"

namespace wavrec_tool {
  class IWaveformRecog {
  public:
    virtual ~IWaveformRecog() noexcept = default;

    // Calculate multi-class probabilities for waveform
    virtual std::vector<std::vector<float>> predictWaveformType(
      const std::vector<std::vector<float>>&) const = 0;
  };
}

#endif
