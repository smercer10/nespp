#pragma once
#include "cpu.h"
#include <array>
#include <cstdint>

class Bus {
public:
  Bus();
  ~Bus();

  uint8_t read(uint16_t addr, bool bReadOnly = false);
  void write(uint16_t addr, uint8_t data);

  // Devices on bus
  CPU cpu_;
  std::array<uint8_t, 64 * 1024> ram_;
};
