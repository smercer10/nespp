#include "cpu.h"
#include "bus.h"

CPU::CPU() {
  opLookup_ = {
      {"BRK", &CPU::BRK, &CPU::imm, 7},  {"ORA", &CPU::ORA, &CPU::indX, 6},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 3},  {"ORA", &CPU::ORA, &CPU::zpg, 3},
      {"ASL", &CPU::ASL, &CPU::zpg, 5},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"PHP", &CPU::PHP, &CPU::imp, 3},  {"ORA", &CPU::ORA, &CPU::imm, 2},
      {"ASL", &CPU::ASL, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 2},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"ORA", &CPU::ORA, &CPU::abs, 4},
      {"ASL", &CPU::ASL, &CPU::abs, 6},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"BPL", &CPU::BPL, &CPU::rel, 2},  {"ORA", &CPU::ORA, &CPU::indY, 5},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"ORA", &CPU::ORA, &CPU::zpgX, 4},
      {"ASL", &CPU::ASL, &CPU::zpgX, 6}, {"???", &CPU::XXX, &CPU::imp, 6},
      {"CLC", &CPU::CLC, &CPU::imp, 2},  {"ORA", &CPU::ORA, &CPU::absY, 4},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 7},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"ORA", &CPU::ORA, &CPU::absX, 4},
      {"ASL", &CPU::ASL, &CPU::absX, 7}, {"???", &CPU::XXX, &CPU::imp, 7},
      {"JSR", &CPU::JSR, &CPU::abs, 6},  {"AND", &CPU::AND, &CPU::indX, 6},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"BIT", &CPU::BIT, &CPU::zpg, 3},  {"AND", &CPU::AND, &CPU::zpg, 3},
      {"ROL", &CPU::ROL, &CPU::zpg, 5},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"PLP", &CPU::PLP, &CPU::imp, 4},  {"AND", &CPU::AND, &CPU::imm, 2},
      {"ROL", &CPU::ROL, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 2},
      {"BIT", &CPU::BIT, &CPU::abs, 4},  {"AND", &CPU::AND, &CPU::abs, 4},
      {"ROL", &CPU::ROL, &CPU::abs, 6},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"BMI", &CPU::BMI, &CPU::rel, 2},  {"AND", &CPU::AND, &CPU::indY, 5},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"AND", &CPU::AND, &CPU::zpgX, 4},
      {"ROL", &CPU::ROL, &CPU::zpgX, 6}, {"???", &CPU::XXX, &CPU::imp, 6},
      {"SEC", &CPU::SEC, &CPU::imp, 2},  {"AND", &CPU::AND, &CPU::absY, 4},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 7},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"AND", &CPU::AND, &CPU::absX, 4},
      {"ROL", &CPU::ROL, &CPU::absX, 7}, {"???", &CPU::XXX, &CPU::imp, 7},
      {"RTI", &CPU::RTI, &CPU::imp, 6},  {"EOR", &CPU::EOR, &CPU::indX, 6},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 3},  {"EOR", &CPU::EOR, &CPU::zpg, 3},
      {"LSR", &CPU::LSR, &CPU::zpg, 5},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"PHA", &CPU::PHA, &CPU::imp, 3},  {"EOR", &CPU::EOR, &CPU::imm, 2},
      {"LSR", &CPU::LSR, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 2},
      {"JMP", &CPU::JMP, &CPU::abs, 3},  {"EOR", &CPU::EOR, &CPU::abs, 4},
      {"LSR", &CPU::LSR, &CPU::abs, 6},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"BVC", &CPU::BVC, &CPU::rel, 2},  {"EOR", &CPU::EOR, &CPU::indY, 5},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"EOR", &CPU::EOR, &CPU::zpgX, 4},
      {"LSR", &CPU::LSR, &CPU::zpgX, 6}, {"???", &CPU::XXX, &CPU::imp, 6},
      {"CLI", &CPU::CLI, &CPU::imp, 2},  {"EOR", &CPU::EOR, &CPU::absY, 4},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 7},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"EOR", &CPU::EOR, &CPU::absX, 4},
      {"LSR", &CPU::LSR, &CPU::absX, 7}, {"???", &CPU::XXX, &CPU::imp, 7},
      {"RTS", &CPU::RTS, &CPU::imp, 6},  {"ADC", &CPU::ADC, &CPU::indX, 6},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 3},  {"ADC", &CPU::ADC, &CPU::zpg, 3},
      {"ROR", &CPU::ROR, &CPU::zpg, 5},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"PLA", &CPU::PLA, &CPU::imp, 4},  {"ADC", &CPU::ADC, &CPU::imm, 2},
      {"ROR", &CPU::ROR, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 2},
      {"JMP", &CPU::JMP, &CPU::ind, 5},  {"ADC", &CPU::ADC, &CPU::abs, 4},
      {"ROR", &CPU::ROR, &CPU::abs, 6},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"BVS", &CPU::BVS, &CPU::rel, 2},  {"ADC", &CPU::ADC, &CPU::indY, 5},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"ADC", &CPU::ADC, &CPU::zpgX, 4},
      {"ROR", &CPU::ROR, &CPU::zpgX, 6}, {"???", &CPU::XXX, &CPU::imp, 6},
      {"SEI", &CPU::SEI, &CPU::imp, 2},  {"ADC", &CPU::ADC, &CPU::absY, 4},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 7},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"ADC", &CPU::ADC, &CPU::absX, 4},
      {"ROR", &CPU::ROR, &CPU::absX, 7}, {"???", &CPU::XXX, &CPU::imp, 7},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"STA", &CPU::STA, &CPU::indX, 6},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"STY", &CPU::STY, &CPU::zpg, 3},  {"STA", &CPU::STA, &CPU::zpg, 3},
      {"STX", &CPU::STX, &CPU::zpg, 3},  {"???", &CPU::XXX, &CPU::imp, 3},
      {"DEY", &CPU::DEY, &CPU::imp, 2},  {"???", &CPU::NOP, &CPU::imp, 2},
      {"TXA", &CPU::TXA, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 2},
      {"STY", &CPU::STY, &CPU::abs, 4},  {"STA", &CPU::STA, &CPU::abs, 4},
      {"STX", &CPU::STX, &CPU::abs, 4},  {"???", &CPU::XXX, &CPU::imp, 4},
      {"BCC", &CPU::BCC, &CPU::rel, 2},  {"STA", &CPU::STA, &CPU::indY, 6},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"STY", &CPU::STY, &CPU::zpgX, 4}, {"STA", &CPU::STA, &CPU::zpgX, 4},
      {"STX", &CPU::STX, &CPU::zpgY, 4}, {"???", &CPU::XXX, &CPU::imp, 4},
      {"TYA", &CPU::TYA, &CPU::imp, 2},  {"STA", &CPU::STA, &CPU::absY, 5},
      {"TXS", &CPU::TXS, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"???", &CPU::NOP, &CPU::imp, 5},  {"STA", &CPU::STA, &CPU::absX, 5},
      {"???", &CPU::XXX, &CPU::imp, 5},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"LDY", &CPU::LDY, &CPU::imm, 2},  {"LDA", &CPU::LDA, &CPU::indX, 6},
      {"LDX", &CPU::LDX, &CPU::imm, 2},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"LDY", &CPU::LDY, &CPU::zpg, 3},  {"LDA", &CPU::LDA, &CPU::zpg, 3},
      {"LDX", &CPU::LDX, &CPU::zpg, 3},  {"???", &CPU::XXX, &CPU::imp, 3},
      {"TAY", &CPU::TAY, &CPU::imp, 2},  {"LDA", &CPU::LDA, &CPU::imm, 2},
      {"TAX", &CPU::TAX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 2},
      {"LDY", &CPU::LDY, &CPU::abs, 4},  {"LDA", &CPU::LDA, &CPU::abs, 4},
      {"LDX", &CPU::LDX, &CPU::abs, 4},  {"???", &CPU::XXX, &CPU::imp, 4},
      {"BCS", &CPU::BCS, &CPU::rel, 2},  {"LDA", &CPU::LDA, &CPU::indY, 5},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"LDY", &CPU::LDY, &CPU::zpgX, 4}, {"LDA", &CPU::LDA, &CPU::zpgX, 4},
      {"LDX", &CPU::LDX, &CPU::zpgY, 4}, {"???", &CPU::XXX, &CPU::imp, 4},
      {"CLV", &CPU::CLV, &CPU::imp, 2},  {"LDA", &CPU::LDA, &CPU::absY, 4},
      {"TSX", &CPU::TSX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 4},
      {"LDY", &CPU::LDY, &CPU::absX, 4}, {"LDA", &CPU::LDA, &CPU::absX, 4},
      {"LDX", &CPU::LDX, &CPU::absY, 4}, {"???", &CPU::XXX, &CPU::imp, 4},
      {"CPY", &CPU::CPY, &CPU::imm, 2},  {"CMP", &CPU::CMP, &CPU::indX, 6},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"CPY", &CPU::CPY, &CPU::zpg, 3},  {"CMP", &CPU::CMP, &CPU::zpg, 3},
      {"DEC", &CPU::DEC, &CPU::zpg, 5},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"INY", &CPU::INY, &CPU::imp, 2},  {"CMP", &CPU::CMP, &CPU::imm, 2},
      {"DEX", &CPU::DEX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 2},
      {"CPY", &CPU::CPY, &CPU::abs, 4},  {"CMP", &CPU::CMP, &CPU::abs, 4},
      {"DEC", &CPU::DEC, &CPU::abs, 6},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"BNE", &CPU::BNE, &CPU::rel, 2},  {"CMP", &CPU::CMP, &CPU::indY, 5},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"CMP", &CPU::CMP, &CPU::zpgX, 4},
      {"DEC", &CPU::DEC, &CPU::zpgX, 6}, {"???", &CPU::XXX, &CPU::imp, 6},
      {"CLD", &CPU::CLD, &CPU::imp, 2},  {"CMP", &CPU::CMP, &CPU::absY, 4},
      {"NOP", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 7},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"CMP", &CPU::CMP, &CPU::absX, 4},
      {"DEC", &CPU::DEC, &CPU::absX, 7}, {"???", &CPU::XXX, &CPU::imp, 7},
      {"CPX", &CPU::CPX, &CPU::imm, 2},  {"SBC", &CPU::SBC, &CPU::indX, 6},
      {"???", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"CPX", &CPU::CPX, &CPU::zpg, 3},  {"SBC", &CPU::SBC, &CPU::zpg, 3},
      {"INC", &CPU::INC, &CPU::zpg, 5},  {"???", &CPU::XXX, &CPU::imp, 5},
      {"INX", &CPU::INX, &CPU::imp, 2},  {"SBC", &CPU::SBC, &CPU::imm, 2},
      {"NOP", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::SBC, &CPU::imp, 2},
      {"CPX", &CPU::CPX, &CPU::abs, 4},  {"SBC", &CPU::SBC, &CPU::abs, 4},
      {"INC", &CPU::INC, &CPU::abs, 6},  {"???", &CPU::XXX, &CPU::imp, 6},
      {"BEQ", &CPU::BEQ, &CPU::rel, 2},  {"SBC", &CPU::SBC, &CPU::indY, 5},
      {"???", &CPU::XXX, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 8},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"SBC", &CPU::SBC, &CPU::zpgX, 4},
      {"INC", &CPU::INC, &CPU::zpgX, 6}, {"???", &CPU::XXX, &CPU::imp, 6},
      {"SED", &CPU::SED, &CPU::imp, 2},  {"SBC", &CPU::SBC, &CPU::absY, 4},
      {"NOP", &CPU::NOP, &CPU::imp, 2},  {"???", &CPU::XXX, &CPU::imp, 7},
      {"???", &CPU::NOP, &CPU::imp, 4},  {"SBC", &CPU::SBC, &CPU::absX, 4},
      {"INC", &CPU::INC, &CPU::absX, 7}, {"???", &CPU::XXX, &CPU::imp, 7},
  };
}

CPU::~CPU() {}

uint8_t CPU::read(uint16_t addr) { return bus_->read(addr, false); }

void CPU::write(uint16_t addr, uint8_t data) { bus_->write(addr, data); }

uint8_t CPU::flag(eStatusFlags flag) { return ((status_ & flag) > 0) ? 1 : 0; }

void CPU::setFlag(eStatusFlags flag, bool b) {
  if (b)
    status_ |= flag;
  else
    status_ &= ~flag;
}

void CPU::clock() {
  if (!remCycles_) {
    opcode_ = read(prgCtr_);

    setFlag(U, true);

    prgCtr_++;

    remCycles_ = opLookup_[opcode_].requiredCycles;

    // Check if extra cycle is required
    uint8_t extraCycle1{(this->*opLookup_[opcode_].addrMode)()};
    uint8_t extraCycle2{(this->*opLookup_[opcode_].opcode)()};
    remCycles_ += (extraCycle1 & extraCycle1);

    setFlag(U, true);
  }
  remCycles_--;
}

void CPU::reset() {

  // Read and set program counter from fixed address
  addrAbs_ = 0xFFFC;
  uint16_t lo{read(addrAbs_)};
  uint16_t hi{read(addrAbs_ + 1)};
  prgCtr_ = (hi << 8) | lo;

  acc_ = 0x00;
  regX_ = 0x00;
  regY_ = 0x00;
  stkPtr_ = 0xFD;
  status_ = 0x00 | U;
  addrRel_ = 0x0000;
  addrAbs_ = 0x0000;
  memory_ = 0x00;
  remCycles_ = 8;
}

// Push program counter to stack
void CPU::prgCtrToStk() {
  write(0x0100 + stkPtr_, (prgCtr_ >> 8) & 0x00FF);
  stkPtr_--;
  write(0x0100 + stkPtr_, prgCtr_ & 0x00FF);
  stkPtr_--;
}

void CPU::irq() {
  if (!flag(I))
    nmi(7, 0xFFFE);
}

void CPU::nmi(uint8_t cycles = 8, uint16_t address = 0xFFFA) {
  prgCtrToStk();

  // Push status register to stack
  setFlag(B, false);
  setFlag(U, true);
  setFlag(I, true);
  write(0x0100 + stkPtr_, status_);
  stkPtr_--;

  // Read new program counter from fixed address
  addrAbs_ = address;
  uint16_t lo{read(addrAbs_)};
  uint16_t hi{read(addrAbs_ + 1)};
  prgCtr_ = (hi << 8) | lo;

  remCycles_ = cycles;
}

uint8_t CPU::isPageChanged(uint16_t hi) {
  if ((addrAbs_ & 0xFF00) != (hi << 8))
    return 1;
  else
    return 0;
}

// Base function for absolute addressing modes
uint16_t CPU::absBase(uint8_t reg = 0) {
  uint16_t lo{read(prgCtr_)};
  prgCtr_++;
  uint16_t hi{read(prgCtr_)};
  prgCtr_++;

  addrAbs_ = (hi << 8) | lo;
  addrAbs_ += reg;

  return hi;
}

// Base function for zero page addressing modes
void CPU::zpgBase(uint8_t reg = 0) {
  addrAbs_ = read(prgCtr_) + reg;
  prgCtr_++;
  addrAbs_ &= 0x00FF;
}

// Read from supplied 16-bit address
uint8_t CPU::abs() {
  absBase();

  return 0;
}

// Read from (supplied 16-bit address + register X)
uint8_t CPU::absX() {
  uint16_t hi{absBase(regX_)};

  // If resulting address changes the page an extra cycle is required
  return isPageChanged(hi);
}

// Read from (supplied 16-bit address + register Y)
uint8_t CPU::absY() {
  uint16_t hi{absBase(regY_)};

  // If resulting address changes the page an extra cycle is required
  return isPageChanged(hi);
}

// Read from address pointed to by supplied 16-bit address
uint8_t CPU::ind() {
  uint16_t ptrLo{read(prgCtr_)};
  prgCtr_++;
  uint16_t ptrHi{read(prgCtr_)};
  prgCtr_++;

  uint16_t ptr = (ptrHi << 8) | ptrLo;

  // Resolve page boundary hardware bug
  if (ptrLo == 0x00FF)
    addrAbs_ = (read(ptr & 0xFF00) << 8) | read(ptr);
  else
    addrAbs_ = (read(ptr + 1) << 8) | read(ptr);

  return 0;
}

// Read from address pointed to by ((supplied 8-bit address + register
// X) in zero page)
uint8_t CPU::indX() {
  uint16_t ptr{read(prgCtr_)};
  prgCtr_++;

  uint16_t lo{read((ptr + regX_) & 0x00FF)};
  uint16_t hi{read((ptr + regX_ + 1) & 0x00FF)};

  addrAbs_ = (hi << 8) | lo;

  return 0;
}

// Read from address pointed to by ((supplied 8-bit address in zero
// page) + register Y)
uint8_t CPU::indY() {
  uint16_t ptr{read(prgCtr_)};
  prgCtr_++;

  uint16_t lo{read(ptr & 0x00FF)};
  uint16_t hi{read((ptr + 1) & 0x00FF)};

  addrAbs_ = (hi << 8) | lo;
  addrAbs_ += regY_;

  // If resulting address changes the page an extra cycle is required
  return isPageChanged(hi);
}

// Read from next byte
uint8_t CPU::imm() {
  addrAbs_ = prgCtr_++;

  return 0;
}

// No extra data required (but target accumulator anyway)
uint8_t CPU::imp() {
  memory_ = acc_;

  return 0;
}

// Get branch destination from supplied 8-bit offset
uint8_t CPU::rel() {
  addrRel_ = read(prgCtr_);
  prgCtr_++;

  // Sign extension for negative offset
  if (addrRel_ & 0x0080) {
    addrRel_ |= 0xFF00;
  }

  return 0;
}

// Read from supplied 8-bit address in zero page
uint8_t CPU::zpg() {
  zpgBase();

  return 0;
}

// Read from (supplied 8-bit address + register X) in zero page
uint8_t CPU::zpgX() {
  zpgBase(regX_);

  return 0;
}

// Read from (supplied 8-bit address + register Y) in zero page
uint8_t CPU::zpgY() {
  zpgBase(regY_);

  return 0;
}

uint8_t CPU::fetchMem() {
  if (!(opLookup_[opcode_].addrMode == &CPU::imp))
    memory_ = read(addrAbs_);

  return memory_;
}

void CPU::writeToAccOrMem(uint16_t data) {
  if (opLookup_[opcode_].addrMode == &CPU::imp)
    acc_ = data & 0x00FF;
  else
    write(addrAbs_, data & 0x00FF);
}

// Base function for branch operations
void CPU::branchBase(bool flag) {
  if (flag) {
    remCycles_++;

    addrAbs_ = prgCtr_ + addrRel_;

    // If the branch crosses a page an extra cycle is required
    if ((addrAbs_ & 0xFF00) != (prgCtr_ & 0xFF00))
      remCycles_++;

    prgCtr_ = addrAbs_;
  }
}

// Base function for compare operations
void CPU::compareBase(uint8_t reg) {
  fetchMem();

  temp_ = (uint16_t)reg - (uint16_t)memory_;

  setFlag(C, reg >= memory_);
  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, temp_ & 0x0080);
}

// Base function for load operations
void CPU::loadBase(uint8_t reg) {
  fetchMem();

  reg = memory_;

  setFlag(Z, !reg);
  setFlag(N, reg & 0x80);
}

// Add memory to accumulator with carry
uint8_t CPU::ADC() {
  fetchMem();

  // Add in 16-bit domain to capture carry bit
  temp_ = (uint16_t)acc_ + (uint16_t)memory_ + (uint16_t)flag(C);

  setFlag(C, temp_ & 0xFF00);
  setFlag(Z, !(temp_ & 0x00FF));

  // Set overflow flag if:
  //   - positive accumulator + positive memory = negative result
  //   - negative accumulator + negative memory = positive result
  setFlag(V,
          (~((uint16_t)acc_ ^ (uint16_t)memory_) & ((uint16_t)acc_ ^ temp_)) &
              0x0080);

  setFlag(N, temp_ & 0x0080);

  acc_ = temp_ & 0x00FF;

  return 1;
}

// AND memory with accumulator
uint8_t CPU::AND() {
  fetchMem();

  acc_ &= memory_;

  setFlag(Z, !acc_);
  setFlag(N, acc_ & 0x80);

  return 1;
}

// Shift accumulator or memory one bit left
uint8_t CPU::ASL() {
  fetchMem();

  temp_ = (uint16_t)memory_ << 1;

  setFlag(C, temp_ & 0xFF00);
  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, temp_ & 0x0080);

  writeToAccOrMem(temp_);

  return 0;
}

// Branch on carry clear
uint8_t CPU::BCC() {
  branchBase(!flag(C));

  return 0;
}

// Branch on carry set
uint8_t CPU::BCS() {
  branchBase(flag(C));

  return 0;
}

// Branch on zero result
uint8_t CPU::BEQ() {
  branchBase(flag(Z));

  return 0;
}

// Test memory bits with accumulator
uint8_t CPU::BIT() {
  fetchMem();

  temp_ = acc_ & memory_;

  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, memory_ & 0x80);
  setFlag(V, memory_ & 0x40);

  return 0;
}

// Branch on negative result
uint8_t CPU::BMI() {
  branchBase(flag(N));

  return 0;
}

// Branch on non-zero result
uint8_t CPU::BNE() {
  branchBase(!flag(Z));

  return 0;
}

// Branch on positive result
uint8_t CPU::BPL() {
  branchBase(!flag(N));

  return 0;
}

// Force break
uint8_t CPU::BRK() {
  prgCtr_++;

  setFlag(I, true);

  prgCtrToStk();

  setFlag(B, true);

  write(0x0100 + stkPtr_, status_);
  stkPtr_--;

  setFlag(B, false);

  prgCtr_ = (uint16_t)read(0xFFFE) | ((uint16_t)read(0xFFFF) << 8);

  return 0;
}

// Branch on overflow clear
uint8_t CPU::BVC() {
  branchBase(!flag(V));

  return 0;
}

// Branch on overflow set
uint8_t CPU::BVS() {
  branchBase(flag(V));

  return 0;
}

// Clear carry flag
uint8_t CPU::CLC() {
  setFlag(C, false);

  return 0;
}

// Clear decimal mode flag
uint8_t CPU::CLD() {
  setFlag(D, false);

  return 0;
}

// Clear interrupt disable flag
uint8_t CPU::CLI() {
  setFlag(I, false);

  return 0;
}

// Clear overflow flag
uint8_t CPU::CLV() {
  setFlag(V, false);

  return 0;
}

// Compare memory and accumulator
uint8_t CPU::CMP() {
  compareBase(acc_);

  return 1;
}

// Compare memory and register X
uint8_t CPU::CPX() {
  compareBase(regX_);

  return 1;
}

// Compare memory and register Y
uint8_t CPU::CPY() {
  compareBase(regY_);

  return 1;
}

// Decrement memory
uint8_t CPU::DEC() {
  fetchMem();

  temp_ = memory_ - 1;
  write(addrAbs_, temp_ & 0x00FF);

  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, temp_ & 0x0080);

  return 0;
}

// Decrement X register
uint8_t CPU::DEX() {
  regX_--;

  setFlag(Z, !regX_);
  setFlag(N, regX_ & 0x80);

  return 0;
}

// Decrement Y register
uint8_t CPU::DEY() {
  regY_--;

  setFlag(Z, !regY_);
  setFlag(N, regY_ & 0x80);

  return 0;
}

// XOR memory with accumulator
uint8_t CPU::EOR() {
  fetchMem();

  acc_ ^= memory_;

  setFlag(Z, !acc_);
  setFlag(N, acc_ & 0x80);

  return 1;
}

// Increment memory
uint8_t CPU::INC() {
  fetchMem();

  temp_ = memory_ + 1;
  write(addrAbs_, temp_ & 0x00FF);

  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, temp_ & 0x80);

  return 0;
}

// Increment X register
uint8_t CPU::INX() {
  regX_++;

  setFlag(Z, !regX_);
  setFlag(N, regX_ & 0x80);

  return 0;
}

// Increment Y register
uint8_t CPU::INY() {
  regY_++;

  setFlag(Z, !regY_);
  setFlag(N, regY_ & 0x80);

  return 0;
}

// Jump to location
uint8_t CPU::JMP() {
  prgCtr_ = addrAbs_;

  return 0;
}

// Jump to location and save return address
uint8_t CPU::JSR() {
  prgCtr_--;

  prgCtrToStk();

  prgCtr_ = addrAbs_;

  return 0;
}

// Load memory into accumulator
uint8_t CPU::LDA() {
  loadBase(acc_);

  return 1;
}

// Load memory into register X
uint8_t CPU::LDX() {
  loadBase(regX_);

  return 1;
}

// Load memory into register Y
uint8_t CPU::LDY() {
  loadBase(regY_);

  return 1;
}

// Shift accumulator or memory one bit right
uint8_t CPU::LSR() {
  fetchMem();

  setFlag(C, memory_ & 0x0001);

  temp_ = (uint16_t)memory_ >> 1;

  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, temp_ & 0x0080);

  writeToAccOrMem(temp_);

  return 0;
}

// Known illegal opcodes:
// https://www.nesdev.org/wiki/CPU_unofficial_opcodes
uint8_t CPU::NOP() {
  switch (opcode_) {
  case 0x1C:
  case 0x3C:
  case 0x5C:
  case 0x7C:
  case 0xDC:
  case 0xFC:
    return 1;
    break;
  }
  return 0;
}

// OR memory with accumulator
uint8_t CPU::ORA() {
  fetchMem();

  acc_ |= memory_;

  setFlag(Z, !acc_);
  setFlag(N, acc_ & 0x80);

  return 1;
}

// Push accumulator to stack
uint8_t CPU::PHA() {
  write(0x0100 + stkPtr_, acc_);
  stkPtr_--;

  return 0;
}

// Push status register to stack (with break and unused flags set)
uint8_t CPU::PHP() {
  write(0x0100 + stkPtr_, status_ | B | U);
  stkPtr_--;

  setFlag(B, false);
  setFlag(U, false);

  return 0;
}

// Pop accumulator off stack
uint8_t CPU::PLA() {
  stkPtr_++;
  acc_ = read(0x0100 + stkPtr_);

  setFlag(Z, !acc_);
  setFlag(N, acc_ & 0x80);

  return 0;
}

// Pop status register off stack
uint8_t CPU::PLP() {
  stkPtr_++;
  status_ = read(0x0100 + stkPtr_);

  setFlag(U, false);

  return 0;
}

// Rotate accumulator or memory one bit left
uint8_t CPU::ROL() {
  fetchMem();

  temp_ = (uint16_t)(memory_ << 1) | flag(C);

  setFlag(C, temp_ & 0xFF00);
  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, temp_ & 0x0080);

  writeToAccOrMem(temp_);

  return 0;
}

// Rotate accumulator or memory one bit right
uint8_t CPU::ROR() {
  fetchMem();

  temp_ = (uint16_t)(flag(C) << 7) | (memory_ >> 1);

  setFlag(C, memory_ & 0x01);
  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(N, temp_ & 0x0080);

  writeToAccOrMem(temp_);

  return 0;
}

// Return from interrupt
uint8_t CPU::RTI() {
  stkPtr_++;
  status_ = read(0x0100 + stkPtr_);
  status_ &= ~B;
  status_ &= ~U;

  stkPtr_++;
  prgCtr_ = (uint16_t)read(0x0100 + stkPtr_);
  stkPtr_++;
  prgCtr_ |= (uint16_t)read(0x0100 + stkPtr_) << 8;

  return 0;
}

// Return from subroutine
uint8_t CPU::RTS() {
  stkPtr_++;
  prgCtr_ = (uint16_t)read(0x0100 + stkPtr_);
  stkPtr_++;
  prgCtr_ |= (uint16_t)read(0x0100 + stkPtr_) << 8;

  return 0;
}

// Subtract memory from accumulator with borrow
uint8_t CPU::SBC() {
  fetchMem();

  uint16_t invMem = (uint16_t)memory_ ^ 0x00FF;

  // Same as ADC with inverted memory
  temp_ = (uint16_t)acc_ + invMem + (uint16_t)flag(C);
  setFlag(C, temp_ & 0xFF00);
  setFlag(Z, !(temp_ & 0x00FF));
  setFlag(V, ((uint16_t)acc_ ^ temp_) & (temp_ ^ invMem) & 0x0080);
  setFlag(N, temp_ & 0x0080);
  acc_ = temp_ & 0x00FF;

  return 1;
}

// Set carry flag
uint8_t CPU::SEC() {
  setFlag(C, true);

  return 0;
}

// Set decimal mode flag
uint8_t CPU::SED() {
  setFlag(D, true);

  return 0;
}

// Set interrupt disable flag
uint8_t CPU::SEI() {
  setFlag(I, true);

  return 0;
}

// Store accumulator in memory
uint8_t CPU::STA() {
  write(addrAbs_, acc_);

  return 0;
}

// Store register X in memory
uint8_t CPU::STX() {
  write(addrAbs_, regX_);

  return 0;
}

// Store register Y in memory
uint8_t CPU::STY() {
  write(addrAbs_, regY_);

  return 0;
}

// Transfer accumulator to register X
uint8_t CPU::TAX() {
  regX_ = acc_;

  setFlag(Z, !regX_);
  setFlag(N, regX_ & 0x80);

  return 0;
}

// Transfer accumulator to register Y
uint8_t CPU::TAY() {
  regY_ = acc_;

  setFlag(Z, !regY_);
  setFlag(N, regY_ & 0x80);

  return 0;
}

// Transfer stack pointer to register X
uint8_t CPU::TSX() {
  regX_ = stkPtr_;

  setFlag(Z, !regX_);
  setFlag(N, regX_ & 0x80);

  return 0;
}

// Transfer register X to accumulator
uint8_t CPU::TXA() {
  acc_ = regX_;

  setFlag(Z, !acc_);
  setFlag(N, acc_ & 0x80);

  return 0;
}

// Transfer register X to stack pointer
uint8_t CPU::TXS() {
  stkPtr_ = regX_;

  return 0;
}

// Transfer register Y to accumulator
uint8_t CPU::TYA() {
  acc_ = regY_;

  setFlag(Z, !acc_);
  setFlag(N, acc_ & 0x80);

  return 0;
}

// Unknown illegal opcodes
uint8_t CPU::XXX() { return 0; }
