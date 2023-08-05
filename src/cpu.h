#pragma once
#include <cstdint>
#include <string>

class Bus;

// MOS Technology 6502 microprocessor:
// https://www.princeton.edu/~mae412/HANDOUTS/Datasheets/6502.pdf
class CPU {
public:
  CPU();
  ~CPU();

  // Registers
  uint8_t acc_{};     // Accumulator
  uint8_t regX_{};    // Index register X
  uint8_t regY_{};    // Index register Y
  uint8_t stkPtr_{};  // Stack pointer
  uint16_t prgCtr_{}; // Program counter
  uint8_t status_{};  // Status

  enum eStatusFlags {
    C = (1 << 0), // Carry
    Z = (1 << 1), // Zero
    I = (1 << 2), // Interrupt disable
    D = (1 << 3), // Decimal mode
    B = (1 << 4), // Break
    U = (1 << 5), // Unused
    V = (1 << 6), // Overflow
    N = (1 << 7), // Negative
  };

  void connectBus(Bus *bus) { bus_ = bus; }

  void clock();
  void reset();
  void irq();                                 // Interrupt request
  void nmi(uint8_t cycles, uint16_t address); // Non-maskable interrupt

private:
  // Bus interface
  Bus *bus_{nullptr};
  uint8_t read(uint16_t addr);
  void write(uint16_t addr, uint8_t data);

  uint8_t flag(eStatusFlags flag);
  void setFlag(eStatusFlags flag, bool b);

  uint8_t opcode_{};
  uint8_t memory_{};
  uint8_t remCycles_{}; // Remaining instruction cycles
  uint16_t addrAbs_{};  // Location of memory (anywhere)
  uint16_t addrRel_{};  // Destination of branch instruction (+/-16 bytes)
  uint16_t temp_{};     // Convenience variable used everywhere

  struct Instruction {
    std::string name; // For potential disassembly
    uint8_t (CPU::*opcode)(void) = nullptr;
    uint8_t (CPU::*addrMode)(void) = nullptr;
    uint8_t requiredCycles{};
  };

  std::vector<Instruction> opLookup_;

  // Helper functions
  void prgCtrToStk();
  uint8_t fetchMem();
  void writeToAccOrMem(uint16_t data);

  // Addressing mode base functions
  uint16_t absBase(uint8_t reg);
  void zpgBase(uint8_t reg);

  // Opcode base functions
  void branchBase(bool flag);
  void compareBase(uint8_t reg);
  void loadBase(uint8_t reg);

  // Addressing modes - if extra cycle required return 1 else return 0
  uint8_t abs();  // Absolute
  uint8_t absX(); // Absolute, X
  uint8_t absY(); // Absolute, Y
  uint8_t ind();  // Indirect
  uint8_t indX(); // Indirect, X
  uint8_t indY(); // Indirect, Y
  uint8_t imm();  // Immediate
  uint8_t imp();  // Implied
  uint8_t rel();  // Relative
  uint8_t zpg();  // Zero page
  uint8_t zpgX(); // Zero page, X
  uint8_t zpgY(); // Zero page, Y

  // Opcodes - if extra cycle could be required return 1 else return 0
  uint8_t ADC();
  uint8_t AND();
  uint8_t ASL();
  uint8_t BCC();
  uint8_t BCS();
  uint8_t BEQ();
  uint8_t BIT();
  uint8_t BMI();
  uint8_t BNE();
  uint8_t BPL();
  uint8_t BRK();
  uint8_t BVC();
  uint8_t BVS();
  uint8_t CLC();
  uint8_t CLD();
  uint8_t CLI();
  uint8_t CLV();
  uint8_t CMP();
  uint8_t CPX();
  uint8_t CPY();
  uint8_t DEC();
  uint8_t DEX();
  uint8_t DEY();
  uint8_t EOR();
  uint8_t INC();
  uint8_t INX();
  uint8_t INY();
  uint8_t JMP();
  uint8_t JSR();
  uint8_t LDA();
  uint8_t LDX();
  uint8_t LDY();
  uint8_t LSR();
  uint8_t NOP();
  uint8_t ORA();
  uint8_t PHA();
  uint8_t PHP();
  uint8_t PLA();
  uint8_t PLP();
  uint8_t ROL();
  uint8_t ROR();
  uint8_t RTI();
  uint8_t RTS();
  uint8_t SBC();
  uint8_t SEC();
  uint8_t SED();
  uint8_t SEI();
  uint8_t STA();
  uint8_t STX();
  uint8_t STY();
  uint8_t TAX();
  uint8_t TAY();
  uint8_t TSX();
  uint8_t TXA();
  uint8_t TXS();
  uint8_t TYA();

  // Covers all unknown opcodes (same functionality as NOP)
  uint8_t XXX();
};
