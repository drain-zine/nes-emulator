use phf::phf_map;

/* https://www.nesdev.org/obelisk-6502-guide/reference.html#LDX */
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mnemonic {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,
}

#[derive(Debug, Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect_X,
    Indirect_Y,
    Relative,
    Indirect,
    Accumulator,
    Implied,
}

#[derive(Debug, Clone)]
pub struct Instruction {
    pub mnemonic: Mnemonic,
    pub opcode: u8,
    pub bytes: u8,
    pub cycles: u8,
    pub addressing_mode: AddressingMode,
}

macro_rules! instruction {
    ($mnemonic:ident, $opcode:expr, $bytes:expr, $cycles:expr, $mode:ident) => {
        Instruction {
            mnemonic: Mnemonic::$mnemonic,
            opcode: $opcode,
            bytes: $bytes,
            cycles: $cycles,
            addressing_mode: AddressingMode::$mode,
        }
    };
}

pub static INSTRUCTION_TABLE: phf::Map<u8, Instruction> = phf_map! {
    // ADC – Add with Carry
    0x69u8 => instruction!(ADC, 0x69, 2, 2, Immediate),
    0x65u8 => instruction!(ADC, 0x65, 2, 3, ZeroPage),
    0x75u8 => instruction!(ADC, 0x75, 2, 4, ZeroPage_X),
    0x6Du8 => instruction!(ADC, 0x6D, 3, 4, Absolute),
    0x7Du8 => instruction!(ADC, 0x7D, 3, 4 /* +1 page */, Absolute_X),
    0x79u8 => instruction!(ADC, 0x79, 3, 4 /* +1 page */, Absolute_Y),
    0x61u8 => instruction!(ADC, 0x61, 2, 6, Indirect_X),
    0x71u8 => instruction!(ADC, 0x71, 2, 5 /* +1 page */, Indirect_Y),

    // AND – Logical AND
    0x29u8 => instruction!(AND, 0x29, 2, 2, Immediate),
    0x25u8 => instruction!(AND, 0x25, 2, 3, ZeroPage),
    0x35u8 => instruction!(AND, 0x35, 2, 4, ZeroPage_X),
    0x2Du8 => instruction!(AND, 0x2D, 3, 4, Absolute),
    0x3Du8 => instruction!(AND, 0x3D, 3, 4 /* +1 page */, Absolute_X),
    0x39u8 => instruction!(AND, 0x39, 3, 4 /* +1 page */, Absolute_Y),
    0x21u8 => instruction!(AND, 0x21, 2, 6, Indirect_X),
    0x31u8 => instruction!(AND, 0x31, 2, 5 /* +1 page */, Indirect_Y),

    // ASL – Arithmetic Shift Left
    0x0Au8 => instruction!(ASL, 0x0A, 1, 2, Accumulator),
    0x06u8 => instruction!(ASL, 0x06, 2, 5, ZeroPage),
    0x16u8 => instruction!(ASL, 0x16, 2, 6, ZeroPage_X),
    0x0Eu8 => instruction!(ASL, 0x0E, 3, 6, Absolute),
    0x1Eu8 => instruction!(ASL, 0x1E, 3, 7, Absolute_X),

    // BCC – Branch if Carry Clear
    0x90u8 => instruction!(BCC, 0x90, 2, 2 /* +1,+2 */, Relative),

    // BCS – Branch if Carry Set
    0xB0u8 => instruction!(BCS, 0xB0, 2, 2 /* +1,+2 */, Relative),

    // BEQ – Branch if Equal (Z=1)
    0xF0u8 => instruction!(BEQ, 0xF0, 2, 2 /* +1,+2 */, Relative),

    // BIT – Bit Test
    0x24u8 => instruction!(BIT, 0x24, 2, 3, ZeroPage),
    0x2Cu8 => instruction!(BIT, 0x2C, 3, 4, Absolute),

    // BMI – Branch if Minus (N=1)
    0x30u8 => instruction!(BMI, 0x30, 2, 2 /* +1,+2 */, Relative),

    // BNE – Branch if Not Equal (Z=0)
    0xD0u8 => instruction!(BNE, 0xD0, 2, 2 /* +1,+2 */, Relative),

    // BPL – Branch if Positive (N=0)
    0x10u8 => instruction!(BPL, 0x10, 2, 2 /* +1,+2 */, Relative),

    // BRK – Force Interrupt
    0x00u8 => instruction!(BRK, 0x00, 1, 7, Implied),

    // BVC – Branch if Overflow Clear
    0x50u8 => instruction!(BVC, 0x50, 2, 2 /* +1,+2 */, Relative),

    // BVS – Branch if Overflow Set
    0x70u8 => instruction!(BVS, 0x70, 2, 2 /* +1,+2 */, Relative),

    // CLC – Clear Carry Flag
    0x18u8 => instruction!(CLC, 0x18, 1, 2, Implied),

    // CLD – Clear Decimal Flag
    0xD8u8 => instruction!(CLD, 0xD8, 1, 2, Implied),

    // CLI – Clear Interrupt Disable
    0x58u8 => instruction!(CLI, 0x58, 1, 2, Implied),

    // CLV – Clear Overflow Flag
    0xB8u8 => instruction!(CLV, 0xB8, 1, 2, Implied),

    // CMP – Compare A
    0xC9u8 => instruction!(CMP, 0xC9, 2, 2, Immediate),
    0xC5u8 => instruction!(CMP, 0xC5, 2, 3, ZeroPage),
    0xD5u8 => instruction!(CMP, 0xD5, 2, 4, ZeroPage_X),
    0xCDu8 => instruction!(CMP, 0xCD, 3, 4, Absolute),
    0xDDu8 => instruction!(CMP, 0xDD, 3, 4 /* +1 page */, Absolute_X),
    0xD9u8 => instruction!(CMP, 0xD9, 3, 4 /* +1 page */, Absolute_Y),
    0xC1u8 => instruction!(CMP, 0xC1, 2, 6, Indirect_X),
    0xD1u8 => instruction!(CMP, 0xD1, 2, 5 /* +1 page */, Indirect_Y),

    // CPX – Compare X
    0xE0u8 => instruction!(CPX, 0xE0, 2, 2, Immediate),
    0xE4u8 => instruction!(CPX, 0xE4, 2, 3, ZeroPage),
    0xECu8 => instruction!(CPX, 0xEC, 3, 4, Absolute),

    // CPY – Compare Y
    0xC0u8 => instruction!(CPY, 0xC0, 2, 2, Immediate),
    0xC4u8 => instruction!(CPY, 0xC4, 2, 3, ZeroPage),
    0xCCu8 => instruction!(CPY, 0xCC, 3, 4, Absolute),

    // DEC – Decrement Memory
    0xC6u8 => instruction!(DEC, 0xC6, 2, 5, ZeroPage),
    0xD6u8 => instruction!(DEC, 0xD6, 2, 6, ZeroPage_X),
    0xCEu8 => instruction!(DEC, 0xCE, 3, 6, Absolute),
    0xDEu8 => instruction!(DEC, 0xDE, 3, 7, Absolute_X),

    // DEX – Decrement X
    0xCAu8 => instruction!(DEX, 0xCA, 1, 2, Implied),

    // DEY – Decrement Y
    0x88u8 => instruction!(DEY, 0x88, 1, 2, Implied),

    // EOR – Exclusive OR
    0x49u8 => instruction!(EOR, 0x49, 2, 2, Immediate),
    0x45u8 => instruction!(EOR, 0x45, 2, 3, ZeroPage),
    0x55u8 => instruction!(EOR, 0x55, 2, 4, ZeroPage_X),
    0x4Du8 => instruction!(EOR, 0x4D, 3, 4, Absolute),
    0x5Du8 => instruction!(EOR, 0x5D, 3, 4 /* +1 page */, Absolute_X),
    0x59u8 => instruction!(EOR, 0x59, 3, 4 /* +1 page */, Absolute_Y),
    0x41u8 => instruction!(EOR, 0x41, 2, 6, Indirect_X),
    0x51u8 => instruction!(EOR, 0x51, 2, 5 /* +1 page */, Indirect_Y),

    // INC – Increment Memory
    0xE6u8 => instruction!(INC, 0xE6, 2, 5, ZeroPage),
    0xF6u8 => instruction!(INC, 0xF6, 2, 6, ZeroPage_X),
    0xEEu8 => instruction!(INC, 0xEE, 3, 6, Absolute),
    0xFEu8 => instruction!(INC, 0xFE, 3, 7, Absolute_X),

    // INX – Increment X
    0xE8u8 => instruction!(INX, 0xE8, 1, 2, Implied),
    // INY – Increment Y
    0xC8u8 => instruction!(INY, 0xC8, 1, 2, Implied),

    // JMP – Jump
    0x4Cu8 => instruction!(JMP, 0x4C, 3, 3, Absolute),
    0x6Cu8 => instruction!(JMP, 0x6C, 3, 5, Indirect),

    // JSR – Jump to Subroutine
    0x20u8 => instruction!(JSR, 0x20, 3, 6, Absolute),

    // LDA – Load Accumulator
    0xA9u8 => instruction!(LDA, 0xA9, 2, 2, Immediate),
    0xA5u8 => instruction!(LDA, 0xA5, 2, 3, ZeroPage),
    0xB5u8 => instruction!(LDA, 0xB5, 2, 4, ZeroPage_X),
    0xADu8 => instruction!(LDA, 0xAD, 3, 4, Absolute),
    0xBDu8 => instruction!(LDA, 0xBD, 3, 4 /* +1 page */, Absolute_X),
    0xB9u8 => instruction!(LDA, 0xB9, 3, 4 /* +1 page */, Absolute_Y),
    0xA1u8 => instruction!(LDA, 0xA1, 2, 6, Indirect_X),
    0xB1u8 => instruction!(LDA, 0xB1, 2, 5 /* +1 page */, Indirect_Y),

    // LDX – Load X Register
    0xA2u8 => instruction!(LDX, 0xA2, 2, 2, Immediate),
    0xA6u8 => instruction!(LDX, 0xA6, 2, 3, ZeroPage),
    0xB6u8 => instruction!(LDX, 0xB6, 2, 4, ZeroPage_Y),
    0xAEu8 => instruction!(LDX, 0xAE, 3, 4, Absolute),
    0xBEu8 => instruction!(LDX, 0xBE, 3, 4 /* +1 page */, Absolute_Y),

    // LDY – Load Y Register
    0xA0u8 => instruction!(LDY, 0xA0, 2, 2, Immediate),
    0xA4u8 => instruction!(LDY, 0xA4, 2, 3, ZeroPage),
    0xB4u8 => instruction!(LDY, 0xB4, 2, 4, ZeroPage_X),
    0xACu8 => instruction!(LDY, 0xAC, 3, 4, Absolute),
    0xBCu8 => instruction!(LDY, 0xBC, 3, 4 /* +1 page */, Absolute_X),

    // LSR – Logical Shift Right
    0x4Au8 => instruction!(LSR, 0x4A, 1, 2, Accumulator),
    0x46u8 => instruction!(LSR, 0x46, 2, 5, ZeroPage),
    0x56u8 => instruction!(LSR, 0x56, 2, 6, ZeroPage_X),
    0x4Eu8 => instruction!(LSR, 0x4E, 3, 6, Absolute),
    0x5Eu8 => instruction!(LSR, 0x5E, 3, 7, Absolute_X),

    // NOP – No Operation
    0xEAu8 => instruction!(NOP, 0xEA, 1, 2, Implied),

    // ORA – Inclusive OR
    0x09u8 => instruction!(ORA, 0x09, 2, 2, Immediate),
    0x05u8 => instruction!(ORA, 0x05, 2, 3, ZeroPage),
    0x15u8 => instruction!(ORA, 0x15, 2, 4, ZeroPage_X),
    0x0Du8 => instruction!(ORA, 0x0D, 3, 4, Absolute),
    0x1Du8 => instruction!(ORA, 0x1D, 3, 4 /* +1 page */, Absolute_X),
    0x19u8 => instruction!(ORA, 0x19, 3, 4 /* +1 page */, Absolute_Y),
    0x01u8 => instruction!(ORA, 0x01, 2, 6, Indirect_X),
    0x11u8 => instruction!(ORA, 0x11, 2, 5 /* +1 page */, Indirect_Y),

    // PHA – Push Accumulator
    0x48u8 => instruction!(PHA, 0x48, 1, 3, Implied),

    // PHP – Push Status
    0x08u8 => instruction!(PHP, 0x08, 1, 3, Implied),

    // PLA – Pull Accumulator
    0x68u8 => instruction!(PLA, 0x68, 1, 4, Implied),

    // PLP – Pull Status
    0x28u8 => instruction!(PLP, 0x28, 1, 4, Implied),

    // ROL – Rotate Left
    0x2Au8 => instruction!(ROL, 0x2A, 1, 2, Accumulator),
    0x26u8 => instruction!(ROL, 0x26, 2, 5, ZeroPage),
    0x36u8 => instruction!(ROL, 0x36, 2, 6, ZeroPage_X),
    0x2Eu8 => instruction!(ROL, 0x2E, 3, 6, Absolute),
    0x3Eu8 => instruction!(ROL, 0x3E, 3, 7, Absolute_X),

    // ROR – Rotate Right
    0x6Au8 => instruction!(ROR, 0x6A, 1, 2, Accumulator),
    0x66u8 => instruction!(ROR, 0x66, 2, 5, ZeroPage),
    0x76u8 => instruction!(ROR, 0x76, 2, 6, ZeroPage_X),
    0x6Eu8 => instruction!(ROR, 0x6E, 3, 6, Absolute),
    0x7Eu8 => instruction!(ROR, 0x7E, 3, 7, Absolute_X),

    // RTI – Return from Interrupt
    0x40u8 => instruction!(RTI, 0x40, 1, 6, Implied),

    // RTS – Return from Subroutine
    0x60u8 => instruction!(RTS, 0x60, 1, 6, Implied),

    // SBC – Subtract with Carry
    0xE9u8 => instruction!(SBC, 0xE9, 2, 2, Immediate),
    0xE5u8 => instruction!(SBC, 0xE5, 2, 3, ZeroPage),
    0xF5u8 => instruction!(SBC, 0xF5, 2, 4, ZeroPage_X),
    0xEDu8 => instruction!(SBC, 0xED, 3, 4, Absolute),
    0xFDu8 => instruction!(SBC, 0xFD, 3, 4 /* +1 page */, Absolute_X),
    0xF9u8 => instruction!(SBC, 0xF9, 3, 4 /* +1 page */, Absolute_Y),
    0xE1u8 => instruction!(SBC, 0xE1, 2, 6, Indirect_X),
    0xF1u8 => instruction!(SBC, 0xF1, 2, 5 /* +1 page */, Indirect_Y),

    // SEC – Set Carry Flag
    0x38u8 => instruction!(SEC, 0x38, 1, 2, Implied),

    // SED – Set Decimal Flag
    0xF8u8 => instruction!(SED, 0xF8, 1, 2, Implied),

    // SEI – Set Interrupt Disable
    0x78u8 => instruction!(SEI, 0x78, 1, 2, Implied),

    // STA – Store Accumulator
    0x85u8 => instruction!(STA, 0x85, 2, 3, ZeroPage),
    0x95u8 => instruction!(STA, 0x95, 2, 4, ZeroPage_X),
    0x8Du8 => instruction!(STA, 0x8D, 3, 4, Absolute),
    0x9Du8 => instruction!(STA, 0x9D, 3, 5, Absolute_X),
    0x99u8 => instruction!(STA, 0x99, 3, 5, Absolute_Y),
    0x81u8 => instruction!(STA, 0x81, 2, 6, Indirect_X),
    0x91u8 => instruction!(STA, 0x91, 2, 6, Indirect_Y),

    // STX – Store X Register
    0x86u8 => instruction!(STX, 0x86, 2, 3, ZeroPage),
    0x96u8 => instruction!(STX, 0x96, 2, 4, ZeroPage_Y),
    0x8Eu8 => instruction!(STX, 0x8E, 3, 4, Absolute),

    // STY – Store Y Register
    0x84u8 => instruction!(STY, 0x84, 2, 3, ZeroPage),
    0x94u8 => instruction!(STY, 0x94, 2, 4, ZeroPage_X),
    0x8Cu8 => instruction!(STY, 0x8C, 3, 4, Absolute),

    // TAX – Transfer A to X
    0xAAu8 => instruction!(TAX, 0xAA, 1, 2, Implied),

    // TAY – Transfer A to Y
    0xA8u8 => instruction!(TAY, 0xA8, 1, 2, Implied),

    // TSX – Transfer Stack-pointer to X
    0xBAu8 => instruction!(TSX, 0xBA, 1, 2, Implied),

    // TXA – Transfer X to A
    0x8Au8 => instruction!(TXA, 0x8A, 1, 2, Implied),

    // TXS – Transfer X to Stack-pointer
    0x9Au8 => instruction!(TXS, 0x9A, 1, 2, Implied),

    // TYA – Transfer Y to A
    0x98u8 => instruction!(TYA, 0x98, 1, 2, Implied),
};
