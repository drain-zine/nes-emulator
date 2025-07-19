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

#[derive(Debug, Clone, Copy)]
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

const INSTRUCTIONS: &[Instruction] = &[
    // ADC – Add with Carry
    instruction!(ADC, 0x69, 2, 2, Immediate),
    instruction!(ADC, 0x65, 2, 3, ZeroPage),
    instruction!(ADC, 0x75, 2, 4, ZeroPage_X),
    instruction!(ADC, 0x6D, 3, 4, Absolute),
    instruction!(ADC, 0x7D, 3, 4 /* +1 page */, Absolute_X),
    instruction!(ADC, 0x79, 3, 4 /* +1 page */, Absolute_Y),
    instruction!(ADC, 0x61, 2, 6, Indirect_X),
    instruction!(ADC, 0x71, 2, 5 /* +1 page */, Indirect_Y),
    // AND – Logical AND
    instruction!(AND, 0x29, 2, 2, Immediate),
    instruction!(AND, 0x25, 2, 3, ZeroPage),
    instruction!(AND, 0x35, 2, 4, ZeroPage_X),
    instruction!(AND, 0x2D, 3, 4, Absolute),
    instruction!(AND, 0x3D, 3, 4 /* +1 page */, Absolute_X),
    instruction!(AND, 0x39, 3, 4 /* +1 page */, Absolute_Y),
    instruction!(AND, 0x21, 2, 6, Indirect_X),
    instruction!(AND, 0x31, 2, 5 /* +1 page */, Indirect_Y),
    // ASL – Arithmetic Shift Left
    instruction!(ASL, 0x0A, 1, 2, Accumulator),
    instruction!(ASL, 0x06, 2, 5, ZeroPage),
    instruction!(ASL, 0x16, 2, 6, ZeroPage_X),
    instruction!(ASL, 0x0E, 3, 6, Absolute),
    instruction!(ASL, 0x1E, 3, 7, Absolute_X),
    // BCC – Branch if Carry Clear
    instruction!(BCC, 0x90, 2, 2 /* +1,+2 */, Relative),
    // BCS – Branch if Carry Set
    instruction!(BCS, 0xB0, 2, 2 /* +1,+2 */, Relative),
    // BEQ – Branch if Equal (Z=1)
    instruction!(BEQ, 0xF0, 2, 2 /* +1,+2 */, Relative),
    // BIT – Bit Test
    instruction!(BIT, 0x24, 2, 3, ZeroPage),
    instruction!(BIT, 0x2C, 3, 4, Absolute),
    // BMI – Branch if Minus (N=1)
    instruction!(BMI, 0x30, 2, 2 /* +1,+2 */, Relative),
    // BNE – Branch if Not Equal (Z=0)
    instruction!(BNE, 0xD0, 2, 2 /* +1,+2 */, Relative),
    // BPL – Branch if Positive (N=0)
    instruction!(BPL, 0x10, 2, 2 /* +1,+2 */, Relative),
    // BRK – Force Interrupt
    instruction!(BRK, 0x00, 1, 7, Implied),
    // BVC – Branch if Overflow Clear
    instruction!(BVC, 0x50, 2, 2 /* +1,+2 */, Relative),
    // BVS – Branch if Overflow Set
    instruction!(BVS, 0x70, 2, 2 /* +1,+2 */, Relative),
    // CLC – Clear Carry Flag
    instruction!(CLC, 0x18, 1, 2, Implied),
    // CLD – Clear Decimal Flag
    instruction!(CLD, 0xD8, 1, 2, Implied),
    // CLI – Clear Interrupt Disable
    instruction!(CLI, 0x58, 1, 2, Implied),
    // CLV – Clear Overflow Flag
    instruction!(CLV, 0xB8, 1, 2, Implied),
    // CMP – Compare A
    instruction!(CMP, 0xC9, 2, 2, Immediate),
    instruction!(CMP, 0xC5, 2, 3, ZeroPage),
    instruction!(CMP, 0xD5, 2, 4, ZeroPage_X),
    instruction!(CMP, 0xCD, 3, 4, Absolute),
    instruction!(CMP, 0xDD, 3, 4 /* +1 page */, Absolute_X),
    instruction!(CMP, 0xD9, 3, 4 /* +1 page */, Absolute_Y),
    instruction!(CMP, 0xC1, 2, 6, Indirect_X),
    instruction!(CMP, 0xD1, 2, 5 /* +1 page */, Indirect_Y),
    // CPX – Compare X
    instruction!(CPX, 0xE0, 2, 2, Immediate),
    instruction!(CPX, 0xE4, 2, 3, ZeroPage),
    instruction!(CPX, 0xEC, 3, 4, Absolute),
    // CPY – Compare Y
    instruction!(CPY, 0xC0, 2, 2, Immediate),
    instruction!(CPY, 0xC4, 2, 3, ZeroPage),
    instruction!(CPY, 0xCC, 3, 4, Absolute),
    // DEC – Decrement Memory
    instruction!(DEC, 0xC6, 2, 5, ZeroPage),
    instruction!(DEC, 0xD6, 2, 6, ZeroPage_X),
    instruction!(DEC, 0xCE, 3, 6, Absolute),
    instruction!(DEC, 0xDE, 3, 7, Absolute_X),
    // DEX – Decrement X
    instruction!(DEX, 0xCA, 1, 2, Implied),
    // DEY – Decrement Y
    instruction!(DEY, 0x88, 1, 2, Implied),
    // EOR – Exclusive OR
    instruction!(EOR, 0x49, 2, 2, Immediate),
    instruction!(EOR, 0x45, 2, 3, ZeroPage),
    instruction!(EOR, 0x55, 2, 4, ZeroPage_X),
    instruction!(EOR, 0x4D, 3, 4, Absolute),
    instruction!(EOR, 0x5D, 3, 4 /* +1 page */, Absolute_X),
    instruction!(EOR, 0x59, 3, 4 /* +1 page */, Absolute_Y),
    instruction!(EOR, 0x41, 2, 6, Indirect_X),
    instruction!(EOR, 0x51, 2, 5 /* +1 page */, Indirect_Y),
    // INC – Increment Memory
    instruction!(INC, 0xE6, 2, 5, ZeroPage),
    instruction!(INC, 0xF6, 2, 6, ZeroPage_X),
    instruction!(INC, 0xEE, 3, 6, Absolute),
    instruction!(INC, 0xFE, 3, 7, Absolute_X),
    // INX – Increment X
    instruction!(INX, 0xE8, 1, 2, Implied),
    // INY – Increment Y
    instruction!(INY, 0xC8, 1, 2, Implied),
    // JMP – Jump
    instruction!(JMP, 0x4C, 3, 3, Absolute),
    instruction!(JMP, 0x6C, 3, 5, Indirect),
    // JSR – Jump to Subroutine
    instruction!(JSR, 0x20, 3, 6, Absolute),
    // LDA – Load Accumulator
    instruction!(LDA, 0xA9, 2, 2, Immediate),
    instruction!(LDA, 0xA5, 2, 3, ZeroPage),
    instruction!(LDA, 0xB5, 2, 4, ZeroPage_X),
    instruction!(LDA, 0xAD, 3, 4, Absolute),
    instruction!(LDA, 0xBD, 3, 4 /* +1 page */, Absolute_X),
    instruction!(LDA, 0xB9, 3, 4 /* +1 page */, Absolute_Y),
    instruction!(LDA, 0xA1, 2, 6, Indirect_X),
    instruction!(LDA, 0xB1, 2, 5 /* +1 page */, Indirect_Y),
    // LDX – Load X Register
    instruction!(LDX, 0xA2, 2, 2, Immediate),
    instruction!(LDX, 0xA6, 2, 3, ZeroPage),
    instruction!(LDX, 0xB6, 2, 4, ZeroPage_Y),
    instruction!(LDX, 0xAE, 3, 4, Absolute),
    instruction!(LDX, 0xBE, 3, 4 /* +1 page */, Absolute_Y),
    // LDY – Load Y Register
    instruction!(LDY, 0xA0, 2, 2, Immediate),
    instruction!(LDY, 0xA4, 2, 3, ZeroPage),
    instruction!(LDY, 0xB4, 2, 4, ZeroPage_X),
    instruction!(LDY, 0xAC, 3, 4, Absolute),
    instruction!(LDY, 0xBC, 3, 4 /* +1 page */, Absolute_X),
    // LSR – Logical Shift Right
    instruction!(LSR, 0x4A, 1, 2, Accumulator),
    instruction!(LSR, 0x46, 2, 5, ZeroPage),
    instruction!(LSR, 0x56, 2, 6, ZeroPage_X),
    instruction!(LSR, 0x4E, 3, 6, Absolute),
    instruction!(LSR, 0x5E, 3, 7, Absolute_X),
    // NOP – No Operation
    instruction!(NOP, 0xEA, 1, 2, Implied),
    // ORA – Logical Inclusive OR
    instruction!(ORA, 0x09, 2, 2, Immediate),
    instruction!(ORA, 0x05, 2, 3, ZeroPage),
    instruction!(ORA, 0x15, 2, 4, ZeroPage_X),
    instruction!(ORA, 0x0D, 3, 4, Absolute),
    instruction!(ORA, 0x1D, 3, 4 /* +1 page */, Absolute_X),
    instruction!(ORA, 0x19, 3, 4 /* +1 page */, Absolute_Y),
    instruction!(ORA, 0x01, 2, 6, Indirect_X),
    instruction!(ORA, 0x11, 2, 5 /* +1 page */, Indirect_Y),
    // PHA – Push Accumulator
    instruction!(PHA, 0x48, 1, 3, Implied),
    // PHP – Push Processor Status
    instruction!(PHP, 0x08, 1, 3, Implied),
    // PLA – Pull Accumulator
    instruction!(PLA, 0x68, 1, 4, Implied),
    // PLP – Pull Processor Status
    instruction!(PLP, 0x28, 1, 4, Implied),
    // ROL – Rotate Left
    instruction!(ROL, 0x2A, 1, 2, Accumulator),
    instruction!(ROL, 0x26, 2, 5, ZeroPage),
    instruction!(ROL, 0x36, 2, 6, ZeroPage_X),
    instruction!(ROL, 0x2E, 3, 6, Absolute),
    instruction!(ROL, 0x3E, 3, 7, Absolute_X),
    // ROR – Rotate Right
    instruction!(ROR, 0x6A, 1, 2, Accumulator),
    instruction!(ROR, 0x66, 2, 5, ZeroPage),
    instruction!(ROR, 0x76, 2, 6, ZeroPage_X),
    instruction!(ROR, 0x6E, 3, 6, Absolute),
    instruction!(ROR, 0x7E, 3, 7, Absolute_X),
    // RTI – Return from Interrupt
    instruction!(RTI, 0x40, 1, 6, Implied),
    // RTS – Return from Subroutine
    instruction!(RTS, 0x60, 1, 6, Implied),
    // SBC – Subtract with Carry
    instruction!(SBC, 0xE9, 2, 2, Immediate),
    instruction!(SBC, 0xE5, 2, 3, ZeroPage),
    instruction!(SBC, 0xF5, 2, 4, ZeroPage_X),
    instruction!(SBC, 0xED, 3, 4, Absolute),
    instruction!(SBC, 0xFD, 3, 4 /* +1 page */, Absolute_X),
    instruction!(SBC, 0xF9, 3, 4 /* +1 page */, Absolute_Y),
    instruction!(SBC, 0xE1, 2, 6, Indirect_X),
    instruction!(SBC, 0xF1, 2, 5 /* +1 page */, Indirect_Y),
    // SEC – Set Carry Flag
    instruction!(SEC, 0x38, 1, 2, Implied),
    // SED – Set Decimal Flag
    instruction!(SED, 0xF8, 1, 2, Implied),
    // SEI – Set Interrupt Disable
    instruction!(SEI, 0x78, 1, 2, Implied),
    // STA – Store Accumulator
    instruction!(STA, 0x85, 2, 3, ZeroPage),
    instruction!(STA, 0x95, 2, 4, ZeroPage_X),
    instruction!(STA, 0x8D, 3, 4, Absolute),
    instruction!(STA, 0x9D, 3, 5, Absolute_X),
    instruction!(STA, 0x99, 3, 5, Absolute_Y),
    instruction!(STA, 0x81, 2, 6, Indirect_X),
    instruction!(STA, 0x91, 2, 6, Indirect_Y),
    // STX – Store X Register
    instruction!(STX, 0x86, 2, 3, ZeroPage),
    instruction!(STX, 0x96, 2, 4, ZeroPage_Y),
    instruction!(STX, 0x8E, 3, 4, Absolute),
    // STY – Store Y Register
    instruction!(STY, 0x84, 2, 3, ZeroPage),
    instruction!(STY, 0x94, 2, 4, ZeroPage_X),
    instruction!(STY, 0x8C, 3, 4, Absolute),
    // TAX – Transfer Accumulator to X
    instruction!(TAX, 0xAA, 1, 2, Implied),
    // TAY – Transfer Accumulator to Y
    instruction!(TAY, 0xA8, 1, 2, Implied),
    // TSX – Transfer Stack Pointer to X
    instruction!(TSX, 0xBA, 1, 2, Implied),
    // TXA – Transfer X to Accumulator
    instruction!(TXA, 0x8A, 1, 2, Implied),
    // TXS – Transfer X to Stack Pointer
    instruction!(TXS, 0x9A, 1, 2, Implied),
    // TYA – Transfer Y to Accumulator
    instruction!(TYA, 0x98, 1, 2, Implied),
];

const fn generate_instruction_table() -> [Option<Instruction>; 0x100] {
    let mut table: [Option<Instruction>; 0x100] = [None; 0x100];
    let mut i = 0;

    while i < INSTRUCTIONS.len() {
        let instr = INSTRUCTIONS[i];
        table[instr.opcode as usize] = Some(instr);
        i += 1;
    }

    table
}

pub static INSTRUCTION_TABLE: [Option<Instruction>; 0x100] = generate_instruction_table();
