/* https://www.nesdev.org/obelisk-6502-guide/reference.html#LDX */

use super::cpu::CPU;

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

#[derive(Debug, Clone, Copy)]
pub struct Dispatch {
    pub instruction: Instruction,
    pub handler: fn(&mut CPU, &AddressingMode),
}

macro_rules! instruction {
    ($mnemonic:ident, $opcode:expr, $bytes:expr, $cycles:expr, $mode:ident, $handler: ident) => {
        Dispatch {
            instruction: Instruction {
                mnemonic: Mnemonic::$mnemonic,
                opcode: $opcode,
                bytes: $bytes,
                cycles: $cycles,
                addressing_mode: AddressingMode::$mode,
            },
            handler: CPU::$handler,
        }
    };
}

const DISPATCHES: &[Dispatch] = &[
    // ADC – Add with Carry
    instruction!(ADC, 0x69, 2, 2, Immediate, adc),
    instruction!(ADC, 0x65, 2, 3, ZeroPage, adc),
    instruction!(ADC, 0x75, 2, 4, ZeroPage_X, adc),
    instruction!(ADC, 0x6D, 3, 4, Absolute, adc),
    instruction!(ADC, 0x7D, 3, 4 /* +1 page */, Absolute_X, adc),
    instruction!(ADC, 0x79, 3, 4 /* +1 page */, Absolute_Y, adc),
    instruction!(ADC, 0x61, 2, 6, Indirect_X, adc),
    instruction!(ADC, 0x71, 2, 5 /* +1 page */, Indirect_Y, adc),
    // AND – Logical AND
    instruction!(AND, 0x29, 2, 2, Immediate, and),
    instruction!(AND, 0x25, 2, 3, ZeroPage, and),
    instruction!(AND, 0x35, 2, 4, ZeroPage_X, and),
    instruction!(AND, 0x2D, 3, 4, Absolute, and),
    instruction!(AND, 0x3D, 3, 4 /* +1 page */, Absolute_X, and),
    instruction!(AND, 0x39, 3, 4 /* +1 page */, Absolute_Y, and),
    instruction!(AND, 0x21, 2, 6, Indirect_X, and),
    instruction!(AND, 0x31, 2, 5 /* +1 page */, Indirect_Y, and),
    // // ASL – Arithmetic Shift Left
    instruction!(ASL, 0x0A, 1, 2, Accumulator, asl),
    instruction!(ASL, 0x06, 2, 5, ZeroPage, asl),
    instruction!(ASL, 0x16, 2, 6, ZeroPage_X, asl),
    instruction!(ASL, 0x0E, 3, 6, Absolute, asl),
    instruction!(ASL, 0x1E, 3, 7, Absolute_X, asl),
    // BCC – Branch if Carry Clear
    instruction!(BCC, 0x90, 2, 2 /* +1,+2 */, Relative, bcc),
    // BCS – Branch if Carry Set
    instruction!(BCS, 0xB0, 2, 2 /* +1,+2 */, Relative, bcs),
    // BEQ – Branch if Equal (Z=1)
    instruction!(BEQ, 0xF0, 2, 2 /* +1,+2 */, Relative, beq),
    // BIT – Bit Test
    instruction!(BIT, 0x24, 2, 3, ZeroPage, bit),
    instruction!(BIT, 0x2C, 3, 4, Absolute, bit),
    // BMI – Branch if Minus (N=1)
    instruction!(BMI, 0x30, 2, 2 /* +1,+2 */, Relative, bmi),
    // BNE – Branch if Not Equal (Z=0)
    instruction!(BNE, 0xD0, 2, 2 /* +1,+2 */, Relative, bne),
    // BPL – Branch if Positive (N=0)
    instruction!(BPL, 0x10, 2, 2 /* +1,+2 */, Relative, bpl),
    // BRK – Force Interrupt
    instruction!(BRK, 0x00, 1, 7, Implied, brk),
    // BVC – Branch if Overflow Clear
    instruction!(BVC, 0x50, 2, 2 /* +1,+2 */, Relative, bvc),
    // BVS – Branch if Overflow Set
    instruction!(BVS, 0x70, 2, 2 /* +1,+2 */, Relative, bvs),
    // CLC – Clear Carry Flag
    instruction!(CLC, 0x18, 1, 2, Implied, clc),
    // CLD – Clear Decimal Flag
    instruction!(CLD, 0xD8, 1, 2, Implied, cld),
    // CLI – Clear Interrupt Disable
    instruction!(CLI, 0x58, 1, 2, Implied, cli),
    // CLV – Clear Overflow Flag
    instruction!(CLV, 0xB8, 1, 2, Implied, clv),
    // CMP – Compare A
    instruction!(CMP, 0xC9, 2, 2, Immediate, cmp),
    instruction!(CMP, 0xC5, 2, 3, ZeroPage, cmp),
    instruction!(CMP, 0xD5, 2, 4, ZeroPage_X, cmp),
    instruction!(CMP, 0xCD, 3, 4, Absolute, cmp),
    instruction!(CMP, 0xDD, 3, 4 /* +1 page */, Absolute_X, cmp),
    instruction!(CMP, 0xD9, 3, 4 /* +1 page */, Absolute_Y, cmp),
    instruction!(CMP, 0xC1, 2, 6, Indirect_X, cmp),
    instruction!(CMP, 0xD1, 2, 5 /* +1 page */, Indirect_Y, cmp),
    // CPX – Compare X
    instruction!(CPX, 0xE0, 2, 2, Immediate, cpx),
    instruction!(CPX, 0xE4, 2, 3, ZeroPage, cpx),
    instruction!(CPX, 0xEC, 3, 4, Absolute, cpx),
    // CPY – Compare Y
    instruction!(CPY, 0xC0, 2, 2, Immediate, cpy),
    instruction!(CPY, 0xC4, 2, 3, ZeroPage, cpy),
    instruction!(CPY, 0xCC, 3, 4, Absolute, cpy),
    // DEC – Decrement Memory
    instruction!(DEC, 0xC6, 2, 5, ZeroPage, dec),
    instruction!(DEC, 0xD6, 2, 6, ZeroPage_X, dec),
    instruction!(DEC, 0xCE, 3, 6, Absolute, dec),
    instruction!(DEC, 0xDE, 3, 7, Absolute_X, dec),
    // DEX – Decrement X
    instruction!(DEX, 0xCA, 1, 2, Implied, dex),
    // DEY – Decrement Y
    instruction!(DEY, 0x88, 1, 2, Implied, dey),
    // EOR – Exclusive OR
    instruction!(EOR, 0x49, 2, 2, Immediate, eor),
    instruction!(EOR, 0x45, 2, 3, ZeroPage, eor),
    instruction!(EOR, 0x55, 2, 4, ZeroPage_X, eor),
    instruction!(EOR, 0x4D, 3, 4, Absolute, eor),
    instruction!(EOR, 0x5D, 3, 4 /* +1 page */, Absolute_X, eor),
    instruction!(EOR, 0x59, 3, 4 /* +1 page */, Absolute_Y, eor),
    instruction!(EOR, 0x41, 2, 6, Indirect_X, eor),
    instruction!(EOR, 0x51, 2, 5 /* +1 page */, Indirect_Y, eor),
    // INC – Increment Memory
    instruction!(INC, 0xE6, 2, 5, ZeroPage, inc),
    instruction!(INC, 0xF6, 2, 6, ZeroPage_X, inc),
    instruction!(INC, 0xEE, 3, 6, Absolute, inc),
    instruction!(INC, 0xFE, 3, 7, Absolute_X, inc),
    // INX – Increment X
    instruction!(INX, 0xE8, 1, 2, Implied, inx),
    // INY – Increment Y
    instruction!(INY, 0xC8, 1, 2, Implied, iny),
    // JMP – Jump
    instruction!(JMP, 0x4C, 3, 3, Absolute, jmp),
    instruction!(JMP, 0x6C, 3, 5, Indirect, jmp),
    // JSR – Jump to Subroutine
    instruction!(JSR, 0x20, 3, 6, Absolute, jsr),
    // LDA – Load Accumulator
    instruction!(LDA, 0xA9, 2, 2, Immediate, lda),
    instruction!(LDA, 0xA5, 2, 3, ZeroPage, lda),
    instruction!(LDA, 0xB5, 2, 4, ZeroPage_X, lda),
    instruction!(LDA, 0xAD, 3, 4, Absolute, lda),
    instruction!(LDA, 0xBD, 3, 4 /* +1 page */, Absolute_X, lda),
    instruction!(LDA, 0xB9, 3, 4 /* +1 page */, Absolute_Y, lda),
    instruction!(LDA, 0xA1, 2, 6, Indirect_X, lda),
    instruction!(LDA, 0xB1, 2, 5 /* +1 page */, Indirect_Y, lda),
    // LDX – Load X Register
    instruction!(LDX, 0xA2, 2, 2, Immediate, ldx),
    instruction!(LDX, 0xA6, 2, 3, ZeroPage, ldx),
    instruction!(LDX, 0xB6, 2, 4, ZeroPage_Y, ldx),
    instruction!(LDX, 0xAE, 3, 4, Absolute, ldx),
    instruction!(LDX, 0xBE, 3, 4 /* +1 page */, Absolute_Y, ldx),
    // LDY – Load Y Register
    instruction!(LDY, 0xA0, 2, 2, Immediate, ldy),
    instruction!(LDY, 0xA4, 2, 3, ZeroPage, ldy),
    instruction!(LDY, 0xB4, 2, 4, ZeroPage_X, ldy),
    instruction!(LDY, 0xAC, 3, 4, Absolute, ldy),
    instruction!(LDY, 0xBC, 3, 4 /* +1 page */, Absolute_X, ldy),
    // LSR – Logical Shift Right
    instruction!(LSR, 0x4A, 1, 2, Accumulator, lsr),
    instruction!(LSR, 0x46, 2, 5, ZeroPage, lsr),
    instruction!(LSR, 0x56, 2, 6, ZeroPage_X, lsr),
    instruction!(LSR, 0x4E, 3, 6, Absolute, lsr),
    instruction!(LSR, 0x5E, 3, 7, Absolute_X, lsr),
    // NOP – No Operation
    instruction!(NOP, 0xEA, 1, 2, Implied, nop),
    // ORA – Logical Inclusive OR
    instruction!(ORA, 0x09, 2, 2, Immediate, ora),
    instruction!(ORA, 0x05, 2, 3, ZeroPage, ora),
    instruction!(ORA, 0x15, 2, 4, ZeroPage_X, ora),
    instruction!(ORA, 0x0D, 3, 4, Absolute, ora),
    instruction!(ORA, 0x1D, 3, 4 /* +1 page */, Absolute_X, ora),
    instruction!(ORA, 0x19, 3, 4 /* +1 page */, Absolute_Y, ora),
    instruction!(ORA, 0x01, 2, 6, Indirect_X, ora),
    instruction!(ORA, 0x11, 2, 5 /* +1 page */, Indirect_Y, ora),
    // PHA – Push Accumulator
    instruction!(PHA, 0x48, 1, 3, Implied, pha),
    // PHP – Push Processor Status
    instruction!(PHP, 0x08, 1, 3, Implied, php),
    // PLA – Pull Accumulator
    instruction!(PLA, 0x68, 1, 4, Implied, pla),
    // PLP – Pull Processor Status
    instruction!(PLP, 0x28, 1, 4, Implied, plp),
    // ROL – Rotate Left
    instruction!(ROL, 0x2A, 1, 2, Accumulator, rol),
    instruction!(ROL, 0x26, 2, 5, ZeroPage, rol),
    instruction!(ROL, 0x36, 2, 6, ZeroPage_X, rol),
    instruction!(ROL, 0x2E, 3, 6, Absolute, rol),
    instruction!(ROL, 0x3E, 3, 7, Absolute_X, rol),
    // ROR – Rotate Right
    instruction!(ROR, 0x6A, 1, 2, Accumulator, ror),
    instruction!(ROR, 0x66, 2, 5, ZeroPage, ror),
    instruction!(ROR, 0x76, 2, 6, ZeroPage_X, ror),
    instruction!(ROR, 0x6E, 3, 6, Absolute, ror),
    instruction!(ROR, 0x7E, 3, 7, Absolute_X, ror),
    // RTI – Return from Interrupt
    instruction!(RTI, 0x40, 1, 6, Implied, rti),
    // RTS – Return from Subroutine
    instruction!(RTS, 0x60, 1, 6, Implied, rts),
    // SBC – Subtract with Carry
    instruction!(SBC, 0xE9, 2, 2, Immediate, sbc),
    instruction!(SBC, 0xE5, 2, 3, ZeroPage, sbc),
    instruction!(SBC, 0xF5, 2, 4, ZeroPage_X, sbc),
    instruction!(SBC, 0xED, 3, 4, Absolute, sbc),
    instruction!(SBC, 0xFD, 3, 4 /* +1 page */, Absolute_X, sbc),
    instruction!(SBC, 0xF9, 3, 4 /* +1 page */, Absolute_Y, sbc),
    instruction!(SBC, 0xE1, 2, 6, Indirect_X, sbc),
    instruction!(SBC, 0xF1, 2, 5 /* +1 page */, Indirect_Y, sbc),
    // SEC – Set Carry Flag
    instruction!(SEC, 0x38, 1, 2, Implied, sec),
    // SED – Set Decimal Flag
    instruction!(SED, 0xF8, 1, 2, Implied, sed),
    // SEI – Set Interrupt Disable
    instruction!(SEI, 0x78, 1, 2, Implied, sei),
    // STA – Store Accumulator
    instruction!(STA, 0x85, 2, 3, ZeroPage, sta),
    instruction!(STA, 0x95, 2, 4, ZeroPage_X, sta),
    instruction!(STA, 0x8D, 3, 4, Absolute, sta),
    instruction!(STA, 0x9D, 3, 5, Absolute_X, sta),
    instruction!(STA, 0x99, 3, 5, Absolute_Y, sta),
    instruction!(STA, 0x81, 2, 6, Indirect_X, sta),
    instruction!(STA, 0x91, 2, 6, Indirect_Y, sta),
    // STX – Store X Register
    instruction!(STX, 0x86, 2, 3, ZeroPage, stx),
    instruction!(STX, 0x96, 2, 4, ZeroPage_Y, stx),
    instruction!(STX, 0x8E, 3, 4, Absolute, stx),
    // STY – Store Y Register
    instruction!(STY, 0x84, 2, 3, ZeroPage, sty),
    instruction!(STY, 0x94, 2, 4, ZeroPage_X, sty),
    instruction!(STY, 0x8C, 3, 4, Absolute, sty),
    // TAX – Transfer Accumulator to X
    instruction!(TAX, 0xAA, 1, 2, Implied, tax),
    // TAY – Transfer Accumulator to Y
    instruction!(TAY, 0xA8, 1, 2, Implied, tay),
    // TSX – Transfer Stack Pointer to X
    instruction!(TSX, 0xBA, 1, 2, Implied, tsx),
    // TXA – Transfer X to Accumulator
    instruction!(TXA, 0x8A, 1, 2, Implied, txa),
    // TXS – Transfer X to Stack Pointer
    instruction!(TXS, 0x9A, 1, 2, Implied, txs),
    // TYA – Transfer Y to Accumulator
    instruction!(TYA, 0x98, 1, 2, Implied, tya),
];

const fn generate_instruction_table() -> [Option<Dispatch>; 0x100] {
    let mut table: [Option<Dispatch>; 0x100] = [None; 0x100];
    let mut i = 0;

    while i < DISPATCHES.len() {
        let dispatch = DISPATCHES[i];
        table[dispatch.instruction.opcode as usize] = Some(dispatch);
        i += 1;
    }

    table
}

pub static DISPATCH_TABLE: [Option<Dispatch>; 0x100] = generate_instruction_table();
