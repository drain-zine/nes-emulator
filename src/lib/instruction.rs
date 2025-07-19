use phf::phf_map;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mnemonic {
    LDA,
    LDX,
    LDY,
    TAX,
    INX,
    BRK,
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
    0xA9u8 => instruction!(LDA, 0xA9, 2, 2, Immediate),
    0xA5u8 => instruction!(LDA, 0xA5, 2, 3, ZeroPage),
    0xB5u8 => instruction!(LDA, 0xB5, 2, 4, ZeroPage_X),
    0xADu8 => instruction!(LDA, 0xAD, 3, 4, Absolute),
    0xBDu8 => instruction!(LDA, 0xBD, 3, 4, Absolute_X),
    0xB9u8 => instruction!(LDA, 0xB9, 3, 4, Absolute_Y),
    0xA1u8 => instruction!(LDA, 0xA1, 2, 6, Indirect_X),
    0xB1u8 => instruction!(LDA, 0xB1, 2, 5, Indirect_Y),
    0xAAu8 => instruction!(TAX, 0xAA, 1, 2, Implied),
    0xE8u8 => instruction!(INX, 0xE8, 1, 2, Implied),
    0x00u8 => instruction!(BRK, 0x00, 1, 7, Implied),
};
