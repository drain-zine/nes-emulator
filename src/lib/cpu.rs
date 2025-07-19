use super::instruction::{AddressingMode, Mnemonic, INSTRUCTION_TABLE};

const PROGRAM_START_ADDRESS: usize = 0x8000;
const PROGRAM_COUNTER_ADDRESS: u16 = 0xFFFC;

const CARRY_FLAG_MASK: u8 = 0b0000_0001;
const ZERO_FLAG_MASK: u8 = 0b0000_0010;
const OVERFLOW_FLAG_MASK: u8 = 0b0010_0000;
const NEGATIVE_FLAG_MASK: u8 = 0b0100_0000;

const SIGN_BIT: u8 = 0b1000_0000;

pub struct CPU {
    pub accumulator: u8,
    pub program_counter: u16,
    pub register_x: u8,
    pub register_y: u8,
    // 8 bit status register ->
    // Carry Flag (C), Zero Flag (Z), Interrupt Disable (I), Decimal Mode (D), Break Command (B), Overflow Flag (O), Negative Flag (N)
    pub status: u8,
    memory: [u8; 0xFFFF],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            accumulator: 0,
            program_counter: 0,
            register_x: 0,
            register_y: 0,
            status: 0,
            memory: [0; 0xFFFF],
        }
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_read_u16(&self, addr: u16) -> u16 {
        let lo = self.mem_read(addr);
        let hi = self.mem_read(addr.wrapping_add(1));
        u16::from_le_bytes([lo, hi])
    }

    pub fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let [lo, hi] = data.to_le_bytes();
        self.mem_write(addr, lo);
        self.mem_write(addr.wrapping_add(1), hi);
    }

    fn reset(&mut self) {
        self.accumulator = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.status = 0;

        self.program_counter = self.mem_read_u16(PROGRAM_COUNTER_ADDRESS);
    }

    pub fn load_rom(&mut self, program: Vec<u8>) {
        self.memory[PROGRAM_START_ADDRESS..PROGRAM_START_ADDRESS + program.len()]
            .copy_from_slice(&program);
        self.mem_write_u16(PROGRAM_COUNTER_ADDRESS, PROGRAM_START_ADDRESS as u16);
    }

    pub fn load_rom_and_run(&mut self, program: Vec<u8>) {
        self.load_rom(program);
        self.reset();
        self.run();
    }

    pub fn run(&mut self) {
        loop {
            let opcode = self.mem_read(self.program_counter);

            let instruction = INSTRUCTION_TABLE
                .get(&opcode)
                .unwrap_or_else(|| panic!("Unknown opcode: {:#X}", opcode));

            self.program_counter += 1;

            match instruction.mnemonic {
                Mnemonic::TAX => self.tax(),
                Mnemonic::LDA => self.lda(&instruction.addressing_mode),
                Mnemonic::LDX => self.ldx(&instruction.addressing_mode),
                Mnemonic::LDY => self.ldy(&instruction.addressing_mode),
                Mnemonic::INX => self.inx(),
                Mnemonic::BRK => return,
                _ => todo!(),
            }

            self.program_counter += (instruction.bytes - 1) as u16;
        }
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,
            AddressingMode::ZeroPage => self.mem_read(self.program_counter) as u16,
            AddressingMode::ZeroPage_X => {
                let base = self.mem_read(self.program_counter);
                base.wrapping_add(self.register_x) as u16
            }
            AddressingMode::ZeroPage_Y => {
                let base = self.mem_read(self.program_counter);
                base.wrapping_add(self.register_y) as u16
            }
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),
            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                base.wrapping_add(self.register_x as u16)
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                base.wrapping_add(self.register_y as u16)
            }
            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);
                let ptr = base.wrapping_add(self.register_x);

                self.mem_read_u16(ptr as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);

                let deref_base = self.mem_read_u16(base as u16);
                deref_base.wrapping_add(self.register_y as u16)
            }
            AddressingMode::Relative => {
                let offset = self.mem_read(self.program_counter) as i8;
                self.program_counter
                    .wrapping_add(1)
                    .wrapping_add(offset as u16)
            }
            // 6502 bug: if low byte is 0xFF, high byte doesnt wrap correctly
            AddressingMode::Indirect => {
                let addr = self.mem_read_u16(self.program_counter);
                let lo = self.mem_read(addr);
                let hi = if addr & 0x00FF == 0x00FF {
                    // emulate page boundary hardware bug
                    self.mem_read(addr & 0xFF00)
                } else {
                    self.mem_read(addr + 1)
                };
                u16::from_le_bytes([lo, hi])
            }
            AddressingMode::Accumulator => panic!("AddressingMode is implied"),
            AddressingMode::Implied => panic!("AddressMode is implied"),
        }
    }

    fn adc(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        let carry = ((self.status & CARRY_FLAG_MASK) != 0) as u8;

        let (initial_sum, carry_initial) = self.accumulator.overflowing_add(m);
        let (result, carry_result) = initial_sum.overflowing_add(carry);

        let carry = carry_initial || carry_result;
        let overflow = ((!(self.accumulator ^ m) & (self.accumulator ^ result)) & SIGN_BIT) != 0;

        self.accumulator = result;

        self.update_carry_flag(carry);
        self.update_overflow_flag(overflow);
        self.update_zero_negative_flags(self.accumulator);
    }

    fn tax(&mut self) {
        self.register_x = self.accumulator;

        self.update_zero_negative_flags(self.register_x)
    }

    fn lda(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let param = self.mem_read(addr);
        self.accumulator = param;

        self.update_zero_negative_flags(self.accumulator);
    }

    fn ldx(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let param = self.mem_read(addr);
        self.register_x = param;

        self.update_zero_negative_flags(self.register_x);
    }

    fn ldy(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let param = self.mem_read(addr);
        self.register_y = param;

        self.update_zero_negative_flags(self.register_y);
    }

    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);

        self.update_zero_negative_flags(self.register_x)
    }

    fn update_zero_negative_flags(&mut self, value: u8) {
        self.update_zero_flag(value);
        self.update_negative_flag(value);
    }

    fn update_zero_flag(&mut self, value: u8) {
        self.update_status_flag(ZERO_FLAG_MASK, value == 0);
    }

    fn update_negative_flag(&mut self, value: u8) {
        self.update_status_flag(NEGATIVE_FLAG_MASK, value & SIGN_BIT != 0);
    }

    fn update_carry_flag(&mut self, carry: bool) {
        self.update_status_flag(CARRY_FLAG_MASK, carry);
    }

    fn update_overflow_flag(&mut self, overflow: bool) {
        self.update_status_flag(OVERFLOW_FLAG_MASK, overflow);
    }

    fn update_status_flag(&mut self, mask: u8, condition: bool) {
        self.status = (self.status & !mask) | (if condition { mask } else { 0 });
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xaa() {
        let mut cpu = CPU::new();

        cpu.load_rom(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x05;
        cpu.run();

        assert_eq!(cpu.register_x, 0x05);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_0xaa_lda_zero_status() {
        let mut cpu = CPU::new();

        cpu.load_rom(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x00;
        cpu.run();

        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    #[test]
    fn test_0xaa_lda_negative_status() {
        let mut cpu = CPU::new();

        cpu.load_rom(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x81;
        cpu.run();

        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    #[test]
    fn test_0xa9() {
        let mut cpu = CPU::new();

        cpu.load_rom_and_run(vec![0xa9, 0x05, 0x00]);

        assert_eq!(cpu.accumulator, 0x05);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_0xa9_lda_zero_status() {
        let mut cpu = CPU::new();

        cpu.load_rom_and_run(vec![0xa9, 0x00, 0x00]);

        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    #[test]
    fn test_0xa9_lda_negative_status() {
        let mut cpu = CPU::new();
        cpu.load_rom_and_run(vec![0xa9, 0x81, 0x00]);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    #[test]
    fn test_0xe8() {
        let mut cpu = CPU::new();

        cpu.load_rom(vec![0xe8, 0x00]);
        cpu.reset();
        cpu.register_x = 0x00;
        cpu.run();

        assert_eq!(cpu.register_x, 0x01);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_0xe8_overflow() {
        let mut cpu = CPU::new();

        cpu.load_rom(vec![0xe8, 0xe8, 0x00]);
        cpu.reset();
        cpu.register_x = 0xff;
        cpu.run();

        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_0xe8_lda_negative_status() {
        let mut cpu = CPU::new();

        cpu.load_rom(vec![0xe8, 0x00]);
        cpu.reset();
        cpu.register_x = 0x7f;
        cpu.run();

        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }
}
