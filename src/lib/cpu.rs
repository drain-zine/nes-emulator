use super::instruction::{AddressingMode, Instruction, DISPATCH_TABLE};

const SIGN_BIT: u8 = 0b1000_0000;

const PROGRAM_START_ADDRESS: usize = 0x8000;
const PROGRAM_COUNTER_ADDRESS: u16 = 0xFFFC;

const CARRY_FLAG_MASK: u8 = 0b0000_0001;
const ZERO_FLAG_MASK: u8 = 0b0000_0010;
const OVERFLOW_FLAG_MASK: u8 = 0b0010_0000;
const NEGATIVE_FLAG_MASK: u8 = 0b0100_0000;

pub struct CPU {
    pub accumulator: u8,
    pub program_counter: u16,
    pub register_x: u8,
    pub register_y: u8,
    // 8 bit status register ->
    // Carry Flag (C), Zero Flag (Z), Interrupt Disable (I), Decimal Mode (D), Break Command (B), Overflow Flag (O), Negative Flag (N)
    pub status: u8,
    memory: [u8; 0x10000],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            accumulator: 0,
            program_counter: 0,
            register_x: 0,
            register_y: 0,
            status: 0,
            memory: [0; 0x10000],
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
        // self.memory = [0; 0x10000];

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

            if opcode == 0 {
                return;
            }

            let dispatch = DISPATCH_TABLE[opcode as usize]
                .as_ref()
                .unwrap_or_else(|| panic!("Unknown opcode: {:#X}", opcode));

            let instruction = dispatch.instruction;
            let handler = dispatch.handler;
            handler(self, &instruction.addressing_mode);

            self.program_counter += instruction.bytes as u16;
        }
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        let peek = self.program_counter.wrapping_add(1);

        match mode {
            AddressingMode::Immediate => peek,
            AddressingMode::ZeroPage => self.mem_read(peek) as u16,
            AddressingMode::ZeroPage_X => {
                let base = self.mem_read(peek);
                base.wrapping_add(self.register_x) as u16
            }
            AddressingMode::ZeroPage_Y => {
                let base = self.mem_read(peek);
                base.wrapping_add(self.register_y) as u16
            }
            AddressingMode::Absolute => self.mem_read_u16(peek),
            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(peek);
                base.wrapping_add(self.register_x as u16)
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(peek);
                base.wrapping_add(self.register_y as u16)
            }
            AddressingMode::Indirect_X => {
                let base = self.mem_read(peek);
                let ptr = base.wrapping_add(self.register_x);
                self.mem_read_u16(ptr as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(peek);
                let deref_base = self.mem_read_u16(base as u16);
                deref_base.wrapping_add(self.register_y as u16)
            }
            AddressingMode::Relative => {
                let offset = self.mem_read(peek) as i8;
                peek.wrapping_add(1).wrapping_add(offset as u16)
            }
            // 6502 bug: if low byte is 0xFF, high byte doesnt wrap correctly
            AddressingMode::Indirect => {
                let addr = self.mem_read_u16(peek);
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

    pub fn and(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        self.accumulator &= m;
    }

    pub fn adc(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        self.add_to_accumulator(m);
    }

    pub fn sbc(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        let (inverse_m, _) = (!m).overflowing_add(1);

        self.add_to_accumulator(inverse_m);
    }

    pub fn tax(&mut self, _: &AddressingMode) {
        self.register_x = self.accumulator;

        self.update_zero_negative_flags(self.register_x)
    }

    pub fn lda(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);
        self.accumulator = m;

        self.update_zero_negative_flags(self.accumulator);
    }

    pub fn ldx(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);
        self.register_x = m;

        self.update_zero_negative_flags(self.register_x);
    }

    pub fn ldy(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);
        self.register_y = m;

        self.update_zero_negative_flags(self.register_y);
    }

    pub fn inx(&mut self, _: &AddressingMode) {
        self.register_x = self.register_x.wrapping_add(1);

        self.update_zero_negative_flags(self.register_x)
    }

    fn add_to_accumulator(&mut self, value: u8) {
        let carry = ((self.status & CARRY_FLAG_MASK) != 0) as u8;

        let (initial_sum, carry_initial) = self.accumulator.overflowing_add(value);
        let (result, carry_result) = initial_sum.overflowing_add(carry);

        let carry = carry_initial || carry_result;
        let overflow =
            ((!(self.accumulator ^ value) & (self.accumulator ^ result)) & SIGN_BIT) != 0;

        self.accumulator = result;

        self.update_carry_flag(carry);
        self.update_overflow_flag(overflow);
        self.update_zero_negative_flags(self.accumulator);
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
    fn test_lda_immediate() {
        let mut cpu = CPU::new();

        cpu.load_rom_and_run(vec![0xa9, 0x05, 0x00]);

        assert_eq!(cpu.accumulator, 0x05);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_lda_immediate_zero_status() {
        let mut cpu = CPU::new();

        cpu.load_rom_and_run(vec![0xa9, 0x00, 0x00]);

        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    #[test]
    fn test_lda_immediate_negative_status() {
        let mut cpu = CPU::new();
        cpu.load_rom_and_run(vec![0xa9, 0x81, 0x00]);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    #[test]
    fn test_lda_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xA5, 0x10, 0x00]);
        cpu.reset();
        cpu.memory[0x10] = 0x84;
        cpu.run();

        assert_eq!(cpu.accumulator, 0x84);
    }

    #[test]
    fn test_lda_zero_page_x() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xB5, 0x10, 0x00]);
        cpu.reset();
        cpu.register_x = 0x01;
        cpu.memory[0x11] = 0x55;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x55);
    }

    #[test]
    fn test_lda_absolute() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xAD, 0x34, 0x12, 0x00]);
        cpu.reset();
        cpu.memory[0x1234] = 0x99;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x99);
    }

    #[test]
    fn test_lda_absolute_x() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xBD, 0x34, 0x12, 0x00]);
        cpu.reset();
        cpu.register_x = 0x01;
        cpu.memory[0x1235] = 0x10;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x10);
    }

    #[test]
    fn test_lda_absolute_y() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xB9, 0x34, 0x12, 0x00]);
        cpu.reset();
        cpu.register_y = 0x01;
        cpu.memory[0x1235] = 0x20;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x20);
    }

    #[test]
    fn test_lda_indirect_x() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xA1, 0x10, 0x00]);
        cpu.reset();
        cpu.register_x = 0x04;
        cpu.memory[0x14] = 0x80;
        cpu.memory[0x80] = 0xAB;
        cpu.run();
        assert_eq!(cpu.accumulator, 0xAB);
    }

    #[test]
    fn test_lda_indirect_y() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xB1, 0x10, 0x00]);
        cpu.reset();
        cpu.register_y = 0x04;
        cpu.memory[0x10] = 0x00;
        cpu.memory[0x11] = 0x80;
        cpu.memory[0x8004] = 0xCD;
        cpu.run();
        assert_eq!(cpu.accumulator, 0xCD);
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

    #[test]
    fn test_update_status_flag_sets_flag_when_condition_true() {
        let mut cpu = CPU::new();
        cpu.status = 0b0000_0000;

        cpu.update_status_flag(0b0000_0010, true);

        assert_eq!(cpu.status, 0b0000_0010);
    }

    #[test]
    fn test_update_status_flag_does_not_change_other_flags() {
        let mut cpu = CPU::new();
        cpu.status = 0b1010_0000;

        cpu.update_status_flag(0b0000_0010, true);

        assert_eq!(cpu.status, 0b1010_0010);
    }

    #[test]
    fn test_update_status_flag_clears_flag_when_condition_false() {
        let mut cpu = CPU::new();
        cpu.status = 0b1111_1111;

        cpu.update_status_flag(0b0000_0010, false);

        assert_eq!(cpu.status, 0b1111_1101);
    }

    #[test]
    fn test_update_status_flag_no_change_when_flag_already_clear() {
        let mut cpu = CPU::new();
        cpu.status = 0b0000_0000;

        cpu.update_status_flag(0b0000_0010, false);

        assert_eq!(cpu.status, 0b0000_0000);
    }

    #[test]
    fn test_update_status_flag_idempotent_set() {
        let mut cpu = CPU::new();
        cpu.status = 0b0000_0010;

        cpu.update_status_flag(0b0000_0010, true);

        assert_eq!(cpu.status, 0b0000_0010);
    }
}
