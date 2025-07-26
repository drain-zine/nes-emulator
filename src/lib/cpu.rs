use super::instruction::{AddressingMode, Mnemonic, DISPATCH_TABLE};

const CARRY_FLAG_MASK: u8 = 0b0000_0001;
const ZERO_FLAG_MASK: u8 = 0b0000_0010;
const INTERRUPT_DISABLE_FLAG_MASK: u8 = 0b0000_0100;
const DECIMAL_MODE_FLAG_MASK: u8 = 0b0000_1000;
const BREAK_FLAG_MASK: u8 = 0b0001_0000;
const OVERFLOW_FLAG_MASK: u8 = 0b0100_0000;
const NEGATIVE_FLAG_MASK: u8 = 0b1000_0000;
const SIGN_BIT: u8 = NEGATIVE_FLAG_MASK;

const PROGRAM_START_ADDRESS: usize = 0x0600;
const PROGRAM_COUNTER_ADDRESS: u16 = 0xFFFC;
const STACK_BASE_ADDRESS: u16 = 0x0100;
const STACK_START_ADDRESS: u8 = 0x00FD;
const INITIAL_STATUS: u8 = 0b0100_0100;

pub struct CPU {
    pub accumulator: u8,
    pub program_counter: u16,
    pub register_x: u8,
    pub register_y: u8,
    // 8 bit status register ->
    // Carry Flag (C), Zero Flag (Z), Interrupt Disable (I), Decimal Mode (D), Break Command (B), X, Overflow Flag (O), Negative Flag (N)
    pub status: u8,
    pub stack_pointer: u8,
    memory: [u8; 0x10000],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            accumulator: 0,
            program_counter: 0,
            register_x: 0,
            register_y: 0,
            status: INITIAL_STATUS,
            stack_pointer: STACK_START_ADDRESS,
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

    pub fn reset(&mut self) {
        self.accumulator = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.status = INITIAL_STATUS;
        self.stack_pointer = STACK_START_ADDRESS;

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
        self.run_with_callback(|_| {});
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {
        loop {
            callback(self);
            let opcode = self.mem_read(self.program_counter);

            // Stop on two consecutive 0x00 bytes
            if opcode == 0 && self.mem_read(self.program_counter + 1) == 0 && self.program_counter != 0x0600 {
                return;
            }

            let dispatch = DISPATCH_TABLE[opcode as usize]
                .as_ref()
                .unwrap_or_else(|| panic!("Unknown opcode: {:#X}", opcode));

            println!("PC: {:#06X}, Op: {:#04X} ({:?})", self.program_counter, opcode, dispatch.instruction.mnemonic);
            println!("Direction: {:#02X}, X: {:#02X}, Y: {:#02X}", self.mem_read(0x02), self.mem_read(0x10), self.mem_read(0x11));

            let instruction = dispatch.instruction;
            let handler = dispatch.handler;
            handler(self, &instruction.addressing_mode);

           // These instructions handle their own PC updates
            match instruction.mnemonic {
                Mnemonic::JMP | Mnemonic::JSR | Mnemonic::RTS | Mnemonic::RTI | Mnemonic::BRK |
                Mnemonic::BCC | Mnemonic::BCS | Mnemonic::BEQ | Mnemonic::BNE | 
                Mnemonic::BMI | Mnemonic::BPL | Mnemonic::BVC | Mnemonic::BVS => {
    
                }
                _ => {
                    self.program_counter += instruction.bytes as u16;
                }
            }
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

    pub fn adc(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        self.add_to_accumulator(m);
    }

    pub fn and(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        self.accumulator &= m;
        self.update_zero_negative_flags(self.accumulator);
    }

    pub fn asl(&mut self, addressing_mode: &AddressingMode) {
        match addressing_mode {
            AddressingMode::Accumulator => {
                let old_carry = (self.accumulator & SIGN_BIT) != 0;
                self.accumulator <<= 1;
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(self.accumulator);
            }
            _ => {
                let addr = self.get_operand_address(addressing_mode);
                let mut data = self.mem_read(addr);
                let old_carry = (data & SIGN_BIT) != 0;
                data <<= 1;
                self.mem_write(addr, data);
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(data);
            }
        }
    }

    pub fn bcc(&mut self, addressing_mode: &AddressingMode) {
        self.branch(!self.is_carry_set());
    }

    pub fn bcs(&mut self, addressing_mode: &AddressingMode) {
        self.branch(self.is_carry_set());
    }

    pub fn beq(&mut self, addressing_mode: &AddressingMode) {
        self.branch(self.is_zero_set());
    }

    pub fn bit(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let data = self.mem_read(addr);
        let result = self.accumulator & data;
        
        self.update_zero_flag(result);
        self.update_negative_flag(data);
        self.update_overflow_flag((data & OVERFLOW_FLAG_MASK) != 0);
    }

    pub fn bmi(&mut self, addressing_mode: &AddressingMode) {
        self.branch(self.is_negative_set());
    }

    pub fn bne(&mut self, addressing_mode: &AddressingMode) {
        self.branch(!self.is_zero_set());
    }

    pub fn bpl(&mut self, addressing_mode: &AddressingMode) {
        self.branch(!self.is_negative_set());
    }

    pub fn brk(&mut self, _: &AddressingMode) {
        self.program_counter += 1;
        self.stack_push_u16(self.program_counter);
        
        let mut flags = self.status;
        flags |= BREAK_FLAG_MASK;
        self.stack_push(flags);
        
        self.update_interrupt_disable_flag(true);
        self.update_break_flag(true);
        
        self.program_counter = self.mem_read_u16(0xFFFE);
    }

    pub fn bvc(&mut self, addressing_mode: &AddressingMode) {
        self.branch(!self.is_overflow_set());
    }

    pub fn bvs(&mut self, addressing_mode: &AddressingMode) {
        self.branch(self.is_overflow_set());
    }

    pub fn clc(&mut self, _: &AddressingMode) {
        self.update_carry_flag(false);
    }

    pub fn cld(&mut self, _: &AddressingMode) {
        self.update_decimal_flag(false);
    }

    pub fn cli(&mut self, _: &AddressingMode) {
        self.update_interrupt_disable_flag(false);
    }

    pub fn clv(&mut self, _: &AddressingMode) {
        self.update_overflow_flag(false);
    }

    pub fn cmp(&mut self, addressing_mode: &AddressingMode) {
        self.compare(addressing_mode, self.accumulator);
    }

    pub fn cpx(&mut self, addressing_mode: &AddressingMode) {
        self.compare(addressing_mode, self.register_x);
    }

    pub fn cpy(&mut self, addressing_mode: &AddressingMode) {
        self.compare(addressing_mode, self.register_y);
    }

    pub fn dec(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(addr);
        data = data.wrapping_sub(1);
        self.mem_write(addr, data);
        self.update_zero_negative_flags(data);
    }

    pub fn dex(&mut self, _: &AddressingMode) {
        self.register_x = self.register_x.wrapping_sub(1);

        self.update_zero_negative_flags(self.register_x)
    }

    pub fn dey(&mut self, _: &AddressingMode) {
        self.register_y = self.register_y.wrapping_sub(1);

        self.update_zero_negative_flags(self.register_y)
    }

    pub fn eor(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        self.accumulator ^= m;
        self.update_zero_negative_flags(self.accumulator);
    }

    pub fn inc(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let mut data = self.mem_read(addr);
        data = data.wrapping_add(1);
        self.mem_write(addr, data);
        self.update_zero_negative_flags(data);
    }

    pub fn inx(&mut self, _: &AddressingMode) {
        self.register_x = self.register_x.wrapping_add(1);

        self.update_zero_negative_flags(self.register_x)
    }

    pub fn iny(&mut self, _: &AddressingMode) {
        self.register_y = self.register_y.wrapping_add(1);

        self.update_zero_negative_flags(self.register_y)
    }

    pub fn jmp(&mut self, addressing_mode: &AddressingMode) {
        match addressing_mode {
            AddressingMode::Absolute => {
                let addr = self.get_operand_address(addressing_mode);
                self.program_counter = addr;
            }
            AddressingMode::Indirect => {
                let addr = self.get_operand_address(addressing_mode);
                self.program_counter = addr;
            }
            _ => panic!("Invalid addressing mode for JMP"),
        }
    }

    pub fn jsr(&mut self, addressing_mode: &AddressingMode) {
        let return_addr = self.program_counter + 2;
        self.stack_push_u16(return_addr);
        
        let addr = self.get_operand_address(addressing_mode);
        self.program_counter = addr;
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

    pub fn lsr(&mut self, addressing_mode: &AddressingMode) {
        match addressing_mode {
            AddressingMode::Accumulator => {
                let old_carry = (self.accumulator & 1) != 0;
                self.accumulator >>= 1;
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(self.accumulator);
            }
            _ => {
                let addr = self.get_operand_address(addressing_mode);
                let mut data = self.mem_read(addr);
                let old_carry = (data & 1) != 0;
                data >>= 1;
                self.mem_write(addr, data);
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(data);
            }
        }
    }

    pub fn nop(&mut self, _: &AddressingMode) {}

    pub fn ora(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        self.accumulator |= m;
        self.update_zero_negative_flags(self.accumulator);
    }

    pub fn pha(&mut self, _: &AddressingMode) {
        self.stack_push(self.accumulator);
    }

    pub fn php(&mut self, _: &AddressingMode) {
        let mut flags = self.status;
        flags |= BREAK_FLAG_MASK;
        self.stack_push(flags);
    }

    pub fn pla(&mut self, _: &AddressingMode) {
        self.accumulator = self.stack_pop();
        self.update_zero_negative_flags(self.accumulator);
    }

    pub fn plp(&mut self, _: &AddressingMode) {
        self.status = self.stack_pop();
        self.clear_break_flag();
    }

    pub fn rol(&mut self, addressing_mode: &AddressingMode) {
        match addressing_mode {
            AddressingMode::Accumulator => {
                let old_carry = (self.accumulator & SIGN_BIT) != 0;
                let carry_in = (self.status & CARRY_FLAG_MASK) != 0;
                self.accumulator <<= 1;
                if carry_in {
                    self.accumulator |= 1;
                }
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(self.accumulator);
            }
            _ => {
                let addr = self.get_operand_address(addressing_mode);
                let mut data = self.mem_read(addr);
                let old_carry = (data & SIGN_BIT) != 0;
                let carry_in = (self.status & CARRY_FLAG_MASK) != 0;
                data <<= 1;
                if carry_in {
                    data |= 1;
                }
                self.mem_write(addr, data);
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(data);
            }
        }
    }

    pub fn ror(&mut self, addressing_mode: &AddressingMode) {
        match addressing_mode {
            AddressingMode::Accumulator => {
                let old_carry = (self.accumulator & 1) != 0;
                let carry_in = (self.status & CARRY_FLAG_MASK) != 0;
                self.accumulator >>= 1;
                if carry_in {
                    self.accumulator |= SIGN_BIT;
                }
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(self.accumulator);
            }
            _ => {
                let addr = self.get_operand_address(addressing_mode);
                let mut data = self.mem_read(addr);
                let old_carry = (data & 1) != 0;
                let carry_in = (self.status & CARRY_FLAG_MASK) != 0;
                data >>= 1;
                if carry_in {
                    data |= SIGN_BIT;
                }
                self.mem_write(addr, data);
                self.update_carry_flag(old_carry);
                self.update_zero_negative_flags(data);
            }
        }
    }

    pub fn rti(&mut self, _: &AddressingMode) {
        self.status = self.stack_pop();
        self.clear_break_flag();
        
        let return_addr = self.stack_pop_u16();
        self.program_counter = return_addr;
    }

    pub fn rts(&mut self, _: &AddressingMode) {
        let return_addr = self.stack_pop_u16();
        self.program_counter = return_addr + 1; 
    }

    pub fn sbc(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        let m = self.mem_read(addr);

        self.add_to_accumulator(m.wrapping_neg().wrapping_sub(1));
    }

    pub fn sec(&mut self, _: &AddressingMode) {
        self.update_carry_flag(true);
    }

    pub fn sed(&mut self, _: &AddressingMode) {
        self.update_decimal_flag(true);
    }

    pub fn sei(&mut self, _: &AddressingMode) {
        self.update_interrupt_disable_flag(true);
    }

    pub fn sta(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        self.mem_write(addr, self.accumulator);
    }

    pub fn stx(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        self.mem_write(addr, self.register_x);
    }

    pub fn sty(&mut self, addressing_mode: &AddressingMode) {
        let addr = self.get_operand_address(addressing_mode);
        self.mem_write(addr, self.register_y);
    }

    pub fn tax(&mut self, _: &AddressingMode) {
        self.register_x = self.accumulator;

        self.update_zero_negative_flags(self.register_x)
    }

    pub fn tay(&mut self, _: &AddressingMode) {
        self.register_y = self.accumulator;

        self.update_zero_negative_flags(self.register_y)
    }

    pub fn tsx(&mut self, _: &AddressingMode) {
        self.register_x = self.stack_pointer;

        self.update_zero_negative_flags(self.register_x)
    }

    pub fn txa(&mut self, _: &AddressingMode) {
        self.accumulator = self.register_x;

        self.update_zero_negative_flags(self.accumulator)
    }

    pub fn txs(&mut self, _: &AddressingMode) {
        self.stack_pointer = self.register_x;
    }

    pub fn tya(&mut self, _: &AddressingMode) {
        self.accumulator = self.register_y;

        self.update_zero_negative_flags(self.accumulator)
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

    fn branch(&mut self, condition: bool) {
        let offset = self.mem_read(self.program_counter + 1) as i8;
        if condition {
            let jump_addr = self.program_counter.wrapping_add(2).wrapping_add(offset as u16);
            self.program_counter = jump_addr;
        } else {
            self.program_counter += 2;
        }
    }

    fn compare(&mut self, addressing_mode: &AddressingMode, compare_with: u8) {
        let addr = self.get_operand_address(addressing_mode);
        let data = self.mem_read(addr);
        
        let result = compare_with.wrapping_sub(data);
        
        self.update_carry_flag(compare_with >= data);
        self.update_zero_negative_flags(result);
    }


    fn stack_push(&mut self, data: u8) {
        self.mem_write(STACK_BASE_ADDRESS + self.stack_pointer as u16, data);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.mem_read(STACK_BASE_ADDRESS + self.stack_pointer as u16)
    }

    fn stack_push_u16(&mut self, data: u16) {
        let [lo, hi] = data.to_le_bytes();
        self.stack_push(hi);
        self.stack_push(lo);
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop();
        let hi = self.stack_pop();
        u16::from_le_bytes([lo, hi])
    }

    fn is_carry_set(&self) -> bool {
        (self.status & CARRY_FLAG_MASK) != 0
    }

    fn is_zero_set(&self) -> bool {
        (self.status & ZERO_FLAG_MASK) != 0
    }

    fn is_negative_set(&self) -> bool {
        (self.status & NEGATIVE_FLAG_MASK) != 0
    }

    fn is_overflow_set(&self) -> bool {
        (self.status & OVERFLOW_FLAG_MASK) != 0
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

    fn update_decimal_flag(&mut self, decimal: bool) {
        self.update_status_flag(DECIMAL_MODE_FLAG_MASK, decimal);
    }

    fn update_interrupt_disable_flag(&mut self, interrupt_disable: bool) {
        self.update_status_flag(INTERRUPT_DISABLE_FLAG_MASK, interrupt_disable);
    }

    fn update_break_flag(&mut self, break_flag: bool) {
        self.update_status_flag(BREAK_FLAG_MASK, break_flag);
    }

    fn update_status_flag(&mut self, mask: u8, condition: bool) {
        self.status = (self.status & !mask) | (if condition { mask } else { 0 });
    }

    fn clear_break_flag(&mut self) {
        self.status &= !BREAK_FLAG_MASK;
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

    #[test]
    fn test_bit_instruction() {
        let mut cpu = CPU::new();
        
        // Test BIT instruction: LDA #$04, BIT $02, where $02 contains 0x04
        cpu.load_rom(vec![0xa9, 0x04, 0x24, 0x02, 0x00]);
        cpu.reset();
        cpu.memory[0x02] = 0x04; // Set memory location $02 to 0x04
        cpu.run();
        
        // After BIT, accumulator should still be 0x04
        assert_eq!(cpu.accumulator, 0x04);
        // Zero flag should be clear (0x04 & 0x04 = 0x04, not zero)
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        // Negative flag should be clear (0x04 doesn't have bit 7 set)
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
        // Overflow flag should be clear (0x04 doesn't have bit 6 set)
        assert!(cpu.status & OVERFLOW_FLAG_MASK == 0);
    }

    #[test]
    fn test_bit_instruction_zero_result() {
        let mut cpu = CPU::new();
        
        // Test BIT instruction: LDA #$04, BIT $02, where $02 contains 0x00
        cpu.load_rom(vec![0xa9, 0x04, 0x24, 0x02, 0x00]);
        cpu.reset();
        cpu.memory[0x02] = 0x00; // Set memory location $02 to 0x00
        cpu.run();
        
        // After BIT, accumulator should still be 0x04
        assert_eq!(cpu.accumulator, 0x04);
        // Zero flag should be set (0x04 & 0x00 = 0x00)
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    #[test]
    fn test_bit_instruction_negative_flag() {
        let mut cpu = CPU::new();
        
        // Test BIT instruction: LDA #$04, BIT $02, where $02 has bit 7 set
        cpu.load_rom(vec![0xa9, 0x04, 0x24, 0x02, 0x00]);
        cpu.reset();
        cpu.memory[0x02] = 0x80; // Set memory location $02 to 0x80 (bit 7 set)
        cpu.run();
        
        // After BIT, accumulator should still be 0x04
        assert_eq!(cpu.accumulator, 0x04);
        // Zero flag should be clear (0x04 & 0x80 = 0x00, but we're testing the memory value's bit 7)
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
        // Negative flag should be set (0x80 has bit 7 set)
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    #[test]
    fn test_bit_instruction_overflow_flag() {
        let mut cpu = CPU::new();
        
        // Test BIT instruction: LDA #$04, BIT $02, where $02 has bit 6 set
        cpu.load_rom(vec![0xa9, 0x04, 0x24, 0x02, 0x00]);
        cpu.reset();
        cpu.memory[0x02] = 0x40; // Set memory location $02 to 0x40 (bit 6 set)
        cpu.run();
        
        // After BIT, accumulator should still be 0x04
        assert_eq!(cpu.accumulator, 0x04);
        // Zero flag should be clear (0x04 & 0x40 = 0x00, but we're testing the memory value's bit 6)
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
        // Overflow flag should be set (0x40 has bit 6 set)
        assert!(cpu.status & OVERFLOW_FLAG_MASK != 0);
    }

    // ADC Tests
    #[test]
    fn test_adc_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x69, 0x05, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x03;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x08);
        assert!(cpu.status & CARRY_FLAG_MASK == 0);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_adc_with_carry() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x69, 0x05, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x03;
        cpu.status |= CARRY_FLAG_MASK; // Set carry flag
        cpu.run();
        assert_eq!(cpu.accumulator, 0x09);
    }

    #[test]
    fn test_adc_overflow() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x69, 0x7F, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x01;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x80);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
        assert!(cpu.status & OVERFLOW_FLAG_MASK != 0);
    }

    // AND Tests
    #[test]
    fn test_and_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x29, 0x0F, 0x00]);
        cpu.reset();
        cpu.accumulator = 0xFF;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x0F);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_and_zero_result() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x29, 0x00, 0x00]);
        cpu.reset();
        cpu.accumulator = 0xFF;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x00);
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    // ASL Tests
    #[test]
    fn test_asl_accumulator() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x0A, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x40;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x80);
        assert!(cpu.status & CARRY_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    #[test]
    fn test_asl_with_carry() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x0A, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x80;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x00);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    // SBC Tests
    #[test]
    fn test_sbc_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xE9, 0x02, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x05;
        cpu.status |= CARRY_FLAG_MASK; // Set carry flag
        cpu.run();
        assert_eq!(cpu.accumulator, 0x03);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
    }

    #[test]
    fn test_sbc_without_carry() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xE9, 0x02, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x05;
        // Carry flag is clear by default
        cpu.run();
        assert_eq!(cpu.accumulator, 0x02);
    }

    // EOR Tests
    #[test]
    fn test_eor_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x49, 0x0F, 0x00]);
        cpu.reset();
        cpu.accumulator = 0xF0;
        cpu.run();
        assert_eq!(cpu.accumulator, 0xFF);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    #[test]
    fn test_eor_zero_result() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x49, 0xFF, 0x00]);
        cpu.reset();
        cpu.accumulator = 0xFF;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x00);
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    // ORA Tests
    #[test]
    fn test_ora_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x09, 0x0F, 0x00]);
        cpu.reset();
        cpu.accumulator = 0xF0;
        cpu.run();
        assert_eq!(cpu.accumulator, 0xFF);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    // LDX Tests
    #[test]
    fn test_ldx_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xA2, 0x42, 0x00]);
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.register_x, 0x42);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_ldx_zero() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xA2, 0x00, 0x00]);
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.register_x, 0x00);
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    // LDY Tests
    #[test]
    fn test_ldy_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xA0, 0x42, 0x00]);
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.register_y, 0x42);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    // INY Tests
    #[test]
    fn test_iny() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC8, 0x00]);
        cpu.reset();
        cpu.register_y = 0x42;
        cpu.run();
        assert_eq!(cpu.register_y, 0x43);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_iny_overflow() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC8, 0x00]);
        cpu.reset();
        cpu.register_y = 0xFF;
        cpu.run();
        assert_eq!(cpu.register_y, 0x00);
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    // DEX Tests
    #[test]
    fn test_dex() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xCA, 0x00]);
        cpu.reset();
        cpu.register_x = 0x42;
        cpu.run();
        assert_eq!(cpu.register_x, 0x41);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_dex_underflow() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xCA, 0x00]);
        cpu.reset();
        cpu.register_x = 0x00;
        cpu.run();
        assert_eq!(cpu.register_x, 0xFF);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    // DEY Tests
    #[test]
    fn test_dey() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x88, 0x00]);
        cpu.reset();
        cpu.register_y = 0x42;
        cpu.run();
        assert_eq!(cpu.register_y, 0x41);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    // TAY Tests
    #[test]
    fn test_tay() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xA8, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x42;
        cpu.run();
        assert_eq!(cpu.register_y, 0x42);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    // TSX Tests
    #[test]
    fn test_tsx() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xBA, 0x00]);
        cpu.reset();
        cpu.stack_pointer = 0x42;
        cpu.run();
        assert_eq!(cpu.register_x, 0x42);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    // TXA Tests
    #[test]
    fn test_txa() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x8A, 0x00]);
        cpu.reset();
        cpu.register_x = 0x42;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x42);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    // TXS Tests
    #[test]
    fn test_txs() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x9A, 0x00]);
        cpu.reset();
        cpu.register_x = 0x42;
        cpu.run();
        assert_eq!(cpu.stack_pointer, 0x42);
        // TXS doesn't affect flags
    }

    // TYA Tests
    #[test]
    fn test_tya() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x98, 0x00]);
        cpu.reset();
        cpu.register_y = 0x42;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x42);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    // ASL Memory Tests
    #[test]
    fn test_asl_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x06, 0x10, 0x00]);
        cpu.reset();
        cpu.memory[0x10] = 0x40;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0x80);
        assert!(cpu.status & CARRY_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    // LSR Tests
    #[test]
    fn test_lsr_accumulator() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x4A, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x80;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x40);
        assert!(cpu.status & CARRY_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_lsr_with_carry() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x4A, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x01;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x00);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    // ROL Tests
    #[test]
    fn test_rol_accumulator() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x2A, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x40;
        cpu.status |= CARRY_FLAG_MASK; // Set carry flag
        cpu.run();
        assert_eq!(cpu.accumulator, 0x81);
        assert!(cpu.status & CARRY_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    // ROR Tests
    #[test]
    fn test_ror_accumulator() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x6A, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x01;
        cpu.status |= CARRY_FLAG_MASK; // Set carry flag
        cpu.run();
        assert_eq!(cpu.accumulator, 0x80);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    // INC Tests
    #[test]
    fn test_inc_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xE6, 0x10, 0x00]);
        cpu.reset();
        cpu.memory[0x10] = 0x42;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0x43);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_inc_overflow() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xE6, 0x10, 0x00]);
        cpu.reset();
        cpu.memory[0x10] = 0xFF;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0x00);
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
    }

    // DEC Tests
    #[test]
    fn test_dec_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC6, 0x10, 0x00]);
        cpu.reset();
        cpu.memory[0x10] = 0x42;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0x41);
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_dec_underflow() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC6, 0x10, 0x00]);
        cpu.reset();
        cpu.memory[0x10] = 0x00;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0xFF);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    // CMP Tests
    #[test]
    fn test_cmp_immediate_equal() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC9, 0x42, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x42;
        cpu.run();
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0);
    }

    #[test]
    fn test_cmp_immediate_greater() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC9, 0x42, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x80;
        cpu.run();
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK == 0); // 0x80 - 0x42 = 0x3E (positive)
    }

    #[test]
    fn test_cmp_immediate_less() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC9, 0x80, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x42;
        cpu.run();
        assert!(cpu.status & ZERO_FLAG_MASK == 0);
        assert!(cpu.status & CARRY_FLAG_MASK == 0);
        assert!(cpu.status & NEGATIVE_FLAG_MASK != 0);
    }

    // CPX Tests
    #[test]
    fn test_cpx_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xE0, 0x42, 0x00]);
        cpu.reset();
        cpu.register_x = 0x42;
        cpu.run();
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
    }

    // CPY Tests
    #[test]
    fn test_cpy_immediate() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xC0, 0x42, 0x00]);
        cpu.reset();
        cpu.register_y = 0x42;
        cpu.run();
        assert!(cpu.status & ZERO_FLAG_MASK != 0);
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
    }

    // STA Tests
    #[test]
    fn test_sta_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x85, 0x10, 0x00]);
        cpu.reset();
        cpu.accumulator = 0x42;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0x42);
    }

    // STX Tests
    #[test]
    fn test_stx_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x86, 0x10, 0x00]);
        cpu.reset();
        cpu.register_x = 0x42;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0x42);
    }

    // STY Tests
    #[test]
    fn test_sty_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x84, 0x10, 0x00]);
        cpu.reset();
        cpu.register_y = 0x42;
        cpu.run();
        assert_eq!(cpu.memory[0x10], 0x42);
    }

    // Flag Setting Tests
    #[test]
    fn test_sec() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x38, 0x00]);
        cpu.reset();
        cpu.run();
        assert!(cpu.status & CARRY_FLAG_MASK != 0);
    }

    #[test]
    fn test_sed() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xF8, 0x00]);
        cpu.reset();
        cpu.run();
        assert!(cpu.status & DECIMAL_MODE_FLAG_MASK != 0);
    }

    #[test]
    fn test_sei() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x78, 0x00]);
        cpu.reset();
        cpu.run();
        assert!(cpu.status & INTERRUPT_DISABLE_FLAG_MASK != 0);
    }

    // Flag Clearing Tests
    #[test]
    fn test_clc() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x18, 0x00]);
        cpu.reset();
        cpu.status |= CARRY_FLAG_MASK; // Set carry flag first
        cpu.run();
        assert!(cpu.status & CARRY_FLAG_MASK == 0);
    }

    #[test]
    fn test_cld() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xD8, 0x00]);
        cpu.reset();
        cpu.status |= DECIMAL_MODE_FLAG_MASK; // Set decimal flag first
        cpu.run();
        assert!(cpu.status & DECIMAL_MODE_FLAG_MASK == 0);
    }

    #[test]
    fn test_cli() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x58, 0x00]);
        cpu.reset();
        cpu.status |= INTERRUPT_DISABLE_FLAG_MASK; // Set interrupt flag first
        cpu.run();
        assert!(cpu.status & INTERRUPT_DISABLE_FLAG_MASK == 0);
    }

    #[test]
    fn test_clv() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xB8, 0x00]);
        cpu.reset();
        cpu.status |= OVERFLOW_FLAG_MASK; // Set overflow flag first
        cpu.run();
        assert!(cpu.status & OVERFLOW_FLAG_MASK == 0);
    }

    // NOP Test
    #[test]
    fn test_nop() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xEA, 0x00]);
        cpu.reset();
        let initial_pc = cpu.program_counter;
        let initial_status = cpu.status;
        cpu.run();
        // NOP should only increment PC, not change anything else
        assert_eq!(cpu.status, initial_status);
    }

    // Stack Operation Tests
    #[test]
    fn test_pha_pla() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x48, 0x68, 0x00]); // PHA, PLA
        cpu.reset();
        cpu.accumulator = 0x42;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x42); // Should be restored
    }

    #[test]
    fn test_php_plp() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x08, 0x28, 0x00]); // PHP, PLP
        cpu.reset();
        cpu.status = 0xFF;
        cpu.run();
        assert_eq!(cpu.status, 0xEF); // Status restored with break flag cleared
    }

    // Jump Tests
    #[test]
    fn test_jmp_absolute() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x4C, 0x34, 0x12, 0x00]); // JMP $1234
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.program_counter, 0x1234);
    }

    #[test]
    fn test_jsr_rts() {
        let mut cpu = CPU::new();
        // JSR to address 0x1234, then RTS back
        cpu.load_rom(vec![0x20, 0x34, 0x12, 0x00]); // JSR $1234
        cpu.reset();
        cpu.memory[0x1234] = 0x60; // RTS at the target address
        cpu.run();
        // Should return to address after JSR + 1
        assert_eq!(cpu.program_counter, 0x0603);
    }

    // Branch Tests
    #[test]
    fn test_bcc_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x90, 0x02, 0x00, 0x00]); // BCC +2
        cpu.reset();
        // Carry flag is clear by default
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward
    }

    #[test]
    fn test_bcc_not_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x90, 0x02, 0x00, 0x00]); // BCC +2
        cpu.reset();
        cpu.status |= CARRY_FLAG_MASK; // Set carry flag
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0602); // Should not jump
    }

    #[test]
    fn test_bcs_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xB0, 0x02, 0x00, 0x00]); // BCS +2
        cpu.reset();
        cpu.status |= CARRY_FLAG_MASK; // Set carry flag
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward
    }

    #[test]
    fn test_beq_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xF0, 0x02, 0x00, 0x00]); // BEQ +2
        cpu.reset();
        cpu.status |= ZERO_FLAG_MASK; // Set zero flag
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward
    }

    #[test]
    fn test_bne_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0xD0, 0x02, 0x00, 0x00]); // BNE +2
        cpu.reset();
        // Zero flag is clear by default
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward
    }

    #[test]
    fn test_bmi_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x30, 0x02, 0x00, 0x00]); // BMI +2
        cpu.reset();
        cpu.status |= NEGATIVE_FLAG_MASK; // Set negative flag
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward
    }

    #[test]
    fn test_bpl_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x10, 0x02, 0x00, 0x00]); // BPL +2
        cpu.reset();
        // Negative flag is clear by default
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward
    }

    #[test]
    fn test_bvc_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x50, 0x02, 0x00, 0x00]); // BVC +2
        cpu.reset();
        cpu.status &= !OVERFLOW_FLAG_MASK; // Clear overflow flag
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward (0x0600 + 2 + 2)
    }

    #[test]
    fn test_bvs_taken() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x70, 0x02, 0x00, 0x00]); // BVS +2
        cpu.reset();
        cpu.status |= OVERFLOW_FLAG_MASK; // Set overflow flag
        cpu.run();
        assert_eq!(cpu.program_counter, 0x0604); // Should jump forward
    }

    // BRK Test
    #[test]
    fn test_brk() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x00, 0x00]); // BRK
        cpu.reset();
        cpu.memory[0xFFFE] = 0x34; // Interrupt vector low byte
        cpu.memory[0xFFFF] = 0x12; // Interrupt vector high byte
        cpu.run();
        assert_eq!(cpu.program_counter, 0x1234);
        assert!(cpu.status & INTERRUPT_DISABLE_FLAG_MASK != 0);
        assert!(cpu.status & BREAK_FLAG_MASK != 0);
    }

    // RTI Test
    #[test]
    fn test_rti() {
        let mut cpu = CPU::new();
        cpu.load_rom(vec![0x40, 0x00]); // RTI
        cpu.reset();
        cpu.stack_pointer = 0xFB;
        cpu.memory[0x01FC] = 0xFF; // Status register
        cpu.memory[0x01FD] = 0x34; // Return address low byte
        cpu.memory[0x01FE] = 0x12; // Return address high byte
        cpu.run();
        assert_eq!(cpu.program_counter, 0x1234);
        assert_eq!(cpu.status, 0xEF); // Status restored with break flag cleared
    }
}
