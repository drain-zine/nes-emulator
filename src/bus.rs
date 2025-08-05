const RAM_START: u16 = 0x0000;
const RAM_MIRRORS_END: u16 = 0x1FFF;
const PPU_START: u16 = 0x2000;
const PPU_MIRRORS_END: u16 = 0x3FFF;

pub struct Bus {
    cpu_vram: [u8; 2048]
}

impl Bus {
    pub fn new() -> Self{
        Bus {
            cpu_vram: [0; 2048]
        }
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        match addr {
            RAM_START ..= RAM_MIRRORS_END => {
                let mirror_down_addr = addr & 0b00000111_11111111;
                self.cpu_vram[mirror_down_addr as usize]
            }
            PPU_START ..= PPU_MIRRORS_END => {
                let _mirror_down_addr = addr & 0b00100000_00000111;
                todo!("PPU is not supported yet")
            }
            _ => {
                let mirror_down_addr = addr & 0b00000111_11111111;
                self.cpu_vram[mirror_down_addr as usize]
            }
        }
    }
 
       pub fn mem_write(&mut self, addr: u16, data: u8) {
        match addr {
            RAM_START ..= RAM_MIRRORS_END => {
                let mirror_down_addr = addr & 0b00000111_11111111;
                self.cpu_vram[mirror_down_addr as usize] = data;
            }
            PPU_START ..= PPU_MIRRORS_END => {
                let _mirror_down_addr = addr & 0b00100000_00000111;
                todo!("PPU is not supported yet");
            }
            _ => {
                let mirror_down_addr = addr & 0b00000111_11111111;
                self.cpu_vram[mirror_down_addr as usize] = data;
            }
        }
    }

    pub fn mem_read_u16(&self, addr: u16) -> u16 {
        let lo = self.mem_read(addr);   
        let hi = self.mem_read(addr.wrapping_add(1));
        u16::from_le_bytes([lo, hi])
    }

    pub fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let [lo, hi] = data.to_le_bytes();
        self.mem_write(addr, lo);
        self.mem_write(addr.wrapping_add(1), hi);
    }
 }
