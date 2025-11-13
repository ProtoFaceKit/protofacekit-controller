use crate::data::{FRAME_HEIGHT, FRAME_WIDTH};
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Level, Output};

pub struct Hub75<'a> {
    buffer: heapless::Vec<heapless::Vec<LEDData, FRAME_WIDTH>, FRAME_HEIGHT>,
    brightness_step: u8,
    brightness_count: u8,
    pins: Hub75Pins<'a>,
}

const GAMMA8: [u8; 256] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14,
    14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27,
    27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46,
    47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72,
    73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104,
    105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137,
    138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
    177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220,
    223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255,
];

impl<'a> Hub75<'a> {
    pub fn new(pins: Hub75Pins<'a>, brightness_bits: u8) -> Self {
        let brightness_step = 1 << (8 - brightness_bits);
        let brightness_count = ((1 << brightness_bits as u16) - 1) as u8;

        Self {
            buffer: Default::default(),
            brightness_count,
            brightness_step,
            pins,
        }
    }

    /// Paint to the buffer
    pub fn paint(&mut self, data: impl Iterator<Item = (u8, u8, u8)>) {
        for (index, (r, g, b)) in data.enumerate() {
            let row = index % FRAME_HEIGHT;
            let column = index / FRAME_HEIGHT;

            let data = self
                .buffer
                .get_mut(row)
                .expect("should be within buffer range")
                .get_mut(column)
                .expect("should be within column range");

            if row >= FRAME_HEIGHT {
                data.r2 = GAMMA8[r as usize];
                data.g2 = GAMMA8[g as usize];
                data.b2 = GAMMA8[b as usize];
            } else {
                data.r1 = GAMMA8[r as usize];
                data.g1 = GAMMA8[g as usize];
                data.b1 = GAMMA8[b as usize];
            }
        }
    }

    pub async fn output(&mut self) {
        // Enable the output (The previous last row will continue to display)
        self.pins.oe.set_low();

        // PWM cycle
        for mut brightness in 0..self.brightness_count {
            brightness = (brightness + 1).saturating_mul(self.brightness_step);
            for (count, row) in self.buffer.iter().enumerate() {
                for element in row.iter() {
                    let r1 = if element.r1 >= brightness {
                        Level::High
                    } else {
                        Level::Low
                    };

                    let g1 = if element.g1 >= brightness {
                        Level::High
                    } else {
                        Level::Low
                    };

                    let b1 = if element.b1 >= brightness {
                        Level::High
                    } else {
                        Level::Low
                    };

                    let r2 = if element.r2 >= brightness {
                        Level::High
                    } else {
                        Level::Low
                    };

                    let g2 = if element.g2 >= brightness {
                        Level::High
                    } else {
                        Level::Low
                    };

                    let b2 = if element.b2 >= brightness {
                        Level::High
                    } else {
                        Level::Low
                    };

                    self.pins.r1.set_level(r1);
                    self.pins.g1.set_level(g1);
                    self.pins.b1.set_level(b1);

                    self.pins.r2.set_level(r2);
                    self.pins.g2.set_level(g2);
                    self.pins.b2.set_level(b2);

                    self.pins.clk.set_high();
                    self.pins.clk.set_low();
                }

                self.pins.oe.set_high();

                // Prevents ghosting, no idea why
                Timer::after(Duration::from_millis(2)).await;

                self.pins.lat.set_low();
                Timer::after(Duration::from_millis(2)).await;
                self.pins.lat.set_high();

                // Select row
                let addr_a = if count & 1 != 0 {
                    Level::High
                } else {
                    Level::Low
                };

                let addr_b = if count & 2 != 0 {
                    Level::High
                } else {
                    Level::Low
                };

                let addr_c = if count & 4 != 0 {
                    Level::High
                } else {
                    Level::Low
                };

                let addr_d = if count & 8 != 0 {
                    Level::High
                } else {
                    Level::Low
                };

                self.pins.addr_a.set_level(addr_a);
                self.pins.addr_b.set_level(addr_b);
                self.pins.addr_c.set_level(addr_c);
                self.pins.addr_c.set_level(addr_d);

                Timer::after(Duration::from_millis(2)).await;
                self.pins.oe.set_low();
            }
        }

        // Disable the output (Prevents one row from being much brighter than the others)
        self.pins.oe.set_high();
    }
}

#[derive(Default, Clone, Copy)]
struct LEDData {
    r1: u8,
    g1: u8,
    b1: u8,

    r2: u8,
    g2: u8,
    b2: u8,
}
impl LEDData {
    fn reset(&mut self) {
        self.r1 = 0;
        self.g1 = 0;
        self.b1 = 0;
        self.r2 = 0;
        self.g2 = 0;
        self.b2 = 0;
    }
}

pub struct Hub75Pins<'a> {
    //
    pub r1: Output<'a>,
    pub g1: Output<'a>,
    pub b1: Output<'a>,
    //
    pub r2: Output<'a>,
    pub g2: Output<'a>,
    pub b2: Output<'a>,
    //
    pub addr_a: Output<'a>,
    pub addr_b: Output<'a>,
    pub addr_c: Output<'a>,
    pub addr_d: Output<'a>,
    pub addr_e: Output<'a>,
    //
    pub clk: Output<'a>,
    pub oe: Output<'a>,
    pub lat: Output<'a>,
}
