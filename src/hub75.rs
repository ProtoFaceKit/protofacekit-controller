use crate::data::{DISPLAY_HEIGHT, DISPLAY_WIDTH, DISPLAYS, FRAME_WIDTH, RLEPixelProducerIterator};
use crate::face_control::{FaceConsumer, face_render_task};
use crate::mk_static;
use embassy_executor::task;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_graphics::prelude::Point;
use esp_hal::gpio::AnyPin;
use esp_hal::interrupt::Priority;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::peripherals::{CPU_CTRL, LCD_CAM, SW_INTERRUPT};
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hub75::framebuffer::plain::DmaFrameBuffer;
use esp_hub75::framebuffer::tiling::{ChainTopRightDown, TiledFrameBuffer, compute_tiled_cols};
use esp_hub75::framebuffer::{FrameBufferOperations, compute_frame_count, compute_rows};
use esp_hub75::{Color, Hub75, Hub75Pins16};
use esp_rtos::embassy::{Executor, InterruptExecutor};

// Stuck with 4 bits for color, not enough stack room available for more
const BITS: u8 = 4;

const TILED_COLS: usize = DISPLAYS;
const TILED_ROWS: usize = 1;

const PANEL_ROWS: usize = DISPLAY_HEIGHT;
const PANEL_COLS: usize = DISPLAY_WIDTH;

const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
const NROWS: usize = compute_rows(PANEL_ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type Hub75Type = Hub75<'static, esp_hal::Async>;

type FBType = DmaFrameBuffer<PANEL_ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
type TiledFBType = TiledFrameBuffer<
    FBType,
    ChainTopRightDown<PANEL_ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
    PANEL_ROWS,
    PANEL_COLS,
    NROWS,
    BITS,
    FRAME_COUNT,
    TILED_ROWS,
    TILED_COLS,
    FB_COLS,
>;

type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut TiledFBType>;

pub struct Hub75Peripherals<'d> {
    pub lcd_cam: LCD_CAM<'d>,
    pub dma_channel: esp_hal::peripherals::DMA_CH0<'d>,
    pub red1: AnyPin<'d>,
    pub grn1: AnyPin<'d>,
    pub blu1: AnyPin<'d>,
    pub red2: AnyPin<'d>,
    pub grn2: AnyPin<'d>,
    pub blu2: AnyPin<'d>,
    pub addr0: AnyPin<'d>,
    pub addr1: AnyPin<'d>,
    pub addr2: AnyPin<'d>,
    pub addr3: AnyPin<'d>,
    pub addr4: AnyPin<'d>,
    pub blank: AnyPin<'d>,
    pub clock: AnyPin<'d>,
    pub latch: AnyPin<'d>,
}

static FB_TX: FrameBufferExchange = FrameBufferExchange::new();
static FB_RX: FrameBufferExchange = FrameBufferExchange::new();

/// Setup the second core and runtime to drive the Hub75 specific behavior that
/// needs higher priority execution
pub fn setup_hub75_dedicated_core(
    cpu_ctrl: CPU_CTRL<'static>,
    software_interrupts: SW_INTERRUPT<'static>,
    hub75_peripherals: Hub75Peripherals<'static>,
    face: FaceConsumer,
) {
    let software_interrupts = SoftwareInterruptControl::new(software_interrupts);
    let software_interrupt = software_interrupts.software_interrupt2;

    // Allocate frame buffers
    let fb0 = mk_static!(TiledFBType, TiledFBType::new());
    let fb1 = mk_static!(TiledFBType, TiledFBType::new());
    defmt::info!("initialized frame buffers");

    // Tasks for the second core to drive
    let cpu1_function = move || {
        let hp_executor = mk_static!(
            InterruptExecutor<2>,
            InterruptExecutor::new(software_interrupt)
        );
        let high_pri_spawner = hp_executor.start(Priority::Priority3);

        // hub75 runs as high priority task
        high_pri_spawner
            .spawn(matrix_driver(hub75_peripherals, fb1))
            .ok();

        let lp_executor = mk_static!(Executor, Executor::new());
        // display task runs as low priority task
        lp_executor.run(|spawner| {
            let face_matrix_driver = FaceMatrixDriver::new(&FB_TX, &FB_RX, fb0);
            spawner
                .spawn(face_render_task(face, face_matrix_driver))
                .ok();
        });
    };

    // Stack memory for the second core to use
    const DISPLAY_STACK_SIZE: usize = 4096;

    let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());

    esp_rtos::start_second_core(
        cpu_ctrl,
        software_interrupts.software_interrupt0,
        software_interrupts.software_interrupt1,
        app_core_stack,
        cpu1_function,
    );
}

pub struct FaceMatrixDriver {
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut TiledFBType,
}

impl FaceMatrixDriver {
    pub fn new(
        rx: &'static FrameBufferExchange,
        tx: &'static FrameBufferExchange,
        fb: &'static mut TiledFBType,
    ) -> Self {
        Self { rx, tx, fb }
    }

    /// Write all the contents from the `pixel_producer` to the current frame
    /// buffer, erases the frame buffer
    pub fn write_frame<'a>(&mut self, pixel_producer: RLEPixelProducerIterator<'a>) {
        self.fb.erase();

        for (index, (r, g, b)) in pixel_producer.enumerate() {
            let row = index / FRAME_WIDTH;
            let column = index % FRAME_WIDTH;
            let point = Point::new(column as i32, row as i32);
            self.fb.set_pixel(point, Color::new(r, g, b));
        }
    }

    /// Exchange the current frame buffer with the matrix driver to be
    /// rendered
    pub async fn swap(mut self) -> Self {
        self.tx.signal(self.fb);
        self.fb = self.rx.wait().await;
        self
    }
}

/// Task that drives the Hub75 LED matrix based on the current frame buffer
/// data and handles swapping buffers
#[task]
pub async fn matrix_driver(peripherals: Hub75Peripherals<'static>, fb: &'static mut TiledFBType) {
    let rx = &FB_RX;
    let tx = &FB_TX;

    defmt::info!("starting hub75 matrix driver");

    let channel = peripherals.dma_channel;
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());

    let pins = Hub75Pins16 {
        red1: peripherals.red1,
        grn1: peripherals.grn1,
        blu1: peripherals.blu1,
        red2: peripherals.red2,
        grn2: peripherals.grn2,
        blu2: peripherals.blu2,
        addr0: peripherals.addr0,
        addr1: peripherals.addr1,
        addr2: peripherals.addr2,
        addr3: peripherals.addr3,
        addr4: peripherals.addr4,
        blank: peripherals.blank,
        clock: peripherals.clock,
        latch: peripherals.latch,
    };

    let mut hub75 = Hub75Type::new_async(
        peripherals.lcd_cam,
        pins,
        channel,
        tx_descriptors,
        Rate::from_mhz(20),
    )
    .expect("failed to create hub75");

    let mut fb = fb;

    loop {
        // Handle swapping framebuffer when signaled
        if rx.signaled() {
            let new_fb = rx.wait().await;
            tx.signal(fb);
            fb = new_fb;
        }

        // Transfer the frame buffer to the device
        let mut transfer = hub75
            .render(fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to begin render");

        // Wait for the transfer
        transfer
            .wait_for_done()
            .await
            .expect("render DMA transfer failed");

        // Get the outcome
        let (result, new_hub75) = transfer.wait();

        // Store the hub75 again
        hub75 = new_hub75;
        result.expect("transfer failed");
    }
}
