use crate::{
    data::Expression,
    face_control::{EXPRESSION_TICK_MS, FaceExpressionController},
};
use embassy_executor::task;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::Async;

const DEV_ADDR: u8 = 0x39;

/// Enabled state register
const REG_ENABLE: u8 = 0x80;

/// Proximity data register
const REG_PDATA: u8 = 0x9C;

/// Enable chip flag
const ENABLE_PON: u8 = 0b0000_0001;

/// Enable proximity flag
const ENABLE_PEN: u8 = 0b0000_0100;

/// Distance threshold for detection
const DISTANCE_THRESHOLD: u8 = 40;
const DISTANCE_THRESHOLD_ENTER: u8 = DISTANCE_THRESHOLD + 5;
const DISTANCE_THRESHOLD_EXIT: u8 = DISTANCE_THRESHOLD - 10;

type I2c = esp_hal::i2c::master::I2c<'static, Async>;
type I2cResult<V> = Result<V, esp_hal::i2c::master::Error>;

#[task]
pub async fn proximity_expression_task(
    mut i2c: I2c,
    expression_controller: FaceExpressionController,
) {
    // Enable the sensor and the proximity function
    if let Err(err) = write_register(&mut i2c, REG_ENABLE, ENABLE_PON | ENABLE_PEN).await {
        defmt::error!("failed to write proximity enable state: {}", err);
        return;
    };

    let mut object_close = false;
    let mut last_signal = Instant::now();

    let mut data: [u8; 1] = [0];

    loop {
        match read_register(&mut i2c, REG_PDATA, &mut data).await {
            Ok(value) => value,
            Err(err) => {
                defmt::error!("failed to read proximity data: {}", err);
                Timer::after(Duration::from_micros(10)).await;
                continue;
            }
        };

        let value = data[0];

        if object_close {
            // End touch detection
            if value < DISTANCE_THRESHOLD_EXIT {
                object_close = false;
                defmt::info!("Touch END = {}", value);
            }

            // If 50ms has elapsed since the last expression signal pulse the touched signal
            if last_signal.elapsed().as_millis() > EXPRESSION_TICK_MS {
                expression_controller.signal(Expression::Touched);
                last_signal = Instant::now();
            }
        } else {
            // Begin touch when over threshold
            if value > DISTANCE_THRESHOLD_ENTER {
                object_close = true;
                defmt::info!("Touch START = {}", value);

                expression_controller.signal(Expression::Touched);
                last_signal = Instant::now();
            }
        }

        Timer::after(Duration::from_micros(10)).await;
    }
}

async fn write_register(i2c: &mut I2c, address: u8, value: u8) -> I2cResult<()> {
    i2c.write_async(DEV_ADDR, &[address, value]).await
}

async fn read_register(i2c: &mut I2c, register: u8, data: &mut [u8]) -> I2cResult<()> {
    i2c.write_read_async(DEV_ADDR, &[register], data).await
}
