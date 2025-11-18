use crate::{data::Expression, face_control::FaceExpressionController};
use embassy_executor::task;
use embassy_time::{Duration, Timer};
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

type I2c = esp_hal::i2c::master::I2c<'static, Async>;
type I2cResult<V> = Result<V, esp_hal::i2c::master::Error>;

#[task]
pub async fn proximity_expression_task(
    mut i2c: I2c,
    expression_controller: FaceExpressionController,
) {
    // Enable the sensor and the proximity function
    write_register(&mut i2c, REG_ENABLE, ENABLE_PON | ENABLE_PEN)
        .await
        .expect("failed to enable proximity sensor");

    let threshold = 50;
    let mut data: [u8; 1] = [0];

    loop {
        read_register(&mut i2c, REG_PDATA, &mut data)
            .await
            .expect("failed to read proximity");

        let value = data[0];

        if value >= threshold {
            expression_controller.signal(Expression::TOUCHED);
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
