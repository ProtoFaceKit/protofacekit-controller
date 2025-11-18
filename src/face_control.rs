use crate::{
    data::{
        BeginExpressionError, BeginFrameError, Expression, Face, PushPixelError,
        RLEPixelProducerIterator, RLESlicePixelIterator,
    },
    hub75::FaceMatrixDriver,
};
use alloc::string::String;
use embassy_executor::task;
use embassy_futures::select::{Either, select};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::{Mutex, MutexGuard},
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use embedded_graphics::{mono_font::MonoTextStyle, prelude::Point, text::Alignment};
use esp_hub75::Color;

/// Consumer of face data
pub struct FaceConsumer {
    pub face: &'static Mutex<CriticalSectionRawMutex, Face>,
    pub expression_signal: &'static Signal<CriticalSectionRawMutex, Expression>,
    pub text_signal: &'static Signal<CriticalSectionRawMutex, Option<TextDisplay>>,
}

pub struct TextDisplay {
    pub text: String,
    pub position: Point,
    pub character_style: MonoTextStyle<'static, Color>,
    pub alignment: Alignment,
    pub duration: u64,
}

impl FaceConsumer {
    pub fn new(
        face: &'static Mutex<CriticalSectionRawMutex, Face>,
        expression_signal: &'static Signal<CriticalSectionRawMutex, Expression>,
        text_signal: &'static Signal<CriticalSectionRawMutex, Option<TextDisplay>>,
    ) -> Self {
        Self {
            face,
            expression_signal,
            text_signal,
        }
    }
}

/// Controller to provide the current expression
#[derive(Clone, Copy)]
pub struct FaceExpressionController {
    expression_signal: &'static Signal<CriticalSectionRawMutex, Expression>,
}

impl FaceExpressionController {
    pub fn new(expression_signal: &'static Signal<CriticalSectionRawMutex, Expression>) -> Self {
        Self { expression_signal }
    }

    pub fn signal(&self, value: Expression) {
        self.expression_signal.signal(value);
    }
}

/// Controller for the external bluetooth interface to
/// interact with the face over
pub struct FaceController {
    face: &'static Mutex<CriticalSectionRawMutex, Face>,
    mutex_guard: Option<MutexGuard<'static, CriticalSectionRawMutex, Face>>,
    text_signal: &'static Signal<CriticalSectionRawMutex, Option<TextDisplay>>,
}

impl Clone for FaceController {
    fn clone(&self) -> Self {
        Self {
            face: self.face,
            mutex_guard: None,
            text_signal: self.text_signal,
        }
    }
}

impl FaceController {
    pub fn new(
        face: &'static Mutex<CriticalSectionRawMutex, Face>,
        text_signal: &'static Signal<CriticalSectionRawMutex, Option<TextDisplay>>,
    ) -> Self {
        Self {
            face,
            mutex_guard: None,
            text_signal,
        }
    }
}

#[derive(defmt::Format)]
pub enum FaceControllerError {
    InvalidState,
    BeginExpression(BeginExpressionError),
    BeginFrame(BeginFrameError),
    PushPixels(PushPixelError),
}

impl FaceController {
    /// Send some text to be displayed
    pub fn write_text(&mut self, display: TextDisplay) {
        self.text_signal.signal(Some(display));
    }

    /// Clear the active text returning to displaying faces
    pub fn clear_text(&mut self) {
        self.text_signal.signal(None);
    }

    pub async fn begin_face(&mut self) {
        let face = match self.mutex_guard.as_mut() {
            Some(value) => value,
            None => {
                let guard = self.face.lock().await;
                self.mutex_guard.insert(guard)
            }
        };

        defmt::info!("beginning face");
        face.reset();
    }

    pub fn display_face(&mut self) {
        if self.mutex_guard.take().is_none() {
            defmt::warn!("called display_face before begin_face");
        } else {
            defmt::info!("displaying face");
        }
    }

    pub fn begin_expression(&mut self, expression: Expression) -> Result<(), FaceControllerError> {
        let face = self
            .mutex_guard
            .as_mut()
            .ok_or(FaceControllerError::InvalidState)?;
        defmt::info!("beginning expression {}", expression);

        face.begin_expression(expression)
            .map_err(FaceControllerError::BeginExpression)?;
        Ok(())
    }

    pub fn begin_frame(&mut self, duration: u8) -> Result<(), FaceControllerError> {
        let face = self
            .mutex_guard
            .as_mut()
            .ok_or(FaceControllerError::InvalidState)?;
        defmt::info!("beginning frame {}", duration);

        face.begin_frame(duration)
            .map_err(FaceControllerError::BeginFrame)?;
        Ok(())
    }

    pub fn push_pixels(&mut self, pixel_data: &[u8]) -> Result<(), FaceControllerError> {
        let face = self
            .mutex_guard
            .as_mut()
            .ok_or(FaceControllerError::InvalidState)?;
        defmt::info!("writing pixel data");
        let pixels = RLESlicePixelIterator::new(pixel_data);

        face.push_pixels(pixels)
            .map_err(FaceControllerError::PushPixels)?;
        Ok(())
    }
}

/// Task to render the face
#[task]
pub async fn face_render_task(face: FaceConsumer, mut driver: FaceMatrixDriver) {
    let mut expression = Expression::IDLE;

    let mut frame_index = 0;

    let FaceConsumer {
        face,
        expression_signal,
        text_signal,
    } = face;

    loop {
        // Brief delay to allow controller to acquire the lock and update the face
        Timer::after(Duration::from_millis(10)).await;

        // Handle expression changes
        if expression_signal.signaled() {
            let next_expression = expression_signal.wait().await;
            expression = next_expression;
        }

        if text_signal.signaled() {
            let mut signal: Option<TextDisplay> = text_signal.wait().await;
            while let Some(text) = signal.take() {
                driver.write_text(&text);
                driver = driver.swap().await;

                match select(
                    Timer::after(Duration::from_millis(text.duration)),
                    text_signal.wait(),
                )
                .await
                {
                    Either::First(_) => break,
                    Either::Second(next_signal) => signal = next_signal,
                }
            }

            // Erase the frame buffer before continuing
            driver.erase();
            driver = driver.swap().await;
        }

        let face = face.lock().await;
        let frames = match face.get_expression_frames(expression) {
            Some(value) if !value.is_empty() => value,

            // Nothing to render
            _ => continue,
        };

        // Wrap around frames
        if frame_index >= frames.len() {
            frame_index = 0;
        }

        let frame = &frames[frame_index];

        let pixels = match face.get_frame_pixels(frame) {
            Some(value) => value,
            // Nothing to render
            None => continue,
        };

        let pixel_producer = RLEPixelProducerIterator::new(pixels);
        driver.write_frame(pixel_producer);
        driver = driver.swap().await;

        let duration = frame.duration as u64;
        drop(face);
        Timer::after_millis(duration).await;
        frame_index += 1;
    }
}
