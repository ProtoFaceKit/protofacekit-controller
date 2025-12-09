use crate::{
    data::Expression,
    face_control::{EXPRESSION_TICK_MS, FaceExpressionController},
};
use embassy_executor::task;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    Blocking,
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    peripherals::{ADC2, GPIO11},
};

type MicPin = GPIO11<'static>;
type MicAdcPin = ADC2<'static>;

// Calibration constants
const DC_CENTER_SAMPLES: usize = 300;
const DC_CENTER_SMOOTHING: f32 = 0.99;
const DC_CENTER_NEW_WEIGHT: f32 = 0.01;

const NOISE_FLOOR_SAMPLES: usize = 500;
const NOISE_FLOOR_SMOOTHING: f32 = 0.99;
const NOISE_FLOOR_NEW_WEIGHT: f32 = 0.01;
const NOISE_FLOOR_SAMPLE_INTERVAL_MS: u64 = 1;

// Speech detection thresholds (multipliers of noise peak)
const SPEECH_START_MULTIPLIER: f32 = 4.0;
const SPEECH_STOP_MULTIPLIER: f32 = 2.0;

// Peak envelope detector constants
const PEAK_ATTACK_SMOOTH: f32 = 0.7;
const PEAK_ATTACK_NEW: f32 = 0.3;
const PEAK_RELEASE_SMOOTH: f32 = 0.995;
const PEAK_RELEASE_NEW: f32 = 0.005;

// Main loop timing
const ADC_READ_RETRY_MS: u64 = 1;
const DETECTION_LOOP_INTERVAL_MS: u64 = 2;

// Read the next value from the mic pins
async fn read_mic_pin(
    adc: &mut Adc<'static, MicAdcPin, Blocking>,
    pin: &mut AdcPin<MicPin, MicAdcPin>,
) -> f32 {
    loop {
        match adc.read_oneshot(pin) {
            Ok(v) => return v as f32,
            Err(nb::Error::WouldBlock) => {
                Timer::after(Duration::from_millis(ADC_READ_RETRY_MS)).await;
                continue;
            }
            Err(_) => unreachable!("should never return any other error type"),
        }
    }
}

/// Calibration data to use for microphone input detection
struct MicCalibration {
    dc_center: f32,
    start_threshold: f32,
    stop_threshold: f32,
    noise_peak: f32,
}

/// Calibrate the mic based on the current environment
async fn calibrate_mic(
    adc: &mut Adc<'static, MicAdcPin, Blocking>,
    pin: &mut AdcPin<MicPin, MicAdcPin>,
) -> MicCalibration {
    // Detect DC center (MAX981 output midpoint)
    let mut dc_center = 0.0;
    for _ in 0..DC_CENTER_SAMPLES {
        let raw_value = read_mic_pin(adc, pin).await;
        dc_center = dc_center * DC_CENTER_SMOOTHING + raw_value * DC_CENTER_NEW_WEIGHT;
    }

    // Calibrate noise floor using peak detector
    let mut noise_peak = 0.0;
    for _ in 0..NOISE_FLOOR_SAMPLES {
        let raw_value = read_mic_pin(adc, pin).await;
        let ac_amplitude = (raw_value - dc_center).abs();
        noise_peak = noise_peak * NOISE_FLOOR_SMOOTHING + ac_amplitude * NOISE_FLOOR_NEW_WEIGHT;
        Timer::after(Duration::from_millis(NOISE_FLOOR_SAMPLE_INTERVAL_MS)).await;
    }

    let start_threshold = noise_peak * SPEECH_START_MULTIPLIER;
    let stop_threshold = noise_peak * SPEECH_STOP_MULTIPLIER;

    MicCalibration {
        dc_center,
        noise_peak,
        start_threshold,
        stop_threshold,
    }
}

#[task]
pub async fn microphone_expression_task(
    mic_pin: MicPin,
    adc_pin: MicAdcPin,
    expression_controller: FaceExpressionController,
) {
    // ADC setup
    let mut adc2_config = AdcConfig::new();
    let mut pin = adc2_config.enable_pin(mic_pin, Attenuation::_11dB);
    let mut adc = Adc::new(adc_pin, adc2_config);

    // Perform calibration
    let MicCalibration {
        dc_center,
        noise_peak,
        start_threshold,
        stop_threshold,
    } = calibrate_mic(&mut adc, &mut pin).await;

    defmt::info!(
        "microphone calibrated: DC={} noise={} start={} stop={}",
        dc_center,
        noise_peak,
        start_threshold,
        stop_threshold
    );

    let mut speaking = false;
    let mut peak_envelope = 0.0;
    let mut last_signal = Instant::now();

    // Sound detection loop
    loop {
        let raw = read_mic_pin(&mut adc, &mut pin).await;

        // Calculate AC amplitude (signal minus DC offset)
        let ac_amplitude = (raw - dc_center).abs();

        // Peak envelope detector (fast attack, slow release)
        if ac_amplitude > peak_envelope {
            // Fast attack when signal increases
            peak_envelope = PEAK_ATTACK_SMOOTH * peak_envelope + PEAK_ATTACK_NEW * ac_amplitude;
        } else {
            // Slow release when signal decreases
            peak_envelope = PEAK_RELEASE_SMOOTH * peak_envelope + PEAK_RELEASE_NEW * ac_amplitude;
        }

        if speaking {
            // End speaking detection
            if peak_envelope < stop_threshold {
                speaking = false;
                defmt::info!("Speech END. peak={}", peak_envelope);
            }

            // If 50ms has elapsed since the last expression signal pulse the talking signal
            if last_signal.elapsed().as_millis() > EXPRESSION_TICK_MS {
                expression_controller.signal(Expression::Talking);
                last_signal = Instant::now();
            }
        } else {
            // Begin speaking when over threshold
            if peak_envelope > start_threshold {
                speaking = true;
                defmt::info!("Speech START. peak={}", peak_envelope);

                expression_controller.signal(Expression::Talking);
                last_signal = Instant::now();
            }
        }

        // Throttle loop to prevent blocking other tasks
        Timer::after(Duration::from_millis(DETECTION_LOOP_INTERVAL_MS)).await;
    }
}
