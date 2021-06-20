use stm32f4xx_hal as hal;

use hal::gpio;
use hal::prelude::*;
use hal::pwm;

const PWM_FREQ: f32 = 1_000_000.0;
const NOTE_TABLE: [u32; 12] = [
    (PWM_FREQ / 16.35160) as u32, // C0
    (PWM_FREQ / 17.32391) as u32,
    (PWM_FREQ / 18.35405) as u32,
    (PWM_FREQ / 19.44544) as u32,
    (PWM_FREQ / 20.60172) as u32,
    (PWM_FREQ / 21.82676) as u32,
    (PWM_FREQ / 23.12465) as u32,
    (PWM_FREQ / 24.49971) as u32,
    (PWM_FREQ / 25.95654) as u32,
    (PWM_FREQ / 27.50000) as u32,
    (PWM_FREQ / 29.13524) as u32,
    (PWM_FREQ / 30.86771) as u32,
];

pub struct Buzzer<T: pwm::C1> {
    pwm: pwm::Pwm<T>,
    pin: gpio::Pin<gpio::Output<gpio::PushPull>>,
}

impl<T: pwm::Instance> Buzzer<T> {
    pub fn new(pwm: T, mut pin: gpio::Pin<gpio::Output<gpio::PushPull>>) -> Self {
        let pwm = pwm::Pwm::new(pwm);
        pwm.set_prescaler(pwm::Prescaler::Div16);
        pwm.set_output_pin(pwm::Channel::C0, &pin);
        pwm.set_counter_mode(pwm::CounterMode::Up);
        pwm.disable();
        pin.set_low().unwrap();

        Self { pwm, pin }
    }

    // 0 = C0, goes up by semitone from there. For example, A4 is 57
    pub fn on(&mut self, pitch: u8) {
        let pitch = pitch as usize;
        let max_duty = NOTE_TABLE[pitch % 12] >> (pitch / 12);
        self.pwm.enable();
        self.pwm.set_max_duty(max_duty as u16);
        self.pwm.set_duty(pwm::Channel::C0, (max_duty / 2) as u16);
    }

    pub fn off(&mut self) {
        self.pwm.disable();
        self.pin.set_low().unwrap();
    }
}
