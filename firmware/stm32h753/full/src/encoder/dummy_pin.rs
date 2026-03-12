use core::marker::PhantomData;
use embedded_hal::digital::v2::OutputPin;

// Usage:
// let dummy_cs = DummyPin::new();

// No-op dummy pin
pub struct DummyPin {
    _marker: PhantomData<*const ()>,
}

impl DummyPin {
    pub fn new() -> Self {
        DummyPin {
            _marker: PhantomData,
        }
    }
}

impl OutputPin for DummyPin {
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(()) // Do nothing
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(()) // Do nothing
    }
}
