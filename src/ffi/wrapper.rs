pub mod wrapper {
    use crate::ffi::raw;

    #[derive(Debug)]
    pub enum CompassError {
        InvalidSensorData, // e.g. NaN or Infinity
        SaturationLimit,   // e.g. values exceeding expected mT
    }

    pub fn safe_calc_heading_in_rad(mx: i16, my: i16) -> Result<f32, CompassError> {
        // Ensure values aren't stuck at sensor limits (possible hardware failure)
        if mx == i16::MIN || mx == i16::MAX {
            return Err(CompassError::SaturationLimit);
        }

        // The Unsafe Boundary
        let result = unsafe { raw::calc_heading_in_rad(mx, my) };

        // Post-Condition Check
        if result.is_nan() {
            return Err(CompassError::InvalidSensorData);
        }

        Ok(result)
    }
}
