use crate::ffi::raw;

pub fn safe_calc_heading_in_rad(mx: i16, my: i16) -> f32 {
    unsafe { raw::calc_heading_in_rad(mx, my) }
}
