
pub fn deg2rad(deg: f64) -> f64 {
    std::f64::consts::PI / 180.0 * deg
}

pub fn rad2deg(rad: f64) -> f64 {
    180.0 / std::f64::consts::PI * rad
}

pub fn normalize_angle(ang: f64) -> f64 {
    let temp = ang % (std::f64::consts::PI * 2.0);
    
    if temp < 0.0 {
        temp + std::f64::consts::PI * 2.0
    }else{
        temp
    }
}

