use egui::Ui;

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
    } else {
        temp
    }
}

pub fn drag_angle(ui: &mut Ui, radians: &mut f64) -> egui::Response {
    let mut degrees = rad2deg(*radians);
    let mut response = ui.add(egui::DragValue::new(&mut degrees).speed(1.0).suffix("°"));

    // only touch `*radians` if we actually changed the degree value
    if degrees != rad2deg(*radians) {
        *radians = deg2rad(degrees);
        response.changed = true;
    }

    response
}
