use std::collections::VecDeque;

use iterwindows::IterArrayWindows;
use macroquad::prelude::*;
use nalgebra::DVector;

#[derive(Clone)]
pub struct Dulum {
    pub visible: bool,
    pub visible_line: bool,
    pub visible_trace: bool,

    pub angle: f64,
    pub len: f64,
    pub default_len: f64,
    pub mass: f64,

    pub angle_der: f64,
    pub len_der: f64,

    pub elastic: bool,
    pub push_elastic: bool,
    pub hardness: f64,

    pub color: Color,
    pub size: f32,

    trail: VecDeque<(f32, f32)>,
}

impl Dulum {
    pub fn new(
        angle: f64,
        len: f64,
        mass: f64,
        elastic: bool,
        hardness: f64,
        default_len: f64,
        color: Color,
        size: f32,
    ) -> Self {
        Self {
            visible: true,
            visible_line: true,
            visible_trace: true,
            angle,
            len,
            mass,
            angle_der: 0.0,
            len_der: 0.0,
            elastic,
            push_elastic: false,
            color,
            size,
            trail: VecDeque::with_capacity(1024),
            hardness,
            default_len,
        }
    }

    pub fn render_line(&self, previous_x: f32, previous_y: f32) -> (f32, f32) {
        let x = previous_x + (self.len * self.angle.sin()) as f32;
        let y = previous_y + (self.len * self.angle.cos()) as f32;

        if self.visible_line {
            draw_line(previous_x, previous_y, x, y, 0.1, ORANGE);
        }

        (x, y)
    }

    pub fn render_circle(&self, previous_x: f32, previous_y: f32) -> (f32, f32) {
        let x = previous_x + (self.len * self.angle.sin()) as f32;
        let y = previous_y + (self.len * self.angle.cos()) as f32;

        if !self.visible {
            return (x, y);
        }

        draw_circle(x, y, self.size, self.color);

        (x, y)
    }
}

//meth part
impl Dulum {
    pub fn get_jacobi_vectors(
        &self,
        id: usize,
        count: usize,
    ) -> (DVector<f64>, Option<DVector<f64>>) {
        //create vectors
        let mut angle_vector = DVector::zeros(count * 2);
        let mut len_vector = if self.elastic {
            Some(DVector::zeros(count * 2))
        } else {
            None
        };
        //vyplň je
        let ang_sin = self.angle.sin();
        let ang_cos = self.angle.cos();
        let len_cos = self.len * ang_cos;
        let len_sin = -self.len * ang_sin;

        for i in id..count {
            //fill angle_vector
            angle_vector[i * 2] = len_cos;
            angle_vector[i * 2 + 1] = len_sin;
            //fill len vector
            if self.elastic {
                //SAFETY: is some if elastic which we checked
                let len_ref = unsafe { len_vector.as_mut().unwrap_unchecked() };
                len_ref[i * 2] = ang_sin;
                len_ref[i * 2 + 1] = ang_cos;
            }
        }

        (angle_vector, len_vector)
    }

    pub fn get_coordinates(&self) -> (f64, Option<f64>) {
        (self.angle, if self.elastic { Some(self.len) } else { None })
    }

    pub fn get_coordinates_der(&self) -> (f64, Option<f64>) {
        (
            self.angle_der,
            if self.elastic {
                Some(self.len_der)
            } else {
                None
            },
        )
    }

    pub fn get_partial_constraint(
        &self,
        id: usize,
        count: usize,
    ) -> (DVector<f64>, Option<DVector<f64>>) {
        //vytvoř vektory
        let mut angle_constraint = DVector::zeros(count * 2);
        let mut len_constraint = if self.elastic {
            Some(DVector::zeros(count * 2))
        } else {
            None
        };

        //vyplň vektory
        let angle_sin = self.angle.sin();
        let angle_cos = self.angle.cos();
        let len_sin = -self.len * angle_sin * self.angle_der;
        let len_cos = -self.len * angle_cos * self.angle_der;

        for i in id..count {
            //fill angle_constraint
            if self.elastic {
                angle_constraint[i * 2] = len_sin + angle_cos * self.len_der;
                angle_constraint[i * 2 + 1] = len_cos - angle_sin * self.len_der;
            } else {
                angle_constraint[i * 2] = len_sin;
                angle_constraint[i * 2 + 1] = len_cos;
            }
            //fill len_constraint
            if self.elastic {
                //SAFE: len_constraint only exists if elastic which we checked
                let len_ref = unsafe { len_constraint.as_mut().unwrap_unchecked() };
                len_ref[i * 2] = angle_cos * self.angle_der;
                len_ref[i * 2 + 1] = -angle_sin * self.angle_der;
            }
        }

        (angle_constraint, len_constraint)
    }

    pub fn hooks_force(&self) -> (f64, Option<f64>) {
        (
            0.0,
            if self.elastic {
                if self.push_elastic && self.len < self.default_len{
                    Some(0.0)
                }else{
                    Some(-self.hardness * (self.len - self.default_len))
                }
            } else {
                None
            },
        )
    }
}

//steppin and gettin
impl Dulum {
    pub fn move_state(&mut self, dt: f64, angle_der_der: f64, len_der_der: f64) {
        //pokud není elastický, vymaž len_der
        if !self.elastic {
            self.len_der = 0.0;
        }
        //split in half for reasons
        self.angle_der += angle_der_der * dt / 2.0;
        self.len_der += len_der_der * dt / 2.0;

        //increase only by half
        self.angle += self.angle_der * dt;
        self.len += self.len_der * dt;

        //put back the second half
        self.angle_der += angle_der_der * dt / 2.0;
        self.len_der += len_der_der * dt / 2.0;
    }

    pub fn add_trail(&mut self, previous_x: f32, previous_y: f32) -> (f32, f32) {
        if self.trail.len() >= 1024 {
            self.trail.pop_front();
        }

        let x = previous_x + (self.len * self.angle.sin()) as f32;
        let y = previous_y + (self.len * self.angle.cos()) as f32;

        self.trail.push_back((x, y));
        (x, y)
    }

    pub fn render_trail(&self) {
        if !self.visible_trace {
            return;
        }

        for [(ind, prev), (_, now)] in self.trail.iter().enumerate().array_windows() {
            let ratio = ind as f32 / self.trail.len() as f32;
            let color = Color {
                r: self.color.r * ratio,
                g: self.color.g * ratio,
                b: self.color.b * ratio,
                a: self.color.a * ratio,
            };
            draw_line(prev.0, prev.1, now.0, now.1, 0.1, color)
        }
    }

    pub fn is_elastic(&self) -> bool {
        self.elastic
    }
}
