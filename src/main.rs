#![windows_subsystem = "windows"] //console ma boi

pub mod dulum;
pub mod meth;
pub mod mouse;

use std::collections::VecDeque;

use dulum::Dulum;
use egui::plot::{PlotPoints, Line};
use macroquad::{
    hash,
    prelude::*,
    ui::{root_ui, widgets},
};
use nalgebra::{DMatrix, DVector};

use crate::{
    meth::{deg2rad, normalize_angle, rad2deg},
    mouse::MouseMovement,
};

const DULUMS_COLORS: [Color; 6] = [
    color_u8!(255, 0, 0, 255),
    color_u8!(255, 255, 0, 255),
    color_u8!(0, 255, 0, 255),
    color_u8!(0, 255, 255, 255),
    color_u8!(0, 0, 255, 255),
    color_u8!(255, 0, 255, 255),
];

fn accumulate_jacobi(dulums: &[Dulum]) -> DMatrix<f64> {
    let count = dulums.len();

    let columns = dulums
        .iter()
        .enumerate()
        .flat_map(|(id, x)| {
            let (j_vec_1, j_vec_2) = x.get_jacobi_vectors(id, count);
            [Some(j_vec_1), j_vec_2]
        })
        .filter_map(|x| x)
        .collect::<Vec<_>>();

    DMatrix::from_columns(columns.as_slice())
}

fn accumulate_constraint(dulums: &[Dulum]) -> DMatrix<f64> {
    let count = dulums.len();

    let columns = dulums
        .iter()
        .enumerate()
        .flat_map(|(id, x)| {
            let (j_vec_1, j_vec_2) = x.get_partial_constraint(id, count);
            [Some(j_vec_1), j_vec_2]
        })
        .filter_map(|x| x)
        .collect::<Vec<_>>();

    let partial_constraint = DMatrix::from_columns(columns.as_slice());

    let elements: Vec<_> = dulums
        .iter()
        .flat_map(|x| {
            let (a, b) = x.get_coordinates_der();
            [Some(a), b]
        })
        .filter_map(|x| x)
        .collect();

    let derivatives = DMatrix::from_vec(elements.len(), 1, elements);

    partial_constraint * derivatives
}

pub fn accumulate_mass(dulums: &[Dulum]) -> DMatrix<f64> {
    DMatrix::from_diagonal(&DVector::from_iterator(
        dulums.len() * 2,
        dulums.iter().flat_map(|x| [x.mass, x.mass]),
    ))
}

pub fn accumulate_hooks_force(dulums: &[Dulum]) -> DMatrix<f64> {
    let elements: Vec<_> = dulums
        .iter()
        .flat_map(|x| {
            let (a, b) = x.hooks_force();
            [Some(a), b]
        })
        .filter_map(|x| x)
        .collect();

    DMatrix::from_vec(elements.len(), 1, elements)
}

pub fn accumulate_gravity(dulums: &[Dulum]) -> DMatrix<f64> {
    let elements: Vec<_> = dulums.iter().flat_map(|_| [0.0, 9.8]).collect();

    DMatrix::from_vec(elements.len(), 1, elements)
}

#[macroquad::main("Multiple Pendulums")]
async fn main() {
    //camera states
    let mut camera_scale = 70.0;
    let mut camera_origin = vec2(0.0, 0.0);

    let mut mouse = MouseMovement::new();
    //prepare state
    let mut dulums = vec![
        Dulum::new(0.0, 2.0, 1.0, false, 100.0, 2.0, DULUMS_COLORS[0], 0.2),
        Dulum::new(0.0, 2.0, 1.0, false, 100.0, 2.0, DULUMS_COLORS[1], 0.2),
    ];

    let mut time_step: f32 = 0.001;
    //za warudo
    let mut simulate: bool = false;
    let mut time_budget: f32 = 0.0;

    //energy trails 
    let mut energy_trail: VecDeque<f64> = VecDeque::with_capacity(1024);

    loop {
        mouse.update();
        //camera input handling
        if !root_ui().is_mouse_captured() {
            let norm_mouse = if mouse_wheel().1.abs() <= 0.01 {
                0.0
            } else {
                mouse_wheel().1.signum()
            };

            camera_scale *= 1.05f32.powf(norm_mouse);
            if is_mouse_button_down(MouseButton::Right) {
                camera_origin.x -= mouse.dx / camera_scale;
                camera_origin.y -= mouse.dy / camera_scale;
            }
        }
        if is_key_down(KeyCode::Up) {
            camera_scale *= 3.00f32.powf(get_frame_time());
        }
        if is_key_down(KeyCode::Down) {
            camera_scale /= 3.00f32.powf(get_frame_time());
        }
        //step the dulums
        if simulate {
            //add to the time budget
            time_budget += get_frame_time();
            //snow balling protection 
            if time_budget >= 1.0 {
                time_budget = 0.0;
                simulate = false;
            }

            while time_budget >= time_step {
                time_budget -= time_step;
                //gain variables
                let jacobi = accumulate_jacobi(&dulums);
                let jacobi_trans = jacobi.transpose();
                let mass = accumulate_mass(&dulums);
                let constraint = accumulate_constraint(&dulums);
                let hooks = accumulate_hooks_force(&dulums);
                let gravity = accumulate_gravity(&dulums);

                //calculate sides
                let left = jacobi_trans.clone() * mass.clone() * jacobi;
                let Some(left) = left.try_inverse() else {
                    panic!("If no inverse, no working!");
                };
                let right = hooks + jacobi_trans * mass.clone() * (gravity.clone() - constraint);

                let shit = left * right;
                //extract values
                let mut pointer = 0;
                for dulum in &mut dulums {
                    if dulum.is_elastic() {
                        dulum.leapfrog_part_one(time_step as f64, shit[pointer], shit[pointer + 1]);
                        pointer += 2;
                    } else {
                        dulum.leapfrog_part_one(time_step as f64, shit[pointer], 0.0);
                        pointer += 1;
                    }
                }
                //do it once again, but with newer positions (and velocities, oops)
                //gain variables
                let jacobi = accumulate_jacobi(&dulums);
                let jacobi_trans = jacobi.transpose();
                let constraint = accumulate_constraint(&dulums);
                let hooks = accumulate_hooks_force(&dulums);

                //calculate sides
                let left = jacobi_trans.clone() * mass.clone() * jacobi;
                let Some(left) = left.try_inverse() else {
                    panic!("If no inverse, no working!");
                };
                let right = hooks + jacobi_trans * mass * (gravity - constraint);

                let shit = left * right;
                //extract values
                let mut pointer = 0;
                for dulum in &mut dulums {
                    if dulum.is_elastic() {
                        dulum.leapfrog_part_two(time_step as f64, shit[pointer], shit[pointer + 1]);
                        pointer += 2;
                    } else {
                        dulum.leapfrog_part_two(time_step as f64, shit[pointer], 0.0);
                        pointer += 1;
                    }
                }
            }
        }

        clear_background(BLACK);

        set_camera(&Camera2D {
            rotation: 0.0,
            zoom: vec2(
                camera_scale / screen_width(),
                camera_scale / screen_height(),
            ),
            target: camera_origin,
            offset: vec2(0.0, 0.0),
            render_target: None,
            viewport: None,
        });

        //render the dulums
        //trails
        let mut previous_x: f32 = 0.0;
        let mut previous_y: f32 = 0.0;
        for dulum in &mut dulums {
            (previous_x, previous_y) = dulum.add_trail(previous_x, previous_y);
            dulum.render_trail();
        }
        (previous_x, previous_y) = (0.0, 0.0);
        //lines
        for dulum in &dulums {
            (previous_x, previous_y) = dulum.render_line(previous_x, previous_y);
        }

        //mass
        (previous_x, previous_y) = (0.0, 0.0);

        for dulum in &dulums {
            (previous_x, previous_y) = dulum.render_circle(previous_x, previous_y);
        }

        //egui
        set_default_camera();

        egui_macroquad::ui(|egui_ctx| {
            egui::Window::new("Simulation controls").show(egui_ctx, |ui| {
                //simulace?
                ui.add(egui::Checkbox::new(&mut simulate, "Simulate"));
                //time step size
                ui.horizontal(|ui| {
                    ui.label("Step size");
                    ui.add(
                        egui::Slider::new(&mut time_step, 0.0..=1.0)
                            .logarithmic(true)
                    );
                });
                //number of dulums
                let mut expected_dulums = dulums.len();
                ui.horizontal(|ui| {
                    ui.label("Number of Dulums");
                    ui.add(egui::Slider::new(&mut expected_dulums, 1..=6));
                });
                if ui.button("Reset").clicked() {
                    dulums.clear();
                }
                //correct number of dulums
                if expected_dulums < dulums.len() {
                    dulums.truncate(expected_dulums);
                }
                if expected_dulums > dulums.len() {
                    for _ in 0..(expected_dulums - dulums.len()) {
                        let color = DULUMS_COLORS[dulums.len() % DULUMS_COLORS.len()];
                        dulums.push(Dulum::new(0.0, 2.0, 1.0, false, 100.0, 2.0, color, 0.2));
                    }
                }

                //ovládání pro dula
                for (ind, dulum) in dulums.iter_mut().enumerate() {
                    egui::CollapsingHeader::new(format!("Dulum #{}", ind + 1)).show(ui, |ui| {
                        //dulum's angle
                        ui.horizontal(|ui| {
                            ui.label("Angle");
                            meth::drag_angle(ui, &mut dulum.angle)
                        });
                        ui.horizontal(|ui| {
                            ui.label("Angle Der");
                            meth::drag_angle(ui, &mut dulum.angle_der)
                        });

                        //dulum's length
                        ui.horizontal(|ui| {
                            ui.label("Length");
                            ui.add(egui::DragValue::new(&mut dulum.len));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Length Der");
                            ui.add(egui::DragValue::new(&mut dulum.len_der));
                        });

                        ui.horizontal(|ui| {
                            ui.label("Mass");
                            ui.add(egui::DragValue::new(&mut dulum.mass));
                        });

                        //elastic
                        ui.checkbox(&mut dulum.elastic, "Elastic");
                        ui.checkbox(&mut dulum.push_elastic, "Push elastic");

                        //hardness
                        ui.horizontal(|ui| {
                            ui.label("Hardness");
                            ui.add(egui::DragValue::new(&mut dulum.hardness));
                        });

                        ui.horizontal(|ui| {
                            ui.label("Default Len");
                            ui.add(egui::DragValue::new(&mut dulum.default_len));
                        });

                        //visibility
                        ui.checkbox(&mut dulum.visible, "Show dulum");
                        ui.checkbox(&mut dulum.visible_line, "Show line");
                        ui.checkbox(&mut dulum.visible_trace, "Show trace");
                    });
                }

                //energy handling
                egui::CollapsingHeader::new("Energy")
                    .show(ui, |ui| {

                        let points: PlotPoints = energy_trail.iter()
                            .enumerate()
                            .map(|(x, y)| [x as f64, *y])
                            .collect();

                        let line = Line::new(points);

                        egui::plot::Plot::new("Total Energy")
                            .view_aspect(2.0)
                            .show(ui, |x| {
                                x.line(line)
                        });


                        let mut x = 0.0;
                        let mut y = 0.0;
                        let mut ultra_total = 0.0;

                        for (ind, dulum) in dulums.iter().enumerate() {
                            let pot_grav = dulum.potential_gravity_energy(x, y);
                            x = pot_grav.1;
                            y = pot_grav.2;

                            let pot_elas = dulum.potential_elastic_energy();
                            let kinet = dulum.kinetic_energy();

                            let total = pot_grav.0 + pot_elas + kinet;
                            ultra_total += total;

                            egui::CollapsingHeader::new(format!("Dulum {}", ind))   
                                .show(ui, |ui| {
                                    ui.label(format!("Pot Grav: {:.2}", pot_grav.0).as_str());
                                    ui.label(format!("Pot Elas: {:.2}", pot_elas).as_str());
                                    ui.label(format!("En Kinet: {:.2}", kinet).as_str());
                                    ui.label(format!("Total: {:.2}", total).as_str());
                            });
                        }

                        //přidej ultra total 
                        if energy_trail.len() >= 1024 {
                            energy_trail.pop_front();
                        }
                        energy_trail.push_back(ultra_total);

                    })
            });
        });

        egui_macroquad::draw();

        next_frame().await;
    }
}
