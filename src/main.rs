#![windows_subsystem = "windows"] //console ma boi

pub mod dulum;
pub mod meth;
pub mod mouse;

use dulum::Dulum;
use macroquad::{prelude::*, ui::{root_ui, widgets}, hash};
use nalgebra::{DMatrix, DVector};

use crate::{meth::{deg2rad, rad2deg, normalize_angle}, mouse::MouseMovement};

const DULUMS_COLORS: [Color; 6] = [
    color_u8!(255,   0,   0, 255),
    color_u8!(255, 255,   0, 255),
    color_u8!(  0, 255,   0, 255),
    color_u8!(  0, 255, 255, 255),
    color_u8!(  0,   0, 255, 255),
    color_u8!(255,   0, 255, 255),
];

fn accumulate_jacobi(dulums: &[Dulum]) -> DMatrix<f64> {
    let count = dulums.len();

    let columns = 
        dulums.iter()
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

    let columns = 
        dulums.iter()
            .enumerate()
            .flat_map(|(id, x)| {
                let (j_vec_1, j_vec_2) = x.get_partial_constraint(id, count);
                [Some(j_vec_1), j_vec_2]
            })
            .filter_map(|x| x)
            .collect::<Vec<_>>();

    let partial_constraint = DMatrix::from_columns(columns.as_slice());

    let elements: Vec<_> = dulums.iter()
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
    DMatrix::from_diagonal(
        &DVector::from_iterator(dulums.len() * 2,
            dulums.iter()
                .flat_map(|x| [x.mass, x.mass])
        )
    )
}

pub fn accumulate_hooks_force(dulums: &[Dulum]) -> DMatrix<f64> {
    let elements: Vec<_> = dulums.iter()
        .flat_map(|x| {
            let (a, b) = x.hooks_force();
            [Some(a), b]
        })
        .filter_map(|x| x)
        .collect();

    DMatrix::from_vec(elements.len(), 1, elements)
}

pub fn accumulate_gravity(dulums: &[Dulum]) -> DMatrix<f64> {
    let elements: Vec<_> = dulums.iter()
        .flat_map(|_| {
            [0.0, 9.8]
        })
        .collect();

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
        Dulum::new(0.0, 2.0, 1.0, false, 100.0, 1.0, DULUMS_COLORS[0], 0.2),
        Dulum::new(0.0, 2.0, 1.0, false, 100.0, 1.0, DULUMS_COLORS[1], 0.2)
    ];

    const TIME_STEP: f32 = 0.001;
    //za warudo
    let mut simulate: bool = false;
    //window status
    let mut hide_window = true;

    loop {
        mouse.update();
        //camera input handling
        if !root_ui().is_mouse_captured() {
            let norm_mouse = if mouse_wheel().1.abs() <= 0.01 {
                0.0
            }else{
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
            let step_count = (get_frame_time() / TIME_STEP) as usize;
            for _ in 0..step_count {
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
                let right = hooks + jacobi_trans * mass * (
                    gravity - constraint
                );

                let shit = left * right;
                //extract values
                let mut pointer = 0;
                for dulum in &mut dulums {
                    if dulum.is_elastic() {
                        dulum.move_state(TIME_STEP as f64, shit[pointer], shit[pointer + 1]);
                        pointer += 2;
                    }else{
                        dulum.move_state(TIME_STEP as f64, shit[pointer], 0.0);
                        pointer += 1;
                    }
                }
            }
        }

        clear_background(BLACK);

        set_camera(&Camera2D{ 
            rotation: 0.0, 
            zoom: vec2(camera_scale / screen_width(), camera_scale / screen_height()),
            target: camera_origin, 
            offset: vec2(0.0, 0.0),
            render_target: None,
            viewport: None
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

        //GUI COMES HERE
        //uncomment for screen space
        //set_default_camera(); 

        next_frame().await;
    }
}
