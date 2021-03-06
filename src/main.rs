//! Space Habitiats: a sphere packing method to identify optimal layouts of deep space exploration habitats.
//!
//! Based on ideas discussed in:
//!
//! Valentina Sumini and Caitlin Mueller, Proceedings of the International Association for Shell and Spatial Structures (IASS) Symposium, 2017

#![feature(vec_remove_item)]

extern crate kiss3d;
extern crate nalgebra;
extern crate rand;

/// Handles any errors that could occur during habitat optimisation.
mod errors;
/// Implementation of a modified version of the advancing front packing algorithm from
/// Valera *et al.*, [Computational Particle Mechanics 2, 161 (2015)](https://doi.org/10.1007/s40571-015-0045-8).
mod sphere_packing;

use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::{Point3, Translation3};
use rand::distributions::Uniform;
use rand::prelude::*;
use sphere_packing::{pack_spheres, Sphere};
use std::collections::VecDeque;

// For testing random spheres or using autorotation of the camera
//use nalgebra::UnitQuaternion;
//use std::iter::repeat;

#[derive(Debug)]
/// Sample room structure for testing. May be removed or significantly altered in the future.
struct Room {
    /// Name/function of the room in the habitat.
    name: String,
    /// Room size, since it will be a sphere this is measured via its `radius`.
    /// Test conditions are mostly less than one (for example, Sleep is 0.7)
    /// to fit within some camera value, but correspond to real values in meters.
    /// Sleep in reality should be 700 m.
    radius: f32,
    /// A color to distinguish the room in the visualisation.
    color: Color,
    /// Location in space of current room.
    position: Point3<f32>,
}

impl Room {
    /// Generates a `new` room based on user generated data.
    fn new(name: &str, radius: f32, color: Color, position: Point3<f32>) -> Room {
        Room {
            name: name.to_string(),
            radius,
            color,
            position,
        }
    }
}

#[derive(Debug)]
/// Simple representation of an rgb color. Kiss3D requires values on $[0.0, 1.0]$.
struct Color {
    red: f32,
    green: f32,
    blue: f32,
}

impl Color {
    /// Constructs a `new` color based on rgb values between 0 and 1.
    fn new(red: f32, green: f32, blue: f32) -> Color {
        Color { red, green, blue }
    }
}

/// Makes sure randomly added rooms to not lie inside or overlap with another room. Only needed for testing.
fn find_disjoint_position(
    from: &[Room],
    distance: f32,
    between: Uniform<f32>,
    rng: &mut ThreadRng,
) -> Point3<f32> {
    let mut new_point = Point3::new(
        rng.sample(between),
        rng.sample(between),
        rng.sample(between),
    );
    loop {
        if from
            .iter()
            .any(|ref r| nalgebra::distance(&new_point, &r.position) < distance)
        {
            new_point = Point3::new(
                rng.sample(between),
                rng.sample(between),
                rng.sample(between),
            );
        } else {
            return new_point;
        }
    }
}

/// Simple room constructor, currently hardcoded for testing and will most likely be removed in the future.
fn construct_rooms() -> Vec<Room> {
    let mut rng = thread_rng();
    let between = Uniform::new(-1.5, 1.5);

    //For now, this is hardcoded since it's known, but later this will need to be identified first
    let max_distance = 1.2; //Radius of Sleep + Leisure

    let mut rooms = Vec::new();
    let mut new_position = Point3::new(
        rng.sample(between),
        rng.sample(between),
        rng.sample(between),
    );

    rooms.push(Room::new(
        "Sleep",
        0.7,
        Color::new(0.8941, 0.1020, 0.1098),
        new_position,
    ));

    new_position = find_disjoint_position(&rooms, max_distance, between, &mut rng);
    rooms.push(Room::new(
        "Leisure",
        0.5,
        Color::new(0.2157, 0.4941, 0.7216),
        new_position,
    ));

    new_position = find_disjoint_position(&rooms, max_distance, between, &mut rng);
    rooms.push(Room::new(
        "Food",
        0.35,
        Color::new(0.3020, 0.6863, 0.2902),
        new_position,
    ));

    new_position = find_disjoint_position(&rooms, max_distance, between, &mut rng);
    rooms.push(Room::new(
        "Work/Lab",
        0.35,
        Color::new(0.5961, 0.3059, 0.6392),
        new_position,
    ));

    new_position = find_disjoint_position(&rooms, max_distance, between, &mut rng);
    rooms.push(Room::new(
        "Greenhouse",
        0.25,
        Color::new(1.0, 0.4980, 0.0),
        new_position,
    ));

    new_position = find_disjoint_position(&rooms, max_distance, between, &mut rng);
    rooms.push(Room::new(
        "Sport",
        0.4,
        Color::new(1.0, 1.0, 0.2),
        new_position,
    ));
    rooms
}

/// Program entry point.
fn main() {
    let eye = Point3::new(3.0, 3.0, 3.0);
    let at = Point3::origin();
    let mut camera = ArcBall::new(eye, at);

    let mut window = Window::new_with_size("Surface Habitats", 1920, 1080);
    window.set_light(Light::StickToCamera);

    //let habitat = construct_rooms();
    //for room in habitat.iter() {
    //    let mut draw_room = window.add_sphere(room.radius);
    //    draw_room.set_color(room.color.red, room.color.green, room.color.blue);
    //    draw_room.set_local_translation(Translation3::from(room.position.coords));
    //}

    //---------------------------------
    //let between = rng.gen_range(0.1, 0.2);
    //let mut rng = thread_rng();
    //let radii = repeat(between.ind_sample(&mut rng)).take(10000).collect::<VecDeque<f32>>();

    let boundary = Sphere::new(at, 3.0).unwrap();
    let radii: VecDeque<f32> = vec![0.7, 0.5, 0.35, 0.35, 0.25, 0.4].into_iter().collect();

    let spheres = pack_spheres(&boundary, radii).unwrap();
    for sphere in spheres.iter() {
        let mut draw_room = window.add_sphere(sphere.radius);
        draw_room.set_color(random(), random(), random());
        draw_room.set_local_translation(Translation3::from(sphere.center.coords));
    }

    //---------------------------------

    while !window.should_close() {
        window.render_with_camera(&mut camera);
    }
    //let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
    //while window.render_with_camera(&mut camera) {
    //window.scene_mut().prepend_to_local_rotation(&rot);
    //}
}
