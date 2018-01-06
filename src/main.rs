extern crate kiss3d;
extern crate nalgebra;
extern crate rand;

use nalgebra::{Vector3, UnitQuaternion, Point3, Translation3};
use rand::distributions::{IndependentSample, Range};
use kiss3d::camera::ArcBall;
use kiss3d::window::Window;
use kiss3d::light::Light;

#[derive(Debug)]
struct Room {
    name: String,
    size: f32,
    color: (f32, f32, f32),
    position: Translation3<f32>,
}

impl Room {
    fn new(name: &str, size: f32, color: (f32, f32, f32), position: Translation3<f32>) -> Room {
        Room {
            name: name.to_string(),
            size: size,
            color: color,
            position: position,
        }
    }
}

fn construct_rooms() -> Vec<Room> {
    let between = Range::new(-1.5, 1.5);
    let mut rng = rand::thread_rng();

    let mut rooms = Vec::new();
    rooms.push(Room::new(
        "Sleep",
        0.7,
        (0.8941, 0.1020, 0.1098),
        Translation3::new(
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
        ),
    ));
    rooms.push(Room::new(
        "Leisure",
        0.5,
        (0.2157, 0.4941, 0.7216),
        Translation3::new(
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
        ),
    ));
    rooms.push(Room::new(
        "Food",
        0.35,
        (0.3020, 0.6863, 0.2902),
        Translation3::new(
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
        ),
    ));
    rooms.push(Room::new(
        "Work/Lab",
        0.35,
        (0.5961, 0.3059, 0.6392),
        Translation3::new(
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
        ),
    ));
    rooms.push(Room::new(
        "Greenhouse",
        0.25,
        (1.0, 0.4980, 0.0),
        Translation3::new(
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
        ),
    ));
    rooms.push(Room::new(
        "Sport",
        0.4,
        (1.0, 1.0, 0.2),
        Translation3::new(
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
            between.ind_sample(&mut rng),
        ),
    ));
    rooms
}

fn main() {

    let habitat = construct_rooms();

    let eye = Point3::new(3.0, 3.0, 3.0);
    let at = Point3::origin();
    let mut camera = ArcBall::new(eye, at);

    let mut window = Window::new_with_size("Surface Habitats", 1920, 1080);
    window.set_light(Light::StickToCamera);

    for room in habitat.iter() {
        let mut draw_room = window.add_sphere(room.size);
        draw_room.set_color(room.color.0, room.color.1, room.color.2);
        draw_room.set_local_translation(room.position);
    }

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);

    while window.render_with_camera(&mut camera) {
        window.scene_mut().prepend_to_local_rotation(&rot);
    }
}
