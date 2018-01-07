use nalgebra::{Vector3, Point3};
use std::collections::VecDeque;
use std::f32::{MIN, MAX};

#[derive(Debug)]
/// Constructs a sphere located at `center` in Euclidean space with a given `radius`.
pub struct Sphere {
    /// Central point in space where this sphere is located.
    pub center: Point3<f32>,
    /// Radius of current sphere.
    pub radius: f32,
}

impl Sphere {
    /// Creates a `new` sphere given the location of the spheres' `center` and its' `radius`.
    pub fn new(center: Point3<f32>, radius: f32) -> Sphere {
        Sphere {
            center: center,
            radius: radius,
        }
    }
}

/// Creates three initial spheres that are tangent pairwise. The incenter of the triangle formed
/// by verticies located at the centers of each sphere is aligned at the origin.
fn init_spheres(mut radii: VecDeque<f32>) -> Vec<Sphere> {
    let mut init = Vec::new();

    //            C (x,y)
    //            ^
    //           / \
    //        b /   \ a
    //         /     \
    //        /       \
    // A (0,0)--------- B (c,0)
    //            c

    // Sphere A can sit at the origin, sphere B extends outward along the x axis
    // sphere C extends outward along the y axis and complete the triangle
    let radius_a = radii.pop_front().unwrap_or_default();
    let radius_b = radii.pop_front().unwrap_or_default();
    let radius_c = radii.pop_front().unwrap_or_default();
    let distance_c = radius_a + radius_b;
    let distance_b = radius_a + radius_c;
    let distance_a = radius_c + radius_b;

    let x = (distance_b.powi(2) + distance_c.powi(2) - distance_a.powi(2)) / (2. * distance_c);
    let y = (distance_b.powi(2) - x.powi(2)).sqrt();

    // Find incenter
    let perimeter = distance_a + distance_b + distance_c;
    let incenter_x = (distance_b * distance_c + distance_c * x) / perimeter;
    let incenter_y = (distance_c * y) / perimeter;

    // Create spheres at positions shown in the diagram above, but offset such
    // that the incenter is now the origin
    init.push(Sphere::new(
        Point3::new(-incenter_x, -incenter_y, 0.),
        radius_a,
    ));
    init.push(Sphere::new(
        Point3::new(distance_c - incenter_x, -incenter_y, 0.),
        radius_b,
    ));
    init.push(Sphere::new(
        Point3::new(x - incenter_x, y - incenter_y, 0.),
        radius_c,
    ));
    init
}

/// Packs all habitat spheres to be as dense as possible.
/// Requires a `containter` ($G$) and a set of `all_radii` ($D$). This should be from some
/// distribution if this were a complete implementation, but we will just provide a set of known room sizes.
/// Additionally, the `container` can be arbitrary geometry, but we'll just provide a large sphere for
/// simplicity,
pub fn pack_spheres(container: Sphere, mut all_radii: VecDeque<f32>) -> Vec<Sphere> {
    let mut r_min = MIN;
    let mut r_max = MAX;

    for radius in all_radii.iter() {
        if radius.is_finite() {
            r_min = r_min.min(*radius);
            r_max = r_max.max(*radius);
        }
    }

    //Take first three for initialisation, keep the rest.
    let mut radii = all_radii.split_off(3);

    let mut spheres = init_spheres(all_radii);
    spheres
}
