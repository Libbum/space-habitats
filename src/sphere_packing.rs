use nalgebra::{self, Point3, Translation3};
use rand::{self, Rng};
use std::iter::repeat;
use std::collections::VecDeque;
use std::f32::{MIN, MAX};

#[derive(PartialEq, Debug, Clone)]
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

    /// If the distance between the centers of two spheres is less than the sum of
    /// their radii, we can consider them to be overlapping. Will return `true` in this case.
    fn overlaps(&self, other: &Sphere) -> bool {
        nalgebra::distance(&self.center, &other.center) < self.radius + other.radius
    }
}

trait Bounded<T> {
    /// Checks if an object exists inside some bounding geometry.
    fn is_bounded(&self, container: &T) -> bool;
}

impl Bounded<Sphere> for Sphere {
    /// Checks if an object exists inside some bounding geometry.
    fn is_bounded(&self, container: &Sphere) -> bool {
        nalgebra::distance(&Point3::origin(), &self.center) + self.radius < container.radius
    }
}

#[derive(Debug)]
/// Defines an active front. NOTE: I thought this may have required a little more machinery that
/// it currenly requires, so we may demote this soon.
struct Front {
    /// Central point in space where this sphere is located.
    spheres: Vec<Sphere>,
}

impl Front {
    /// Creates a `new` front, expected to be called after `init_spheres`, taking a copy
    /// of the output.
    pub fn new(spheres: Vec<Sphere>) -> Front {
        Front { spheres: spheres }
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
    println!("{:?}", radii);

    // S := {s₁, s₂, s₃}
    let mut spheres = init_spheres(all_radii);

    // F := {s₁, s₂, s₃}
    let mut front = Front::new(spheres.clone());

    let mut new_radius = radii.pop_front().unwrap_or_default();
    let mut rng = rand::thread_rng();

    'outer: while !front.spheres.is_empty() {
        // s₀ := s(c₀, r₀) picked at random
        let curr_sphere = rng.choose(&spheres).unwrap().clone();
        // V := {s(c', r') ∈ S : d(c₀, c') ≤ r₀ + r' + 2r}
        let set_v = spheres
            .clone()
            .into_iter()
            .filter(|s_dash| {
                nalgebra::distance(&curr_sphere.center, &s_dash.center) <=
                    curr_sphere.radius + s_dash.radius + 2. * new_radius
            })
            .collect::<Vec<_>>();

        for (s_i, s_j) in pairs(&set_v) {
            let mut set_f = identify_f(&curr_sphere, s_i, s_j, &container, &set_v, new_radius);
            if !set_f.is_empty() {
                // Found at least one position to place the sphere,
                // choose one and move on
                let s_new = rng.choose(&set_f).unwrap();
                front.spheres.push(s_new.clone());
                spheres.push(s_new.clone());
                new_radius = radii.pop_front().unwrap_or_default();
                continue 'outer;
            }
        }
        // This is a nightly function only
        front.spheres.remove_item(&curr_sphere);
    }
    spheres
}


/// $f$ is as a set of spheres (or the empty set) such that they have a known `radius`,
/// are in outer contact with `s_1`, `s_2` and `s_3` simultaneously, are completely
/// contained in `container` and do not overlap with any element of `set_v`.
/// The set f has at most two elements, because there exist at most two spheres with
/// `radius` in outer contact with `s_1`, `s_2` and `s_3` simultaneously.
fn identify_f(
    s_1: &Sphere,
    s_2: &Sphere,
    s_3: &Sphere,
    container: &Sphere,
    set_v: &Vec<Sphere>,
    radius: f32,
) -> Vec<Sphere> {

    // If we use (mostly) the same conventions as `init_spheres`, we start
    // with a base triangle at s_1=A(0,0,0), s_2=B(c,0,0), s_3=C(e,f.0) and look for solutions
    // to s_4=D(x,y,z). Let AD=m, BD=n, CD=p we arrive at the equations
    // x²+y²+z²=m²; (x−c)²+y²+z²=n²; (x−e)²+(y−f)²+z²=p². This doesn't give us the
    // simplest solution to z, but it is what it is...

    // First, translate our spheres onto this cordinate system so we can use these minimal equations
    let value_c = s_1.radius + s_2.radius;
    let value_e = ((s_1.radius + s_3.radius).powi(2) + value_c.powi(2) - (s_2.radius + s_3.radius).powi(2)) / (2. * value_c);
    let value_f = ((s_1.radius + s_3.radius).powi(2) - value_e.powi(2)).sqrt();
    let reset_a = Translation3::new(-s_1.center.coords.x, -s_1.center.coords.y, -s_1.center.coords.z);
    let reset_b = Translation3::new(-s_2.center.coords.x + value_c, -s_2.center.coords.y, -s_2.center.coords.z);
    let reset_c = Translation3::new(-s_3.center.coords.x + value_e, -s_3.center.coords.y + value_e, -s_3.center.coords.z);
    let center_a = reset_a * s_1.center;
    let center_b = reset_b * s_2.center;
    let center_c = reset_c * s_3.center;
    println!("A {:?}, B {:?}, C {:?}", s_1, s_2, s_3);
    println!("A' {}, B' {}, C' {}", center_a, center_b, center_c);
    //let value_e = s_3.center.x;
    //let value_f = s_3.center.y;
    let distance_m = s_1.radius + radius;
    let distance_n = s_2.radius + radius;
    let distance_p = s_3.radius + radius;

    let x = (distance_m.powi(2) - distance_n.powi(2) + value_c.powi(2)) / (2. * value_c);
    let y = (value_f.powi(2) * value_c + distance_m.powi(2) * value_c -
                 value_e * distance_m.powi(2) + value_e * distance_n.powi(2) -
                 distance_p.powi(2) * value_c - value_e * value_c.powi(2) +
                 value_e.powi(2) * value_c) / (2. * value_f * value_c);
    let z = (distance_m.powi(2) - x.powi(2) - y.powi(2)).sqrt();

    let s_4_positive = Sphere::new(reset_c * Point3::new(x, y, z), radius);
    let s_4_negative = Sphere::new(reset_c * Point3::new(x, y, -z), radius);

    println!("C {}, C' {} {} 0, D {}", s_3.center, value_e, value_f, s_4_positive.center);
    let mut f = Vec::new();
    if s_4_positive.is_bounded(container) && !set_v.iter().any(|v| v.overlaps(&s_4_positive)) {
        f.push(s_4_positive);
    }
    if s_4_negative.is_bounded(container) && !set_v.iter().any(|v| v.overlaps(&s_4_negative)) {
        f.push(s_4_negative);
    }
    f
}

/// Calculates all possible pairs of a `set` of values.
fn pairs(set: &[Sphere]) -> Vec<(&Sphere, &Sphere)> {
    let n = set.len();

    if n == 2 {
        let mut minimal = Vec::new();
        minimal.push((&set[0], &set[1]));
        minimal
    } else {
        let mut vec_pairs = Vec::new();
        if n > 2 {
            // 0..n - m, but m = 2 and rust is non inclusive with its for loops
            for k in 0..n - 1 {
                let subset = &set[k + 1..n];
                vec_pairs.append(&mut subset
                    .iter()
                    .zip(repeat(&set[k]).take(subset.len()))
                    .collect::<Vec<_>>());
            }
        }
        vec_pairs
    }
}
