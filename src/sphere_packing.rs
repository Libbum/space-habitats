use nalgebra::{self, Vector3, Real, Point3};
use nalgebra::core::Matrix;
use nalgebra::geometry::Rotation3;
use rand::{self, Rng};
use std::iter::repeat;
use std::collections::VecDeque;

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
    // that the incenter is now the origin. This offset isn't entirely needed, but
    // positions us better for the camera when viewing the resultant packing.
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
    //Take first three for initialisation, keep the rest.
    let mut radii = all_radii.split_off(3);

    // S := {s₁, s₂, s₃}
    let mut spheres = init_spheres(all_radii);

    // F := {s₁, s₂, s₃}
    let mut front = spheres.clone();

    let mut new_radius = radii.pop_front().unwrap_or_default();
    let mut rng = rand::thread_rng();

    'outer: while !front.is_empty() {
        // s₀ := s(c₀, r₀) picked at random from F
        let curr_sphere = rng.choose(&front).unwrap().clone();
        // V := {s(c', r') ∈ S : d(c₀, c') ≤ r₀ + r' + 2r}
        let set_v = spheres
            .clone()
            .into_iter()
            .filter(|s_dash| {
                s_dash != &curr_sphere &&
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
                front.push(s_new.clone());
                spheres.push(s_new.clone());
                new_radius = radii.pop_front().unwrap_or_default();
                println!(
                    "A {:?}, B {:?}, C {:?}, D {:?}",
                    curr_sphere,
                    s_i,
                    s_j,
                    s_new
                );
                continue 'outer;
            }
        }
        // This is a nightly function only
        front.remove_item(&curr_sphere);
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

    let empty = Vector3::new(0., 0., 0.); // A helper value, since nalgebra doesn't seem to have a method that's helpful here
    // Remove ABC's translation component to compare rotation axes
    let origin_b = s_2.center - s_1.center;
    let origin_c = s_3.center - s_1.center;

    // Identify trapezoid locations in the simplified basis
    let (mut vector_ef, mut vector_eg, mut vector_ehp, mut vector_ehn) =
        generate_trapezoids(&s_1.radius, &s_2.radius, &s_3.radius, &radius);

    // Trapezoid tips must now be translated to D in the the ABCD coordinate system.
    // We denote x,y,z as H in trapeziod EFGH, setting E=A.

    // Rotate EF onto AB
    let cross_ef_ab = Matrix::cross(&vector_ef.coords, &origin_b);

    // Only apply the rotation if needed
    if cross_ef_ab != empty {
        let rot_1 = Rotation3::new(axis_angle(cross_ef_ab, vector_ef.coords, origin_b));

        vector_ef = rot_1 * vector_ef;
        vector_eg = rot_1 * vector_eg;
        vector_ehp = rot_1 * vector_ehp;
        vector_ehn = rot_1 * vector_ehn;
    }

    // Rotate G to C by rotating the face normal of current triangle to the face normal of ABC
    let efg_normal = Matrix::cross(&vector_eg.coords, &vector_ef.coords);
    let abc_normal = Matrix::cross(&origin_c, &origin_b);
    let cross_normals = Matrix::cross(&efg_normal, &abc_normal);

    if cross_normals != empty {
        let rot_2 = Rotation3::new(axis_angle(cross_normals, efg_normal, abc_normal));

        vector_ehp = rot_2 * vector_ehp;
        vector_ehn = rot_2 * vector_ehn;
    }

    // Translate points to the ABCD basis
    vector_ehp.coords += s_1.center.coords;
    vector_ehn.coords += s_1.center.coords;

    // Generate new shperes using rotated coordinates, translated to A's basis
    let s_4_positive = Sphere::new(vector_ehp, radius);
    let s_4_negative = Sphere::new(vector_ehn, radius);

    let mut f = Vec::new();
    // Make sure the spheres are bounded by the containing geometry and do not overlap any spheres in V
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

/// Generates trapezoids in the simplest base coordinates from known radii of a set of tangent spheres (`r_1` to `r_3`)
/// and a new tangent sphere with known `radius` but unknown position.
/// Returns locations to points C and B from a commoo origin A, along with locations of the two possible trapezoid tip locations.
fn generate_trapezoids(
    r_1: &f32,
    r_2: &f32,
    r_3: &f32,
    radius: &f32,
) -> (Point3<f32>, Point3<f32>, Point3<f32>, Point3<f32>) {

    // First, translate values of our spheres onto a cordinate system A(0,0,0), B(c,0,0), C(e,f,0), D(x,y,z)
    // so we can use these minimal equations. Here, let m = AD, n = BD, p = CD be distances between points
    let value_c = r_1 + r_2;
    let value_e = ((r_1 + r_3).powi(2) + value_c.powi(2) - (r_2 + r_3).powi(2)) / (2. * value_c);
    let value_f = ((r_1 + r_3).powi(2) - value_e.powi(2)).sqrt();
    let distance_m = r_1 + radius;
    let distance_n = r_2 + radius;
    let distance_p = r_3 + radius;

    // Identify D(x,y,z)
    let x = (distance_m.powi(2) - distance_n.powi(2) + value_c.powi(2)) / (2. * value_c);
    let y = (value_f.powi(2) * value_c + distance_m.powi(2) * value_c -
                 value_e * distance_m.powi(2) + value_e * distance_n.powi(2) -
                 distance_p.powi(2) * value_c - value_e * value_c.powi(2) +
                 value_e.powi(2) * value_c) /
        (2. * value_f * value_c);
    let z = (distance_m.powi(2) - x.powi(2) - y.powi(2)).sqrt();

    // Generate EFGH points for rotation
    let point_b = Point3::new(value_c, 0., 0.);
    let point_c = Point3::new(value_e, value_f, 0.);
    // Our two found trapezoid tips:
    let point_d_positive = Point3::new(x, y, z);
    let point_d_negative = Point3::new(x, y, -z);

    (point_b, point_c, point_d_positive, point_d_negative)
}


/// Find the rotation axis angle vector $\vec{r} = \theta\hat{e}$ given the un-normalised rotation axis vector
/// and the two (un-normalised) vectors for which the rotation is to be mapped.
fn axis_angle(axis: Vector3<f32>, source: Vector3<f32>, target: Vector3<f32>) -> Vector3<f32> {
    let rotation_axis = axis / nalgebra::norm(&axis);

    let norm_source = source / nalgebra::norm(&source);
    let norm_target = target / nalgebra::norm(&target);
    let rotation_angle = Real::acos(nalgebra::dot(&norm_source, &norm_target));

    rotation_angle * rotation_axis
}
