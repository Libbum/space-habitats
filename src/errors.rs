use std::error::Error;
use std::fmt;

#[derive(Debug)]
/// All errors thrown by the library
pub enum SpaceHabitatsError {
    /// If a sphere is given a negative radius.
    NegativeRadius,
    /// If a sphere is created but is not confined by the `Container`.
    /// This happens quite a lot and is generally handled silently. This error
    /// is only thrown by the `init_spheres` method. Usually this means the geometry
    /// of the container is perhaps not aligned to the origin, it is scaled too small,
    /// or the spheres you're attempting to pack are too large.
    Uncontained,
    /// We choose a random value from the `set_f` vector. `rand` returns an option and we pop
    /// the value. If it's `None` this error is thrown. Due to the contstuction of the
    /// rest of the method, it's safe to say this is unreachable.
    NoneSetF,
    /// We choose a random value from the `front` vector. `rand` returns an option and we pop
    /// the value. If it's `None` this error is thrown. Due to the contstuction of the
    /// rest of the method, it's safe to say this is unreachable.
    NoneFront,
}

impl fmt::Display for SpaceHabitatsError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            SpaceHabitatsError::NegativeRadius => write!(f, "Supplied radius is negative."),
            SpaceHabitatsError::Uncontained => {
                write!(f, "Sphere is not contained within bounding geometry.")
            }
            SpaceHabitatsError::NoneSetF => {
                write!(f, "Returned none when choosing value from set f")
            }
            SpaceHabitatsError::NoneFront => {
                write!(f, "Returned none when choosing value from front")
            }
        }
    }
}

impl Error for SpaceHabitatsError {
    fn description(&self) -> &str {
        match *self {
            SpaceHabitatsError::NegativeRadius => "negative radius",
            SpaceHabitatsError::Uncontained => "outside bounding geometry",
            SpaceHabitatsError::NoneSetF => "none from set f",
            SpaceHabitatsError::NoneFront => "none from front",
        }
    }
}
