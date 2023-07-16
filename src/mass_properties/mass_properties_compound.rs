use crate::mass_properties::MassProperties;
use crate::math::{Isometry};
use crate::shape::SharedShape;
use ad_trait::AD;

impl<T: AD> MassProperties<T> {
    /// Computes the mass properties of a compound shape.
    pub fn from_compound(density: T, shapes: &[(Isometry<T>, SharedShape<T>)]) -> Self {
        shapes
            .iter()
            .map(|s| s.1.mass_properties(density).transform_by(&s.0))
            .sum()
    }
}
