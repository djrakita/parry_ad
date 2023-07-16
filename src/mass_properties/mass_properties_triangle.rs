use crate::mass_properties::MassProperties;
use crate::math::{Point};
use crate::shape::Triangle;
use ad_trait::AD;

impl<T: AD> MassProperties<T> {
    /// Computes the mass properties of a triangle.
    pub fn from_triangle(
        density: T,
        a: &Point<T>,
        b: &Point<T>,
        c: &Point<T>,
    ) -> MassProperties {
        let triangle = Triangle::new(*a, *b, *c);
        let area = triangle.area();
        let com = triangle.center();

        if area == T::zero() {
            return MassProperties::new(com, T::zero(), T::zero());
        }

        let ipart = triangle.unit_angular_inertia();

        Self::new(com, area * density, ipart * area * density)
    }
}
