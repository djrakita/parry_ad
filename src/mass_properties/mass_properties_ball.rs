use crate::mass_properties::MassProperties;
#[cfg(feature = "dim3")]
use crate::math::Vector;
use crate::math::{Point, PrincipalAngularInertia};
use na::RealField;
use ad_trait::AD;

impl<T: AD> MassProperties<T> {
    pub(crate) fn ball_volume_unit_angular_inertia(
        radius: T,
    ) -> (T, PrincipalAngularInertia<T>) {
        #[cfg(feature = "dim2")]
        {
            let volume = T::constant(f64::pi()) * radius * radius;
            let i = radius * radius / T::constant(2.0);
            (volume, i)
        }
        #[cfg(feature = "dim3")]
        {
            let volume = T::constant(f64::pi()) * radius * radius * radius * T::constant(4.0 / 3.0);
            let i = radius * radius * T::constant(2.0 / 5.0);

            (volume, Vector::repeat(i))
        }
    }

    /// Computes the mass properties of a ball.
    pub fn from_ball(density: T, radius: T) -> Self {
        let (vol, unit_i) = Self::ball_volume_unit_angular_inertia(radius);
        let mass = vol * density;
        Self::new(Point::origin(), mass, unit_i * mass)
    }
}
