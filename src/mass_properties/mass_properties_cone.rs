use ad_trait::AD;
use crate::mass_properties::MassProperties;
use crate::math::{Point, PrincipalAngularInertia, Rotation, Vector};

impl<T: AD> MassProperties<T> {
    pub(crate) fn cone_y_volume_unit_inertia(
        half_height: T,
        radius: T,
    ) -> (T, PrincipalAngularInertia<T>) {
        let volume = radius * radius * T::pi() * half_height * T::constant(2.0 / 3.0);
        let sq_radius = radius * radius;
        let sq_height = half_height * half_height * T::constant(4.0);
        let off_principal = sq_radius * T::constant(3.0 / 20.0) + sq_height * T::constant(3.0 / 80.0);
        let principal = sq_radius * T::constant(3.0 / 10.0);

        (volume, Vector::new(off_principal, principal, off_principal))
    }

    /// Computes the mass properties of a cone.
    pub fn from_cone(density: T, half_height: T, radius: T) -> Self {
        let (cyl_vol, cyl_unit_i) = Self::cone_y_volume_unit_inertia(half_height, radius);
        let cyl_mass = cyl_vol * density;

        Self::with_principal_inertia_frame(
            Point::new(T::constant(0.0), -half_height / T::constant(2.0), T::constant(0.0)),
            cyl_mass,
            cyl_unit_i * cyl_mass,
            Rotation::identity(),
        )
    }
}
