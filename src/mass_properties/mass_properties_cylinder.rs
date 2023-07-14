use crate::mass_properties::MassProperties;
use crate::math::{PrincipalAngularInertia, Vector};
#[cfg(feature = "dim3")]
use {
    crate::math::{Point, Rotation},
    na::RealField,
};
use ad_trait::AD;

impl<T: AD> MassProperties<T> {
    pub(crate) fn cylinder_y_volume_unit_inertia(
        half_height: T,
        radius: T,
    ) -> (T, PrincipalAngularInertia<T>) {
        #[cfg(feature = "dim2")]
        {
            Self::cuboid_volume_unit_inertia(Vector::new(radius, half_height))
        }

        #[cfg(feature = "dim3")]
        {
            let volume = half_height * radius * radius * T::constant(f64::pi() * 2.0);
            let sq_radius = radius * radius;
            let sq_height = half_height * half_height * T::constant(4.0);
            let off_principal = (sq_radius * T::constant(3.0) + sq_height) / T::constant(12.0);

            let inertia = Vector::new(off_principal, sq_radius / T::constant(2.0), off_principal);
            (volume, inertia)
        }
    }

    /// Computes the mass properties of a cylinder.
    #[cfg(feature = "dim3")]
    pub fn from_cylinder(density: T, half_height: T, radius: T) -> Self {
        let (cyl_vol, cyl_unit_i) = Self::cylinder_y_volume_unit_inertia(half_height, radius);
        let cyl_mass = cyl_vol * density;

        Self::with_principal_inertia_frame(
            Point::origin(),
            cyl_mass,
            cyl_unit_i * cyl_mass,
            Rotation::identity(),
        )
    }
}
