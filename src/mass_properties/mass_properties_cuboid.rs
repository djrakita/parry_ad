use ad_trait::AD;
use crate::mass_properties::MassProperties;
use crate::math::{Point, PrincipalAngularInertia, Real, Vector};

impl<T: AD> MassProperties<T> {
    pub(crate) fn cuboid_volume_unit_inertia(
        half_extents: Vector<T>,
    ) -> (Real, PrincipalAngularInertia<T>) {
        #[cfg(feature = "dim2")]
        {
            let volume = half_extents.x * half_extents.y * T::constant(4.0);
            let ix = (half_extents.x * half_extents.x) / T::constant(3.0);
            let iy = (half_extents.y * half_extents.y) / T::constant(3.0);

            (volume, ix + iy)
        }

        #[cfg(feature = "dim3")]
        {
            let volume = half_extents.x * half_extents.y * half_extents.z * T::constant(8.0);
            let ix = (half_extents.x * half_extents.x) / T::constant(3.0);
            let iy = (half_extents.y * half_extents.y) / T::constant(3.0);
            let iz = (half_extents.z * half_extents.z) / T::constant(3.0);

            (volume, Vector::new(iy + iz, ix + iz, ix + iy))
        }
    }

    /// Computes the mass properties of a cuboid.
    pub fn from_cuboid(density: T, half_extents: Vector<T>) -> Self {
        let (vol, unit_i) = Self::cuboid_volume_unit_inertia(half_extents);
        let mass = vol * density;
        Self::new(Point::origin(), mass, unit_i * mass)
    }
}
