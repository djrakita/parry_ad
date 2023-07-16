use crate::mass_properties::MassProperties;
use crate::math::{Point};
use crate::shape::Triangle;
use ad_trait::AD;

impl<T: AD> MassProperties<T> {
    /// Computes the mass properties of a triangle-mesh.
    pub fn from_trimesh(
        density: T,
        vertices: &[Point<T>],
        indices: &[[u32; 3]],
    ) -> MassProperties {
        let (area, com) = trimesh_area_and_center_of_mass(vertices, indices);

        if area == T::zero() {
            return MassProperties::new(com, T::zero(), T::zero());
        }

        let mut itot = T::zero();

        for idx in indices {
            let triangle = Triangle::new(
                vertices[idx[0] as usize],
                vertices[idx[1] as usize],
                vertices[idx[2] as usize],
            );

            // TODO: is the parallel axis theorem correctly applied here?
            let area = triangle.area();
            let ipart = triangle.unit_angular_inertia();
            itot += ipart * area;
        }

        Self::new(com, area * density, itot * density)
    }
}

/// Computes the area and center-of-mass of a triangle-mesh.
pub fn trimesh_area_and_center_of_mass<T: AD>(
    vertices: &[Point<T>],
    indices: &[[u32; 3]],
) -> (T, Point<T>) {
    let mut res = Point::origin();
    let mut areasum = T::zero();

    for idx in indices {
        let triangle = Triangle::new(
            vertices[idx[0] as usize],
            vertices[idx[1] as usize],
            vertices[idx[2] as usize],
        );
        let area = triangle.area();
        let center = triangle.center();

        res += center.coords * area;
        areasum += area;
    }

    if areasum == T::zero() {
        (areasum, res)
    } else {
        (areasum, res / areasum)
    }
}
