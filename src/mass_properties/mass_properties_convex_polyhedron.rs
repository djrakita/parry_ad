use crate::mass_properties::MassProperties;
use crate::math::{Point, DIM};
use ad_trait::AD;

impl<T: AD> MassProperties<T> {
    /// Computes the mass properties of a convex polyhedron.
    pub fn from_convex_polyhedron(
        density: T,
        vertices: &[Point<T>],
        indices: &[[u32; DIM]],
    ) -> MassProperties<T> {
        Self::from_trimesh(density, vertices, indices)
    }
}
