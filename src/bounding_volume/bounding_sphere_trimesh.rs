use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry};
use crate::shape::TriMesh;
use ad_trait::AD;

impl<T: AD> TriMesh<T> {
    /// Computes the world-space bounding sphere of this triangle mesh, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        self.local_aabb().bounding_sphere().transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this triangle mesh.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_aabb().bounding_sphere()
    }
}
