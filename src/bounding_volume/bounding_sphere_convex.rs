use crate::bounding_volume;
use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry};
use crate::shape::ConvexPolyhedron;
use ad_trait::AD;

impl<T: AD> ConvexPolyhedron<T> {
    /// Computes the world-space bounding sphere of this convex polyhedron, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        let bv: BoundingSphere = self.local_bounding_sphere();
        bv.transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this convex polyhedron.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        let (center, radius) = bounding_volume::details::point_cloud_bounding_sphere(self.points());

        BoundingSphere::new(center, radius)
    }
}
