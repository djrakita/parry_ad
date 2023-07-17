use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry, Point};
use crate::shape::Cuboid;
use ad_trait::AD;

impl<T: AD> Cuboid<T> {
    /// Computes the world-space bounding sphere of this cuboid, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        let bv: BoundingSphere<T> = self.local_bounding_sphere();
        bv.transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this cuboid.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        let radius = self.half_extents.norm();
        BoundingSphere::new(Point::origin(), radius)
    }
}
