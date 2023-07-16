use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry, Point};
use crate::shape::Ball;
use ad_trait::AD;

impl<T: AD> Ball<T> {
    /// Computes the world-space bounding sphere of this ball, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        let bv: BoundingSphere<T> = self.local_bounding_sphere();
        bv.transform_by(pos)
    }

    /// Computes the local-space Aabb of this ball.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        BoundingSphere::new(Point::origin(), self.radius)
    }
}
