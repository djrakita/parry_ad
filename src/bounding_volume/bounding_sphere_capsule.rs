use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry};
use crate::shape::Capsule;
use ad_trait::AD;

impl<T: AD> Capsule<T> {
    /// Computes the world-space bounding sphere of this capsule, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        self.local_bounding_sphere().transform_by(pos)
    }

    /// Computes the world-space bounding sphere of this capsule.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        let radius = self.radius + self.half_height();
        BoundingSphere::new(self.center(), radius)
    }
}
