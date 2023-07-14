use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry};
use crate::shape::Polyline;
use ad_trait::AD;

impl<T: AD> Polyline<T> {
    /// Computes the world-space bounding sphere of this polyline, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere {
        self.local_aabb().bounding_sphere().transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this polyline.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere {
        self.local_aabb().bounding_sphere()
    }
}
