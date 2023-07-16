use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry, Point};
use crate::shape::HalfSpace;
use ad_trait::AD;

use num::Bounded;

impl<T: AD> HalfSpace<T> {
    /// Computes the world-space bounding sphere of this half-space, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        let bv: BoundingSphere<T> = self.local_bounding_sphere();
        bv.transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this half-space.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        let radius = T::constant(f64::max_value());

        BoundingSphere::new(Point::origin(), radius)
    }
}
