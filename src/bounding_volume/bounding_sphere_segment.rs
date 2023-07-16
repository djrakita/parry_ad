use crate::bounding_volume;
use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry};
use crate::shape::Segment;
use ad_trait::AD;

impl<T: AD> Segment<T> {
    /// Computes the world-space bounding sphere of this segment, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        let bv: BoundingSphere<T> = self.local_bounding_sphere();
        bv.transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this segment.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        let pts = [self.a, self.b];
        let (center, radius) = bounding_volume::details::point_cloud_bounding_sphere(&pts[..]);

        BoundingSphere::new(center, radius)
    }
}
