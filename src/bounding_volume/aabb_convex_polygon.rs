use crate::bounding_volume::Aabb;
use crate::math::{Isometry};
use crate::shape::ConvexPolygon;
use ad_trait::AD;

impl<T: AD> ConvexPolygon<T> {
    /// Computes the world-space Aabb of this convex polygon, transformed by `pos`.
    #[inline]
    pub fn aabb(&self, pos: &Isometry<T>) -> Aabb<T> {
        super::details::point_cloud_aabb(pos, self.points())
    }

    /// Computes the local-space Aabb of this convex polygon.
    #[inline]
    pub fn local_aabb(&self) -> Aabb<T> {
        super::details::local_point_cloud_aabb(self.points())
    }
}
