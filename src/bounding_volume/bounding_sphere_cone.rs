use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry, Point};
use crate::shape::Cone;
use na::ComplexField;
use ad_trait::AD;

impl<T: AD> Cone<T> {
    /// Computes the world-space bounding sphere of this cone, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        let bv: BoundingSphere = self.local_bounding_sphere();
        bv.transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this cone.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        let radius =
            ComplexField::sqrt(self.radius * self.radius + self.half_height * self.half_height);

        BoundingSphere::new(Point::origin(), radius)
    }
}
