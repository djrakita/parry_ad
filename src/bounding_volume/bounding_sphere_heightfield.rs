use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry};
use crate::shape::{GenericHeightField, HeightFieldStorage};
use ad_trait::AD;

impl<Storage: HeightFieldStorage<T>, T: AD> GenericHeightField<Storage, T> {
    /// Computes the world-space bounding sphere of this height-field, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<T>) -> BoundingSphere<T> {
        self.local_aabb().bounding_sphere().transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this height-field.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_aabb().bounding_sphere()
    }
}
