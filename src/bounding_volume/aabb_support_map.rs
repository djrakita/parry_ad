use ad_trait::AD;
use crate::bounding_volume;
use crate::bounding_volume::Aabb;
use crate::math::{Isometry};
use crate::shape::Segment;
#[cfg(feature = "dim3")]
use crate::shape::{Cone, Cylinder};

#[cfg(feature = "dim3")]
impl<T: AD> Cone<T> {
    /// Computes the world-space Aabb of this cone, transformed by `pos`.
    #[inline]
    pub fn aabb(&self, pos: &Isometry<T>) -> Aabb<T> {
        bounding_volume::details::support_map_aabb(pos, self)
    }

    /// Computes the local-space Aabb of this cone.
    #[inline]
    pub fn local_aabb(&self) -> Aabb<T> {
        bounding_volume::details::local_support_map_aabb(self)
    }
}

#[cfg(feature = "dim3")]
impl<T: AD> Cylinder<T> {
    /// Computes the world-space Aabb of this cylinder, transformed by `pos`.
    #[inline]
    pub fn aabb(&self, pos: &Isometry<T>) -> Aabb<T> {
        bounding_volume::details::support_map_aabb(pos, self)
    }

    /// Computes the local-space Aabb of this cylinder.
    #[inline]
    pub fn local_aabb(&self) -> Aabb<T> {
        bounding_volume::details::local_support_map_aabb(self)
    }
}

impl<T: AD> Segment<T> {
    /// Computes the world-space Aabb of this segment, transformed by `pos`.
    #[inline]
    pub fn aabb(&self, pos: &Isometry<T>) -> Aabb<T> {
        self.transformed(pos).local_aabb()
    }

    /// Computes the local-space Aabb of this segment.
    #[inline]
    pub fn local_aabb(&self) -> Aabb<T> {
        bounding_volume::details::local_support_map_aabb(self)
    }
}
