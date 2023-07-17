//! Support mapping based HalfSpace shape.
use crate::math::{Vector};
use na::Unit;

use ad_trait::AD;

/// A half-space delimited by an infinite plane.
#[derive(PartialEq, Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(as = "Self"),
    archive(check_bytes)
)]
#[cfg_attr(feature = "cuda", derive(cust_core::DeviceCopy))]
#[repr(C)]
pub struct HalfSpace<T: AD> {
    /// The halfspace planar boundary's outward normal.
    pub normal: Unit<Vector<T>>,
}

impl<T: AD> HalfSpace<T> {
    /// Builds a new halfspace from its center and its normal.
    #[inline]
    pub fn new(normal: Unit<Vector<T>>) -> HalfSpace<T> {
        HalfSpace { normal }
    }

    /// Computes a scaled version of this half-space.
    ///
    /// Returns `None` if `self.normal` scaled by `scale` is zero (the scaled half-space
    /// degenerates to a single point).
    pub fn scaled(self, scale: &Vector<T>) -> Option<Self> {
        Unit::try_new(self.normal.component_mul(scale), T::zero()).map(|normal| Self { normal })
    }
}
