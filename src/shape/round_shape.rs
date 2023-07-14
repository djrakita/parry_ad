use crate::math::{Point, Real, Vector};
use crate::shape::SupportMap;
use na::Unit;

use ad_trait::AD;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(check_bytes)
)]
#[cfg_attr(feature = "cuda", derive(cust_core::DeviceCopy))]
#[derive(Copy, Clone, Debug)]
#[repr(C)]
/// A shape with rounded borders.
pub struct RoundShape<S, T: AD> {
    /// The shape being rounded.
    pub inner_shape: S,
    /// The radius of the rounded border.
    pub border_radius: T,
}

impl<S: SupportMap, T: AD> SupportMap for RoundShape<S, T> {
    fn local_support_point(&self, dir: &Vector<T>) -> Point<T> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    fn local_support_point_toward(&self, dir: &Unit<Vector<T>>) -> Point<T> {
        self.inner_shape.local_support_point_toward(dir) + **dir * self.border_radius
    }
}
