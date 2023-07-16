use crate::math::{Isometry};
use crate::shape::HalfSpace;
use crate::shape::SupportMap;
use na;
use ad_trait::AD;

/// Distance between a halfspace and a support-mapped shape.
pub fn distance_halfspace_support_map<G: ?Sized + SupportMap<T>, T: AD>(
    pos12: &Isometry<T>,
    halfspace: &HalfSpace<T>,
    other: &G,
) -> T {
    let deepest = other.support_point_toward(pos12, &-halfspace.normal);
    halfspace.normal.dot(&deepest.coords).max(na::zero())
}

/// Distance between a support-mapped shape and a halfspace.
pub fn distance_support_map_halfspace<G: ?Sized + SupportMap<T>, T: AD>(
    pos12: &Isometry<T>,
    other: &G,
    halfspace: &HalfSpace<T>,
) -> T {
    distance_halfspace_support_map(&pos12.inverse(), halfspace, other)
}
