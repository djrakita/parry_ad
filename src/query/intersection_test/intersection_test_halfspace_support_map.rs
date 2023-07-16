use crate::math::{Isometry};
use crate::shape::HalfSpace;
use crate::shape::SupportMap;
use ad_trait::AD;

/// Intersection test between a halfspace and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn intersection_test_halfspace_support_map<G: ?Sized + SupportMap<T>, T: AD>(
    pos12: &Isometry<T>,
    halfspace: &HalfSpace<T>,
    other: &G,
) -> bool {
    let deepest = other.support_point_toward(pos12, &-halfspace.normal);
    halfspace.normal.dot(&deepest.coords) <= T::zero()
}

/// Intersection test between a support-mapped shape (Cuboid, ConvexHull, etc.) and a halfspace.
pub fn intersection_test_support_map_halfspace<G: ?Sized + SupportMap<T>, T: AD>(
    pos12: &Isometry<T>,
    other: &G,
    halfspace: &HalfSpace<T>,
) -> bool {
    intersection_test_halfspace_support_map(&pos12.inverse(), halfspace, other)
}
