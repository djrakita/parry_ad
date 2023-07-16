use crate::math::{Isometry};
use crate::query::ClosestPoints;
use crate::shape::HalfSpace;
use crate::shape::SupportMap;
use ad_trait::AD;

/// Closest points between a halfspace and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn closest_points_halfspace_support_map<T: AD, G: ?Sized + SupportMap<T>>(
    pos12: &Isometry<T>,
    halfspace: &HalfSpace<T>,
    other: &G,
    margin: T,
) -> ClosestPoints<T> {
    assert!(
        margin >= T::zero(),
        "The proximity margin must be positive or null."
    );

    let deepest = other.support_point(pos12, &-halfspace.normal);
    let distance = halfspace.normal.dot(&(-deepest.coords));

    if distance >= -margin {
        if distance >= T::zero() {
            ClosestPoints::Intersecting
        } else {
            let p1 = deepest + *halfspace.normal * distance;
            let p2 = pos12.inverse_transform_point(&deepest);
            ClosestPoints::WithinMargin(p1, p2)
        }
    } else {
        ClosestPoints::Disjoint
    }
}

/// Closest points between a support-mapped shape (Cuboid, ConvexHull, etc.) and a halfspace.
pub fn closest_points_support_map_halfspace<T: AD, G: ?Sized + SupportMap<T>>(
    pos12: &Isometry<T>,
    other: &G,
    halfspace: &HalfSpace<T>,
    margin: T,
) -> ClosestPoints<T> {
    closest_points_halfspace_support_map(&pos12.inverse(), halfspace, other, margin).flipped()
}
