use crate::math::{Isometry};
use crate::query::ClosestPoints;
use crate::shape::Segment;
use ad_trait::AD;

/// Distance between two segments.
#[inline]
pub fn distance_segment_segment<T: AD>(
    pos12: &Isometry<T>,
    segment1: &Segment<T>,
    segment2: &Segment<T>,
) -> T {
    match crate::query::details::closest_points_segment_segment(
        pos12,
        segment1,
        segment2,
        T::constant(f64::MAX),
    ) {
        ClosestPoints::WithinMargin(p1, p2) => na::distance(&p1, &(pos12 * p2)),
        _ => T::zero(),
    }
}
