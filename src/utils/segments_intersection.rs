use na::Point2;

use crate::shape::{SegmentPointLocation, Triangle, TriangleOrientation};
use ad_trait::AD;

#[cfg(not(feature = "std"))]
use na::ComplexField;

/// Intersection between two segments.
pub enum SegmentsIntersection<T: AD> {
    /// Single point of intersection.
    Point {
        /// Location of the intersection point on the first segment.
        loc1: SegmentPointLocation<T>,
        /// Location of the intersection point on the second segment.
        loc2: SegmentPointLocation<T>,
    },
    /// Intersection along a segment (when both segments are collinear).
    Segment {
        /// Location of the first intersection point on the first segment.
        first_loc1: SegmentPointLocation<T>,
        /// Location of the first intersection point on the second segment.
        first_loc2: SegmentPointLocation<T>,
        /// Location of the second intersection point on the first segment.
        second_loc1: SegmentPointLocation<T>,
        /// Location of the second intersection point on the second segment.
        second_loc2: SegmentPointLocation<T>,
    },
}

/// Computes the intersection between two segments.
pub fn segments_intersection2d<T: AD>(
    a: &Point2<T>,
    b: &Point2<T>,
    c: &Point2<T>,
    d: &Point2<T>,
    epsilon: T,
) -> Option<SegmentsIntersection<T>> {
    let denom = a.x * (d.y - c.y) + b.x * (c.y - d.y) + d.x * (b.y - a.y) + c.x * (a.y - b.y);

    // If denom is zero, then segments are parallel: handle separately.
    if denom.abs() < epsilon || ulps_eq!(denom, T::zero()) {
        return parallel_intersection(a, b, c, d, epsilon);
    }

    let num = a.x * (d.y - c.y) + c.x * (a.y - d.y) + d.x * (c.y - a.y);
    let s = num / denom;

    let num = -(a.x * (c.y - b.y) + b.x * (a.y - c.y) + c.x * (b.y - a.y));
    let t = num / denom;

    if T::zero() > s || s > T::one() || T::zero() > t || t > T::one() {
        None
    } else {
        let loc1 = if s == T::zero() {
            SegmentPointLocation::OnVertex(0)
        } else if s == denom {
            SegmentPointLocation::OnVertex(1)
        } else {
            SegmentPointLocation::OnEdge([T::one() - s, s])
        };

        let loc2 = if t == T::zero() {
            SegmentPointLocation::OnVertex(0)
        } else if t == denom {
            SegmentPointLocation::OnVertex(1)
        } else {
            SegmentPointLocation::OnEdge([T::one() - t, t])
        };

        Some(SegmentsIntersection::Point { loc1, loc2 })
    }
}

fn parallel_intersection<T: AD>(
    a: &Point2<T>,
    b: &Point2<T>,
    c: &Point2<T>,
    d: &Point2<T>,
    epsilon: T,
) -> Option<SegmentsIntersection<T>> {
    if Triangle::orientation2d(a, b, c, epsilon) != TriangleOrientation::Degenerate {
        return None;
    }

    let ab_c = between(a, b, c);
    let ab_d = between(a, b, d);
    if let (Some(loc1), Some(loc2)) = (ab_c, ab_d) {
        return Some(SegmentsIntersection::Segment {
            first_loc1: loc1,
            first_loc2: SegmentPointLocation::OnVertex(0),
            second_loc1: loc2,
            second_loc2: SegmentPointLocation::OnVertex(1),
        });
    }

    let cd_a = between(c, d, a);
    let cd_b = between(c, d, b);
    if let (Some(loc1), Some(loc2)) = (cd_a, cd_b) {
        return Some(SegmentsIntersection::Segment {
            first_loc1: SegmentPointLocation::OnVertex(0),
            first_loc2: loc1,
            second_loc1: SegmentPointLocation::OnVertex(1),
            second_loc2: loc2,
        });
    }

    if let (Some(loc1), Some(loc2)) = (ab_c, cd_b) {
        return Some(SegmentsIntersection::Segment {
            first_loc1: loc1,
            first_loc2: SegmentPointLocation::OnVertex(0),
            second_loc1: SegmentPointLocation::OnVertex(1),
            second_loc2: loc2,
        });
    }

    if let (Some(loc1), Some(loc2)) = (ab_c, cd_a) {
        return Some(SegmentsIntersection::Segment {
            first_loc1: loc1,
            first_loc2: SegmentPointLocation::OnVertex(0),
            second_loc1: SegmentPointLocation::OnVertex(0),
            second_loc2: loc2,
        });
    }

    if let (Some(loc1), Some(loc2)) = (ab_d, cd_b) {
        return Some(SegmentsIntersection::Segment {
            first_loc1: loc1,
            first_loc2: SegmentPointLocation::OnVertex(1),
            second_loc1: SegmentPointLocation::OnVertex(1),
            second_loc2: loc2,
        });
    }

    if let (Some(loc1), Some(loc2)) = (ab_d, cd_a) {
        return Some(SegmentsIntersection::Segment {
            first_loc1: loc1,
            first_loc2: SegmentPointLocation::OnVertex(1),
            second_loc1: SegmentPointLocation::OnVertex(0),
            second_loc2: loc2,
        });
    }

    return None;
}

// Checks that `c` is in-between `a` and `b`.
// Assumes the three points are collinear.
fn between<T: AD>(a: &Point2<T>, b: &Point2<T>, c: &Point2<T>) -> Option<SegmentPointLocation<T>> {
    // If ab not vertical, check betweenness on x; else on y.
    // FIXME: handle cases wher we actually are on a vertex (to return OnEdge instead of OnVertex)?
    if a.x != b.x {
        if a.x <= c.x && c.x <= b.x {
            let bcoord = (c.x - a.x) / (b.x - a.x);
            return Some(SegmentPointLocation::OnEdge([T::one() - bcoord, bcoord]));
        } else if a.x >= c.x && c.x >= b.x {
            let bcoord = (c.x - b.x) / (a.x - b.x);
            return Some(SegmentPointLocation::OnEdge([bcoord, T::one() - bcoord]));
        }
    } else if a.y != b.y {
        if a.y <= c.y && c.y <= b.y {
            let bcoord = (c.y - a.y) / (b.y - a.y);
            return Some(SegmentPointLocation::OnEdge([T::one() - bcoord, bcoord]));
        } else if a.y >= c.y && c.y >= b.y {
            let bcoord = (c.y - b.y) / (a.y - b.y);
            return Some(SegmentPointLocation::OnEdge([bcoord, T::one() - bcoord]));
        }
    } else if a.x == c.x && a.y == c.y {
        return Some(SegmentPointLocation::OnVertex(0));
    }

    None
}
