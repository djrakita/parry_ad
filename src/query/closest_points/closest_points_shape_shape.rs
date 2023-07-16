use crate::math::{Isometry};
use crate::query::{ClosestPoints, DefaultQueryDispatcher, QueryDispatcher, Unsupported};
use crate::shape::Shape;
use ad_trait::AD;

/// Computes the pair of closest points between two shapes.
///
/// Returns `ClosestPoints::Disjoint` if the objects are separated by a distance greater than `max_dist`.
/// The result points in `ClosestPoints::WithinMargin` are expressed in world-space.
pub fn closest_points<T: AD>(
    pos1: &Isometry<T>,
    g1: &dyn Shape<T>,
    pos2: &Isometry<T>,
    g2: &dyn Shape<T>,
    max_dist: T,
) -> Result<ClosestPoints<T>, Unsupported> {
    let pos12 = pos1.inv_mul(pos2);
    DefaultQueryDispatcher
        .closest_points(&pos12, g1, g2, max_dist)
        .map(|res| res.transform_by(pos1, pos2))
}
