use crate::math::{Isometry};
use crate::query::{DefaultQueryDispatcher, QueryDispatcher, Unsupported};
use crate::shape::Shape;
use ad_trait::AD;

/// Tests whether two shapes are intersecting.
pub fn intersection_test<T: AD>(
    pos1: &Isometry<T>,
    g1: &dyn Shape<T>,
    pos2: &Isometry<T>,
    g2: &dyn Shape<T>,
) -> Result<bool, Unsupported> {
    let pos12 = pos1.inv_mul(pos2);
    DefaultQueryDispatcher.intersection_test(&pos12, g1, g2)
}
