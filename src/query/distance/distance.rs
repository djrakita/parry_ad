use crate::math::{Isometry};

use crate::query::{DefaultQueryDispatcher, QueryDispatcher, Unsupported};
use crate::shape::Shape;

use ad_trait::AD;

/// Computes the minimum distance separating two shapes.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn distance<T: AD>(
    pos1: &Isometry<T>,
    g1: &dyn Shape<T>,
    pos2: &Isometry<T>,
    g2: &dyn Shape<T>,
) -> Result<T, Unsupported> {
    let pos12 = pos1.inv_mul(&pos2);
    DefaultQueryDispatcher.distance(&pos12, g1, g2)
}
