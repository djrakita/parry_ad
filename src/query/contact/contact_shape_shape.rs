use crate::math::{Isometry};
use crate::query::{Contact, DefaultQueryDispatcher, QueryDispatcher, Unsupported};
use crate::shape::Shape;
use ad_trait::AD;

/// Computes one pair of contact points point between two shapes.
///
/// Returns `None` if the objects are separated by a distance greater than `prediction`.
/// The result is given in world-space.
pub fn contact<T: AD>(
    pos1: &Isometry<T>,
    g1: &dyn Shape<T>,
    pos2: &Isometry<T>,
    g2: &dyn Shape<T>,
    prediction: T,
) -> Result<Option<Contact<T>>, Unsupported> {
    let pos12 = pos1.inv_mul(pos2);
    let mut result = DefaultQueryDispatcher.contact(&pos12, g1, g2, prediction);

    if let Ok(Some(contact)) = &mut result {
        contact.transform_by_mut(pos1, pos2);
    }

    result
}
