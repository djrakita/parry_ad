use crate::math::{Isometry};
use crate::query::Contact;
use crate::shape::{HalfSpace, SupportMap};
use ad_trait::AD;

/// Contact between a halfspace and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn contact_halfspace_support_map<T: AD, G: ?Sized + SupportMap<T>>(
    pos12: &Isometry<T>,
    halfspace: &HalfSpace<T>,
    other: &G,
    prediction: T,
) -> Option<Contact<T>> {
    let deepest = other.support_point_toward(&pos12, &-halfspace.normal);
    let distance = halfspace.normal.dot(&deepest.coords);

    if distance <= prediction {
        let point1 = deepest - halfspace.normal.into_inner() * distance;
        let point2 = pos12.inverse_transform_point(&deepest);
        let normal2 = pos12.inverse_transform_unit_vector(&-halfspace.normal);

        Some(Contact::new(
            point1,
            point2,
            halfspace.normal,
            normal2,
            distance,
        ))
    } else {
        None
    }
}

/// Contact between a support-mapped shape (Cuboid, ConvexHull, etc.) and a halfspace.
pub fn contact_support_map_halfspace<T: AD, G: ?Sized + SupportMap<T>>(
    pos12: &Isometry<T>,
    other: &G,
    halfspace: &HalfSpace<T>,
    prediction: T,
) -> Option<Contact<T>> {
    contact_halfspace_support_map(pos12, halfspace, other, prediction).map(|c| c.flipped())
}
