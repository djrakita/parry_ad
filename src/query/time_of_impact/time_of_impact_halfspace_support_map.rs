use crate::math::{Isometry, Vector};
use crate::query::{Ray, RayCast, TOIStatus, TOI};
use crate::shape::{HalfSpace, SupportMap};
use ad_trait::AD;

/// Time Of Impact of a halfspace with a support-mapped shape under translational movement.
pub fn time_of_impact_halfspace_support_map<G: ?Sized, T: AD>(
    pos12: &Isometry<T>,
    vel12: &Vector<T>,
    halfspace: &HalfSpace<T>,
    other: &G,
    max_toi: T,
    stop_at_penetration: bool,
) -> Option<TOI<T>>
where
    G: SupportMap<T>,
{
    // FIXME: add method to get only the local support point.
    // This would avoid the `inverse_transform_point` later.
    if !stop_at_penetration && vel12.dot(&halfspace.normal) > T::zero() {
        return None;
    }

    let support_point = other.support_point(pos12, &-halfspace.normal);
    let closest_point = support_point;
    let ray = Ray::new(closest_point, *vel12);

    if let Some(toi) = halfspace.cast_local_ray(&ray, max_toi, true) {
        if toi > max_toi {
            return None;
        }

        let status;
        let witness2 = support_point;
        let mut witness1 = ray.point_at(toi);

        if support_point.coords.dot(&halfspace.normal) < T::zero() {
            status = TOIStatus::Penetrating
        } else {
            // Project the witness point to the halfspace.
            // Note that witness2 is already in the halfspace's local-space.
            witness1 = witness1 - *halfspace.normal * witness1.coords.dot(&halfspace.normal);
            status = TOIStatus::Converged
        }

        Some(TOI {
            toi,
            normal1: halfspace.normal,
            normal2: pos12.inverse_transform_unit_vector(&-halfspace.normal),
            witness1,
            witness2: pos12.inverse_transform_point(&witness2),
            status,
        })
    } else {
        None
    }
}

/// Time Of Impact of a halfspace with a support-mapped shape under translational movement.
pub fn time_of_impact_support_map_halfspace<G: ?Sized, T: AD>(
    pos12: &Isometry<T>,
    vel12: &Vector<T>,
    other: &G,
    halfspace: &HalfSpace<T>,
    max_toi: T,
    stop_at_penetration: bool,
) -> Option<TOI<T>>
where
    G: SupportMap<T>,
{
    time_of_impact_halfspace_support_map(
        &pos12.inverse(),
        &-pos12.inverse_transform_vector(&vel12),
        halfspace,
        other,
        max_toi,
        stop_at_penetration,
    )
    .map(|toi| toi.swapped())
}
