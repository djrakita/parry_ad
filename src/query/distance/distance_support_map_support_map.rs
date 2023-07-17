use crate::math::{Isometry, Vector};
use crate::query::gjk::{self, CSOPoint, GJKResult, VoronoiSimplex};
use crate::shape::SupportMap;
use ad_trait::AD;

use na::{self, Unit};
use num::Bounded;

/// Distance between support-mapped shapes.
pub fn distance_support_map_support_map<G1: ?Sized, G2: ?Sized, T: AD>(
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &G2,
) -> T
where
    G1: SupportMap<T>,
    G2: SupportMap<T>,
{
    distance_support_map_support_map_with_params(pos12, g1, g2, &mut VoronoiSimplex::new(), None)
}

/// Distance between support-mapped shapes.
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn distance_support_map_support_map_with_params<G1: ?Sized, G2: ?Sized, T: AD>(
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &G2,
    simplex: &mut VoronoiSimplex<T>,
    init_dir: Option<Vector<T>>,
) -> T
where
    G1: SupportMap<T>,
    G2: SupportMap<T>,
{
    // FIXME: or m2.translation - m1.translation ?
    let dir = init_dir.unwrap_or_else(|| -pos12.translation.vector);

    if let Some(dir) = Unit::try_new(dir, T::constant(crate::math::DEFAULT_EPSILON)) {
        simplex.reset(CSOPoint::from_shapes(pos12, g1, g2, &dir));
    } else {
        simplex.reset(CSOPoint::from_shapes(
            pos12,
            g1,
            g2,
            &Vector::<T>::x_axis(),
        ));
    }

    match gjk::closest_points(pos12, g1, g2, T::constant(f64::max_value()), true, simplex) {
        GJKResult::Intersection => T::zero(),
        GJKResult::ClosestPoints(p1, p2, _) => na::distance(&p1, &p2),
        GJKResult::Proximity(_) => unreachable!(),
        GJKResult::NoIntersection(_) => T::zero(), // FIXME: GJK did not converge.
    }
}
