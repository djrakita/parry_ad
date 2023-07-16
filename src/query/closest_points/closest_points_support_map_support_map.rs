use crate::math::{Isometry, Vector};
use crate::query::gjk::{self, CSOPoint, GJKResult, VoronoiSimplex};
use crate::query::ClosestPoints;
use crate::shape::SupportMap;
use ad_trait::AD;

use na::Unit;

/// Closest points between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn closest_points_support_map_support_map<T: AD, G1: ?Sized, G2: ?Sized>(
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &G2,
    prediction: T,
) -> ClosestPoints<T>
where
    G1: SupportMap<T>,
    G2: SupportMap<T>,
{
    match closest_points_support_map_support_map_with_params(
        pos12,
        g1,
        g2,
        prediction,
        &mut VoronoiSimplex::new(),
        None,
    ) {
        GJKResult::ClosestPoints(pt1, pt2, _) => {
            ClosestPoints::WithinMargin(pt1, pos12.inverse_transform_point(&pt2))
        }
        GJKResult::NoIntersection(_) => ClosestPoints::Disjoint,
        GJKResult::Intersection => ClosestPoints::Intersecting,
        GJKResult::Proximity(_) => unreachable!(),
    }
}

/// Closest points between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn closest_points_support_map_support_map_with_params<T: AD, G1: ?Sized, G2: ?Sized>(
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &G2,
    prediction: T,
    simplex: &mut VoronoiSimplex,
    init_dir: Option<Vector<T>>,
) -> GJKResult
where
    G1: SupportMap<T>,
    G2: SupportMap<T>,
{
    let dir = match init_dir {
        // FIXME: or pos12.translation.vector (without the minus sign) ?
        None => -pos12.translation.vector,
        Some(dir) => dir,
    };

    if let Some(dir) = Unit::try_new(dir, crate::math::DEFAULT_EPSILON) {
        simplex.reset(CSOPoint::from_shapes(pos12, g1, g2, &dir));
    } else {
        simplex.reset(CSOPoint::from_shapes(
            pos12,
            g1,
            g2,
            &Vector::<T>::x_axis(),
        ));
    }

    gjk::closest_points(pos12, g1, g2, prediction, true, simplex)
}
