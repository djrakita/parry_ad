use na::Unit;

use crate::math::{Isometry, Point, Vector};
#[cfg(feature = "std")]
use crate::query::epa::EPA;
use crate::query::gjk::{self, CSOPoint, ConstantOrigin, VoronoiSimplex};
use crate::query::{PointProjection, PointQuery};
#[cfg(feature = "dim2")]
#[cfg(feature = "std")]
use crate::shape::ConvexPolygon;
#[cfg(feature = "dim3")]
#[cfg(feature = "std")]
use crate::shape::ConvexPolyhedron;
use crate::shape::{FeatureId, SupportMap};
use ad_trait::AD;


/// Projects a point on a shape using the GJK algorithm.
pub fn local_point_projection_on_support_map<G, T: AD>(
    shape: &G,
    simplex: &mut VoronoiSimplex<T>,
    point: &Point<T>,
    solid: bool,
) -> PointProjection<T>
where
    G: SupportMap<T>,
{
    let m = Isometry::new(-point.coords, na::zero());
    let m_inv = Isometry::new(point.coords, na::zero());
    let dir = Unit::try_new(-m.translation.vector, T::constant(crate::math::DEFAULT_EPSILON))
        .unwrap_or(Vector::x_axis());
    let support_point = CSOPoint::from_shapes(&m_inv, shape, &ConstantOrigin, &dir);

    simplex.reset(support_point);

    if let Some(proj) = gjk::project_origin(&m, shape, simplex) {
        PointProjection::new(false, proj)
    } else if solid {
        PointProjection::new(true, *point)
    } else {
        let mut epa = EPA::new();
        if let Some(pt) = epa.project_origin(&m, shape, simplex) {
            return PointProjection::new(true, pt);
        } else {
            // return match minkowski_sampling::project_origin(&m, shape, simplex) {
            //     Some(p) => PointProjection::new(true, p + point.coords),
            //     None => PointProjection::new(true, *point),
            // };

            //// All failed.
            PointProjection::new(true, *point)
        }
    }
}

#[cfg(feature = "dim3")]
impl<T: AD> PointQuery<T> for ConvexPolyhedron<T> {
    #[inline]
    fn project_local_point(&self, point: &Point<T>, solid: bool) -> PointProjection<T> {
        local_point_projection_on_support_map(self, &mut VoronoiSimplex::new(), point, solid)
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        point: &Point<T>,
    ) -> (PointProjection<T>, FeatureId) {
        let proj = self.project_local_point(point, false);
        let dpt = *point - proj.point;
        let local_dir = if proj.is_inside { -dpt } else { dpt };

        if let Some(local_dir) = Unit::try_new(local_dir, T::constant(crate::math::DEFAULT_EPSILON)) {
            let feature = self.support_feature_id_toward(&local_dir);
            (proj, feature)
        } else {
            (proj, FeatureId::Unknown)
        }
    }
}

#[cfg(feature = "dim2")]
impl<T: AD> PointQuery<T> for ConvexPolygon<T> {
    #[inline]
    fn project_local_point(&self, point: &Point<T>, solid: bool) -> PointProjection {
        local_point_projection_on_support_map(self, &mut VoronoiSimplex::new(), point, solid)
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        point: &Point<T>,
    ) -> (PointProjection, FeatureId) {
        let proj = self.project_local_point(point, false);
        let dpt = *point - proj.point;
        let local_dir = if proj.is_inside { -dpt } else { dpt };

        if let Some(local_dir) = Unit::try_new(local_dir, crate::math::DEFAULT_EPSILON) {
            let feature = self.support_feature_id_toward(&local_dir);
            (proj, feature)
        } else {
            (proj, FeatureId::Unknown)
        }
    }
}
