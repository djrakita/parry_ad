use crate::math::{Point};
use crate::query::gjk::VoronoiSimplex;
use crate::query::{PointProjection, PointQuery};
use crate::shape::{FeatureId, RoundShape, SupportMap};
use ad_trait::AD;

// TODO: if PointQuery had a `project_point_with_normal` method, we could just
// call this and adjust the projected point accordingly.
impl<S: SupportMap<T>, T: AD> PointQuery<T> for RoundShape<S, T> {
    #[inline]
    fn project_local_point(&self, point: &Point<T>, solid: bool) -> PointProjection<T> {
        #[cfg(not(feature = "std"))] // FIXME: can’t be used without std because of EPA
        return unimplemented!(
            "The projection of points on a round shapes isn’t supported on no-std platforms yet."
        );

        #[cfg(feature = "std")] // FIXME: can’t be used without std because of EPA
        return crate::query::details::local_point_projection_on_support_map(
            self,
            &mut VoronoiSimplex::new(),
            point,
            solid,
        );
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        point: &Point<T>,
    ) -> (PointProjection<T>, FeatureId) {
        (self.project_local_point(point, false), FeatureId::Unknown)
    }
}
