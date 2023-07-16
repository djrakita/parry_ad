use crate::bounding_volume::BoundingSphere;
use crate::math::{Point};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{Ball, FeatureId};
use ad_trait::AD;

impl<T: AD> PointQuery for BoundingSphere<T> {
    #[inline]
    fn project_local_point(&self, pt: &Point<T>, solid: bool) -> PointProjection {
        let centered_pt = pt - self.center().coords;
        let mut proj = Ball::new(self.radius()).project_local_point(&centered_pt, solid);

        proj.point += self.center().coords;
        proj
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        pt: &Point<T>,
    ) -> (PointProjection, FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_local_point(&self, pt: &Point<T>, solid: bool) -> T {
        let centered_pt = pt - self.center().coords;
        Ball::new(self.radius()).distance_to_local_point(&centered_pt, solid)
    }

    #[inline]
    fn contains_local_point(&self, pt: &Point<T>) -> bool {
        let centered_pt = pt - self.center().coords;
        Ball::new(self.radius()).contains_local_point(&centered_pt)
    }
}
