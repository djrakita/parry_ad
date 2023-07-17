use crate::math::{Point};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{FeatureId, HalfSpace};
use ad_trait::AD;

impl<T: AD> PointQuery<T> for HalfSpace<T> {
    #[inline]
    fn project_local_point(&self, pt: &Point<T>, solid: bool) -> PointProjection<T> {
        let d = self.normal.dot(&pt.coords);
        let inside = d <= T::zero();

        if inside && solid {
            PointProjection::new(true, *pt)
        } else {
            PointProjection::new(inside, *pt + (-*self.normal * d))
        }
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        pt: &Point<T>,
    ) -> (PointProjection<T>, FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_local_point(&self, pt: &Point<T>, solid: bool) -> T {
        let dist = self.normal.dot(&pt.coords);

        if dist < T::zero() && solid {
            T::zero()
        } else {
            // This will automatically be negative if the point is inside.
            dist
        }
    }

    #[inline]
    fn contains_local_point(&self, pt: &Point<T>) -> bool {
        self.normal.dot(&pt.coords) <= T::zero()
    }
}
