use na::{self, ComplexField};

use crate::math::{Point};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{Ball, FeatureId};

use ad_trait::AD;

impl<T: AD> PointQuery<T> for Ball<T> {
    #[inline]
    fn project_local_point(&self, pt: &Point<T>, solid: bool) -> PointProjection<T> {
        let distance_squared = pt.coords.norm_squared();

        let inside = distance_squared <= self.radius * self.radius;

        if inside && solid {
            PointProjection::new(true, *pt)
        } else {
            let proj =
                Point::from(pt.coords * (self.radius / ComplexField::sqrt(distance_squared)));
            PointProjection::new(inside, proj)
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
        let dist = pt.coords.norm() - self.radius;

        if solid && dist < T::zero() {
            T::zero()
        } else {
            dist
        }
    }

    #[inline]
    fn contains_local_point(&self, pt: &Point<T>) -> bool {
        pt.coords.norm_squared() <= self.radius * self.radius
    }
}
