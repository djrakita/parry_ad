use crate::bounding_volume::Aabb;
use crate::math::{Point};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{Cuboid, FeatureId};
use ad_trait::AD;

impl<T: AD> PointQuery for Cuboid<T> {
    #[inline]
    fn project_local_point(&self, pt: &Point<T>, solid: bool) -> PointProjection {
        let dl = Point::from(-self.half_extents);
        let ur = Point::from(self.half_extents);
        Aabb::new(dl, ur).project_local_point(pt, solid)
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        pt: &Point<T>,
    ) -> (PointProjection, FeatureId) {
        let dl = Point::from(-self.half_extents);
        let ur = Point::from(self.half_extents);
        Aabb::new(dl, ur).project_local_point_and_get_feature(pt)
    }

    #[inline]
    fn distance_to_local_point(&self, pt: &Point<T>, solid: bool) -> T {
        let dl = Point::from(-self.half_extents);
        let ur = Point::from(self.half_extents);
        Aabb::new(dl, ur).distance_to_local_point(pt, solid)
    }

    #[inline]
    fn contains_local_point(&self, pt: &Point<T>) -> bool {
        let dl = Point::from(-self.half_extents);
        let ur = Point::from(self.half_extents);
        Aabb::new(dl, ur).contains_local_point(pt)
    }
}
