use crate::bounding_volume::Aabb;
use crate::math::{Point};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::Cuboid;
use ad_trait::AD;

impl<T: AD> RayCast<T> for Cuboid<T> {
    #[inline]
    fn cast_local_ray(&self, ray: &Ray<T>, max_toi: T, solid: bool) -> Option<T> {
        let dl = Point::from(-self.half_extents);
        let ur = Point::from(self.half_extents);
        Aabb::new(dl, ur).cast_local_ray(ray, max_toi, solid)
    }

    #[inline]
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        let dl = Point::from(-self.half_extents);
        let ur = Point::from(self.half_extents);
        Aabb::new(dl, ur).cast_local_ray_and_get_normal(ray, max_toi, solid)
    }
}
