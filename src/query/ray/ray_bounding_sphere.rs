use crate::bounding_volume::BoundingSphere;
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::Ball;
use ad_trait::AD;

impl<T: AD> RayCast<T> for BoundingSphere<T> {
    #[inline]
    fn cast_local_ray(&self, ray: &Ray<T>, max_toi: T, solid: bool) -> Option<T> {
        let centered_ray = ray.translate_by(-self.center().coords);
        Ball::new(self.radius()).cast_local_ray(&centered_ray, max_toi, solid)
    }

    #[inline]
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        let centered_ray = ray.translate_by(-self.center().coords);
        Ball::new(self.radius()).cast_local_ray_and_get_normal(&centered_ray, max_toi, solid)
    }

    #[inline]
    fn intersects_local_ray(&self, ray: &Ray<T>, max_toi: T) -> bool {
        let centered_ray = ray.translate_by(-self.center().coords);
        Ball::new(self.radius()).intersects_local_ray(&centered_ray, max_toi)
    }
}
