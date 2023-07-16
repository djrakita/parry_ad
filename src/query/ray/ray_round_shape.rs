use crate::query::gjk::VoronoiSimplex;
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::{RoundShape, SupportMap};
use ad_trait::AD;

impl<S: SupportMap<T>, T: AD> RayCast<T> for RoundShape<S, T> {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        crate::query::details::local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            ray,
            max_toi,
            solid,
        )
    }
}
