use na;

use crate::math::{Point, Vector};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::{FeatureId, HalfSpace};
use ad_trait::AD;

/// Computes the toi of an unbounded line with a halfspace described by its center and normal.
#[inline]
pub fn line_toi_with_halfspace<T: AD>(
    halfspace_center: &Point<T>,
    halfspace_normal: &Vector<T>,
    line_origin: &Point<T>,
    line_dir: &Vector<T>,
) -> Option<T> {
    let dpos = *halfspace_center - *line_origin;
    let denom = halfspace_normal.dot(line_dir);

    if relative_eq!(denom, T::zero()) {
        None
    } else {
        Some(halfspace_normal.dot(&dpos) / denom)
    }
}

/// Computes the toi of a ray with a halfspace described by its center and normal.
#[inline]
pub fn ray_toi_with_halfspace<T: AD>(
    center: &Point<T>,
    normal: &Vector<T>,
    ray: &Ray<T>,
) -> Option<T> {
    if let Some(t) = line_toi_with_halfspace(center, normal, &ray.origin, &ray.dir) {
        if t >= T::zero() {
            return Some(t);
        }
    }

    None
}

impl<T: AD> RayCast<T> for HalfSpace<T> {
    #[inline]
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        let dpos = -ray.origin;

        let dot_normal_dpos = self.normal.dot(&dpos.coords);

        if solid && dot_normal_dpos > T::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(T::zero(), na::zero(), FeatureId::Face(0)));
        }

        let t = dot_normal_dpos / self.normal.dot(&ray.dir);

        if t >= T::zero() && t <= max_toi {
            let n = if dot_normal_dpos > T::zero() {
                -self.normal
            } else {
                self.normal
            };

            Some(RayIntersection::new(t, *n, FeatureId::Face(0)))
        } else {
            None
        }
    }
}
