
#[cfg(not(feature = "std"))]
use na::ComplexField; // for .abs()

#[cfg(feature = "dim2")]
use crate::query;
use crate::query::gjk::{self, CSOPoint, VoronoiSimplex};
use crate::query::{Ray, RayCast, RayIntersection};
#[cfg(all(feature = "std", feature = "dim2"))]
use crate::shape::ConvexPolygon;
#[cfg(all(feature = "std", feature = "dim3"))]
use crate::shape::ConvexPolyhedron;
use crate::shape::{Capsule, FeatureId, Segment, SupportMap};
#[cfg(feature = "dim3")]
use crate::shape::{Cone, Cylinder};

use ad_trait::AD;

/// Cast a ray on a shape using the GJK algorithm.
pub fn local_ray_intersection_with_support_map_with_params<G: ?Sized, T: AD>(
    shape: &G,
    simplex: &mut VoronoiSimplex<T>,
    ray: &Ray<T>,
    max_toi: T,
    solid: bool,
) -> Option<RayIntersection<T>>
where
    G: SupportMap<T>,
{
    let supp = shape.local_support_point(&-ray.dir);
    simplex.reset(CSOPoint::single_point(supp - ray.origin.coords));

    let inter = gjk::cast_local_ray(shape, simplex, ray, max_toi);

    if !solid {
        inter.and_then(|(toi, normal)| {
            if toi.is_zero() {
                // the ray is inside of the shape.
                let ndir = ray.dir.normalize();
                let supp = shape.local_support_point(&ndir);
                let eps = T::constant(0.001f64);
                let shift = (supp - ray.origin).dot(&ndir) + eps;
                let new_ray = Ray::new(ray.origin + ndir * shift, -ray.dir);

                // FIXME: replace by? : simplex.translate_by(&(ray.origin - new_ray.origin));
                simplex.reset(CSOPoint::single_point(supp - new_ray.origin.coords));

                gjk::cast_local_ray(shape, simplex, &new_ray, shift + eps).and_then(
                    |(toi, normal)| {
                        let toi = shift - toi;
                        if toi <= max_toi {
                            Some(RayIntersection::new(toi, normal, FeatureId::Unknown))
                        } else {
                            None
                        }
                    },
                )
            } else {
                Some(RayIntersection::new(toi, normal, FeatureId::Unknown))
            }
        })
    } else {
        inter.map(|(toi, normal)| RayIntersection::new(toi, normal, FeatureId::Unknown))
    }
}

#[cfg(feature = "dim3")]
impl<T: AD> RayCast<T> for Cylinder<T> {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            &ray,
            max_toi,
            solid,
        )
    }
}

#[cfg(feature = "dim3")]
impl<T: AD> RayCast<T> for Cone<T> {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            &ray,
            max_toi,
            solid,
        )
    }
}

impl<T: AD> RayCast<T> for Capsule<T> {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            &ray,
            max_toi,
            solid,
        )
    }
}

#[cfg(feature = "dim3")]
#[cfg(feature = "std")]
impl<T: AD> RayCast<T> for ConvexPolyhedron<T> {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            &ray,
            max_toi,
            solid,
        )
    }
}

#[cfg(feature = "dim2")]
#[cfg(feature = "std")]
impl<T: AD> RayCast<T> for ConvexPolygon<T> {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            &ray,
            max_toi,
            solid,
        )
    }
}

#[allow(unused_variables)]
impl<T: AD> RayCast<T> for Segment<T> {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray<T>,
        max_toi: T,
        solid: bool,
    ) -> Option<RayIntersection<T>> {
        #[cfg(feature = "dim2")]
        {
            use crate::math::Vector;

            let seg_dir = self.scaled_direction();
            let (s, t, parallel) = query::details::closest_points_line_line_parameters_eps(
                &ray.origin,
                &ray.dir,
                &self.a,
                &seg_dir,
                T::constant(crate::math::DEFAULT_EPSILON),
            );

            if parallel {
                // The lines are parallel, we have to distinguish
                // the case where there is no intersection at all
                // from the case where the line are collinear.
                let dpos = self.a - ray.origin;
                let normal = self.normal().map(|n| *n).unwrap_or_else(Vector::zeros);

                if dpos.dot(&normal).abs() < crate::math::DEFAULT_EPSILON {
                    // The rays and the segment are collinear.
                    let dist1 = dpos.dot(&ray.dir);
                    let dist2 = dist1 + seg_dir.dot(&ray.dir);

                    match (dist1 >= T::zero(), dist2 >= T::zero()) {
                        (true, true) => {
                            let toi = dist1.min(dist2) / ray.dir.norm_squared();
                            if toi > max_toi {
                                None
                            } else if dist1 <= dist2 {
                                Some(RayIntersection::new(toi, normal, FeatureId::Vertex(0)))
                            } else {
                                Some(RayIntersection::new(
                                    dist2 / ray.dir.norm_squared(),
                                    normal,
                                    FeatureId::Vertex(1),
                                ))
                            }
                        }
                        (true, false) | (false, true) => {
                            // The ray origin lies on the segment.
                            Some(RayIntersection::new(T::zero(), normal, FeatureId::Face(0)))
                        }
                        (false, false) => {
                            // The segment is behind the ray.
                            None
                        }
                    }
                } else {
                    // The rays never intersect.
                    None
                }
            } else if s >= T::zero() && s <= max_toi && t >= T::zero() && t <= T::one() {
                let normal = self.normal().map(|n| *n).unwrap_or_else(Vector::zeros);

                if normal.dot(&ray.dir) > T::zero() {
                    Some(RayIntersection::new(s, -normal, FeatureId::Face(1)))
                } else {
                    Some(RayIntersection::new(s, normal, FeatureId::Face(0)))
                }
            } else {
                // The closest points are outside of
                // the ray or segment bounds.
                None
            }
        }
        #[cfg(feature = "dim3")]
        {
            // XXX: implement an analytic solution for 3D too.
            local_ray_intersection_with_support_map_with_params(
                self,
                &mut VoronoiSimplex::new(),
                &ray,
                max_toi,
                solid,
            )
        }
    }
}
