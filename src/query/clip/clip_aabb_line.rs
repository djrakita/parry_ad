use crate::bounding_volume::Aabb;
use crate::math::{Point, Vector, DIM};
use crate::query::Ray;
use crate::shape::Segment;
use num::{Bounded, Zero};
use ad_trait::AD;

impl<T: AD> Aabb<T> {
    /// Computes the intersection of a segment with this Aabb.
    ///
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_segment(&self, pa: &Point<T>, pb: &Point<T>) -> Option<Segment<T>> {
        let ab = pb - pa;
        clip_aabb_line(self, pa, &ab)
            .map(|clip| Segment::new(pa + ab * (clip.0).0.max(T::zero()), pa + ab * (clip.1).0.min(T::one())))
    }

    /// Computes the parameters of the two intersection points between a line and this Aabb.
    ///
    /// The parameters are such that the point are given by `orig + dir * parameter`.
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_line_parameters(
        &self,
        orig: &Point<T>,
        dir: &Vector<T>,
    ) -> Option<(T, T)> {
        clip_aabb_line(self, orig, dir).map(|clip| ((clip.0).0, (clip.1).0))
    }

    /// Computes the intersection segment between a line and this Aabb.
    ///
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_line(&self, orig: &Point<T>, dir: &Vector<T>) -> Option<Segment<T>> {
        clip_aabb_line(self, orig, dir)
            .map(|clip| Segment::new(orig + dir * (clip.0).0, orig + dir * (clip.1).0))
    }

    /// Computes the parameters of the two intersection points between a ray and this Aabb.
    ///
    /// The parameters are such that the point are given by `ray.orig + ray.dir * parameter`.
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_ray_parameters(&self, ray: &Ray) -> Option<(T, T)> {
        self.clip_line_parameters(&ray.origin, &ray.dir)
            .and_then(|clip| {
                let t0 = clip.0;
                let t1 = clip.1;

                if t1 < T::zero() {
                    None
                } else {
                    Some((t0.max(T::zero()), t1))
                }
            })
    }

    /// Computes the intersection segment between a ray and this Aabb.
    ///
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_ray(&self, ray: &Ray) -> Option<Segment<T>> {
        self.clip_ray_parameters(ray)
            .map(|clip| Segment::new(ray.point_at(clip.0), ray.point_at(clip.1)))
    }
}

/// Computes the segment given by the intersection of a line and an Aabb.
pub fn clip_aabb_line<T: AD>(
    aabb: &Aabb<T>,
    origin: &Point<T>,
    dir: &Vector<T>,
) -> Option<((T, Vector<T>, isize), (T, Vector<T>, isize))> {
    let mut tmax: T = Bounded::max_value();
    let mut tmin: T = -tmax;
    let mut near_side = 0;
    let mut far_side = 0;
    let mut near_diag = false;
    let mut far_diag = false;

    for i in 0usize..DIM {
        if dir[i].is_zero() {
            if origin[i] < aabb.mins[i] || origin[i] > aabb.maxs[i] {
                return None;
            }
        } else {
            let denom = T::constant(1.0) / dir[i];
            let flip_sides;
            let mut inter_with_near_halfspace = (aabb.mins[i] - origin[i]) * denom;
            let mut inter_with_far_halfspace = (aabb.maxs[i] - origin[i]) * denom;

            if inter_with_near_halfspace > inter_with_far_halfspace {
                flip_sides = true;
                std::mem::swap(
                    &mut inter_with_near_halfspace,
                    &mut inter_with_far_halfspace,
                )
            } else {
                flip_sides = false;
            }

            if inter_with_near_halfspace > tmin {
                tmin = inter_with_near_halfspace;
                near_side = if flip_sides {
                    -(i as isize + 1)
                } else {
                    i as isize + 1
                };
                near_diag = false;
            } else if inter_with_near_halfspace == tmin {
                near_diag = true;
            }

            if inter_with_far_halfspace < tmax {
                tmax = inter_with_far_halfspace;
                far_side = if !flip_sides {
                    -(i as isize + 1)
                } else {
                    i as isize + 1
                };
                far_diag = false;
            } else if inter_with_far_halfspace == tmax {
                far_diag = true;
            }

            if tmax < T::zero() || tmin > tmax {
                return None;
            }
        }
    }

    let near = if near_diag {
        (tmin, -dir.normalize(), near_side)
    } else {
        let mut normal = Vector::zeros();

        if near_side < 0 {
            normal[(-near_side - 1) as usize] = T::constant(1.0);
        } else {
            normal[(near_side - 1) as usize] = -T::constant(1.0);
        }

        (tmin, normal, near_side)
    };

    let far = if far_diag {
        (tmax, -dir.normalize(), far_side)
    } else {
        let mut normal = Vector::zeros();

        if far_side < 0 {
            normal[(-far_side - 1) as usize] = -T::constant(1.0);
        } else {
            normal[(far_side - 1) as usize] = T::constant(1.0);
        }

        (tmax, normal, far_side)
    };

    Some((near, far))
}
