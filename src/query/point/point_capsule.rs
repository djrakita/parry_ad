use crate::approx::AbsDiffEq;
use crate::math::{Point, Vector};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{Capsule, FeatureId, Segment};
use na::{self, Unit};
use ad_trait::AD;

impl<T: AD> PointQuery<T> for Capsule<T> {
    #[inline]
    fn project_local_point(&self, pt: &Point<T>, solid: bool) -> PointProjection<T> {
        let seg = Segment::new(self.segment.a, self.segment.b);
        let proj = seg.project_local_point(pt, solid);
        let dproj = *pt - proj.point;

        if let Some((dir, dist)) = Unit::try_new_and_get(dproj, T::constant(f64::default_epsilon())) {
            let inside = dist <= self.radius;
            if solid && inside {
                return PointProjection::new(true, *pt);
            } else {
                return PointProjection::new(inside, proj.point + dir.into_inner() * self.radius);
            }
        } else if solid {
            return PointProjection::new(true, *pt);
        }

        #[cfg(feature = "dim2")]
        if let Some(dir) = seg.normal() {
            PointProjection::new(true, proj.point + *dir * self.radius)
        } else {
            // The segment has no normal, likely because it degenerates to a point.
            PointProjection::new(true, proj.point + Vector::ith(1, self.radius))
        }

        #[cfg(feature = "dim3")]
        if let Some(dir) = seg.direction() {
            use crate::utils::WBasis;
            let dir = dir.orthonormal_basis()[0];
            PointProjection::new(true, proj.point + dir * self.radius)
        } else {
            // The segment has no normal, likely because it degenerates to a point.
            PointProjection::new(true, proj.point + Vector::ith(1, self.radius))
        }
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        pt: &Point<T>,
    ) -> (PointProjection<T>, FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Face(0))
    }
}
