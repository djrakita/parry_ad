use ad_trait::AD;
use crate::bounding_volume::Aabb;
use crate::math::{Point, Vector};
use crate::query::{PointProjection, PointQuery, PointQueryWithLocation};
use crate::shape::{FeatureId, GenericHeightField, HeightFieldStorage, TrianglePointLocation};
#[cfg(not(feature = "std"))]
use na::ComplexField; // For sqrt.

impl<Storage: HeightFieldStorage<T>, T: AD> PointQuery for GenericHeightField<Storage, T> {
    fn project_local_point_with_max_dist(
        &self,
        pt: &Point<T>,
        solid: bool,
        max_dist: T,
    ) -> Option<PointProjection> {
        let aabb = Aabb::new(pt - Vector::repeat(max_dist), pt + Vector::repeat(max_dist));
        let mut sq_smallest_dist = T::constant(f64::MAX);
        let mut best_proj = None;

        self.map_elements_in_local_aabb(&aabb, &mut |_, triangle| {
            let proj = triangle.project_local_point(pt, solid);
            let sq_dist = na::distance_squared(pt, &proj.point);

            if sq_dist < sq_smallest_dist {
                sq_smallest_dist = sq_dist;

                if sq_dist.sqrt() <= max_dist {
                    best_proj = Some(proj);
                }
            }
        });

        best_proj
    }

    #[inline]
    fn project_local_point(&self, point: &Point<T>, _: bool) -> PointProjection {
        let mut smallest_dist = T::constant(f64::MAX);
        let mut best_proj = PointProjection::new(false, *point);

        #[cfg(feature = "dim2")]
        let iter = self.segments();
        #[cfg(feature = "dim3")]
        let iter = self.triangles();
        for elt in iter {
            let proj = elt.project_local_point(point, false);
            let dist = na::distance_squared(point, &proj.point);

            if dist < smallest_dist {
                smallest_dist = dist;
                best_proj = proj;
            }
        }

        best_proj
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        point: &Point<T>,
    ) -> (PointProjection, FeatureId) {
        // FIXME: compute the feature properly.
        (self.project_local_point(point, false), FeatureId::Unknown)
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_local_point(&self, _point: &Point<T>) -> bool {
        false
    }
}

impl<Storage: HeightFieldStorage<T>, T: AD> PointQueryWithLocation for GenericHeightField<Storage, T> {
    type Location = (usize, TrianglePointLocation<T>);

    #[inline]
    fn project_local_point_and_get_location(
        &self,
        _point: &Point<T>,
        _: bool,
    ) -> (PointProjection, Self::Location) {
        unimplemented!()
    }
}
