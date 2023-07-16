use crate::bounding_volume::SimdAabb;
use crate::math::{Point, SIMD_WIDTH};
use crate::partitioning::{SimdBestFirstVisitStatus, SimdBestFirstVisitor};
use crate::query::{PointProjection, PointQuery};
use crate::shape::SimdCompositeShape;
use na;
use simba::simd::{SimdBool as _, SimdPartialOrd, SimdValue};
use ad_trait::AD;

/// Best-first traversal visitor for computing the point closest to a composite shape.
pub struct CompositeClosestPointVisitor<'a, S: 'a, T: AD> {
    shape: &'a S,
    point: &'a Point<T>,
    simd_point: Point<T>,
    solid: bool,
}

impl<'a, S, T: AD> CompositeClosestPointVisitor<'a, S, T> {
    /// Initializes a visitor that allows the computation of the point closest to `point` on `shape`.
    pub fn new(shape: &'a S, point: &'a Point<T>, solid: bool) -> Self {
        CompositeClosestPointVisitor {
            shape,
            point,
            simd_point: Point::splat(*point),
            solid,
        }
    }
}

impl<'a, S: SimdCompositeShape<T> + PointQuery<T>, T: AD> SimdBestFirstVisitor<u32, SimdAabb<T>, T>
    for CompositeClosestPointVisitor<'a, S, T>
{
    type Result = PointProjection<T>;

    #[inline]
    fn visit(
        &mut self,
        best: T,
        aabb: &SimdAabb<T>,
        data: Option<[Option<&u32>; SIMD_WIDTH]>,
    ) -> SimdBestFirstVisitStatus<Self::Result, T> {
        let dist = aabb.distance_to_local_point(&self.simd_point);
        let mask = dist.simd_lt(best);

        if let Some(data) = data {
            let bitmask = mask.bitmask();
            let mut weights = [T::zero(); SIMD_WIDTH];
            let mut mask = [false; SIMD_WIDTH];
            let mut results = [None; SIMD_WIDTH];

            for ii in 0..SIMD_WIDTH {
                if (bitmask & (1 << ii)) != 0 && data[ii].is_some() {
                    self.shape
                        .map_part_at(*data[ii].unwrap(), &mut |part_pos, obj| {
                            let proj = if let Some(part_pos) = part_pos {
                                obj.project_point(part_pos, self.point, self.solid)
                            } else {
                                obj.project_local_point(self.point, self.solid)
                            };

                            weights[ii] = na::distance(self.point, &proj.point);
                            mask[ii] = true;
                            results[ii] = Some(proj);
                        });
                }
            }

            SimdBestFirstVisitStatus::MaybeContinue {
                weights: weights[0],
                mask: mask[0],
                results,
            }
        } else {
            SimdBestFirstVisitStatus::MaybeContinue {
                weights: dist,
                mask,
                results: [None; SIMD_WIDTH],
            }
        }
    }
}
