use crate::bounding_volume::SimdAabb;
use crate::math::{Point, SIMD_WIDTH};
use crate::partitioning::{SimdVisitStatus, SimdVisitor};
use simba::simd::{SimdBool as _, SimdValue};
use std::marker::PhantomData;
use ad_trait::AD;

// FIXME: add a point cost fn.

/// Spatial partitioning structure visitor collecting nodes that may contain a given point.
pub struct PointIntersectionsVisitor<'a, T, F, A: AD> {
    simd_point: Point<A>,
    /// Callback executed for each leaf which Aabb contains `self.point`.
    callback: &'a mut F,
    _phantom: PhantomData<T>,
}

impl<'a, T, F, A: AD> PointIntersectionsVisitor<'a, T, F, A>
where
    F: FnMut(&T) -> bool,
{
    /// Creates a new `PointIntersectionsVisitor`.
    #[inline]
    pub fn new(point: &'a Point<A>, callback: &'a mut F) -> PointIntersectionsVisitor<'a, T, F, A> {
        PointIntersectionsVisitor {
            simd_point: Point::splat(*point),
            callback,
            _phantom: PhantomData,
        }
    }
}

impl<'a, T, F, A: AD> SimdVisitor<T, SimdAabb<A>> for PointIntersectionsVisitor<'a, T, F, A>
where
    F: FnMut(&T) -> bool,
{
    #[inline]
    fn visit(&mut self, bv: &SimdAabb<A>, b: Option<[Option<&T>; SIMD_WIDTH]>) -> SimdVisitStatus {
        let mask = bv.contains_local_point(&self.simd_point);

        if let Some(data) = b {
            let bitmask = mask.bitmask();
            for ii in 0..SIMD_WIDTH {
                if (bitmask & (1 << ii)) != 0 && data[ii].is_some() {
                    if !(self.callback)(data[ii].unwrap()) {
                        return SimdVisitStatus::ExitEarly;
                    }
                }
            }
        }

        SimdVisitStatus::MaybeContinue(mask)
    }
}
