use crate::bounding_volume::SimdAabb;
use crate::math::{SIMD_WIDTH};
use crate::partitioning::{SimdVisitStatus, SimdVisitor};
use crate::query::{Ray, SimdRay};
use simba::simd::{SimdBool as _};
use std::marker::PhantomData;
use ad_trait::AD;

/// Bounding Volume Tree visitor collecting intersections with a given ray.
pub struct RayIntersectionsVisitor<'a, T, F, A: AD> {
    simd_ray: SimdRay<A>,
    max_toi: A,
    callback: &'a mut F,
    _phantom: PhantomData<T>,
}

impl<'a, T, F, A: AD> RayIntersectionsVisitor<'a, T, F, A>
where
    F: FnMut(&T) -> bool,
{
    /// Creates a new `RayIntersectionsVisitor`.
    #[inline]
    pub fn new(ray: &Ray<A>, max_toi: A, callback: &'a mut F) -> RayIntersectionsVisitor<'a, T, F, A> {
        RayIntersectionsVisitor {
            simd_ray: SimdRay::splat(*ray),
            max_toi: max_toi,
            callback,
            _phantom: PhantomData,
        }
    }
}

impl<'a, T, F, A: AD> SimdVisitor<T, SimdAabb<A>> for RayIntersectionsVisitor<'a, T, F, A>
where
    F: FnMut(&T) -> bool,
{
    #[inline]
    fn visit(&mut self, bv: &SimdAabb<A>, b: Option<[Option<&T>; SIMD_WIDTH]>) -> SimdVisitStatus {
        let mask = bv.cast_local_ray(&self.simd_ray, self.max_toi).0;

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
