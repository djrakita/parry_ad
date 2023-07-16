use crate::bounding_volume::{Aabb, SimdAabb};
use crate::math::SIMD_WIDTH;
use crate::partitioning::{SimdVisitStatus, SimdVisitor};
use simba::simd::SimdBool as _;
use std::marker::PhantomData;
use ad_trait::AD;

/// Spatial partitioning data structure visitor collecting interferences with a given bounding volume.
pub struct BoundingVolumeIntersectionsVisitor<T, F, A: AD> {
    bv: SimdAabb<A>,
    callback: F,
    _phantom: PhantomData<T>,
}

impl<T, F, A: AD> BoundingVolumeIntersectionsVisitor<T, F, A>
where
    F: FnMut(&T) -> bool,
{
    /// Creates a new `BoundingVolumeIntersectionsVisitor`.
    #[inline]
    pub fn new(bv: &Aabb<A>, callback: F) -> BoundingVolumeIntersectionsVisitor<T, F, A> {
        BoundingVolumeIntersectionsVisitor {
            bv: SimdAabb::splat(*bv),
            callback,
            _phantom: PhantomData,
        }
    }
}

impl<T, F, A: AD> SimdVisitor<T, SimdAabb<A>> for BoundingVolumeIntersectionsVisitor<T, F, A>
where
    F: FnMut(&T) -> bool,
{
    #[inline]
    fn visit(&mut self, bv: &SimdAabb<A>, b: Option<[Option<&T>; SIMD_WIDTH]>) -> SimdVisitStatus {
        let mask = bv.intersects(&self.bv);

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
