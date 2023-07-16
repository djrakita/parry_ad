use crate::bounding_volume::SimdAabb;
use crate::math::{Isometry, SIMD_WIDTH};
use crate::partitioning::{SimdSimultaneousVisitStatus, SimdSimultaneousVisitor};
use na::SimdValue;
use simba::simd::SimdBool as _;
use std::marker::PhantomData;

#[cfg(feature = "parallel")]
use crate::partitioning::{QbvhNode, SimdNodeIndex};

use ad_trait::AD;

/// Spatial partitioning data structure visitor collecting interferences with a given bounding volume.
pub struct BoundingVolumeIntersectionsSimultaneousVisitor<T1, T2, F, T: AD> {
    pos12: Option<Isometry<T>>,
    callback: F,
    _phantom: PhantomData<(T1, T2)>,
}

impl<T1, T2, F, T: AD> BoundingVolumeIntersectionsSimultaneousVisitor<T1, T2, F, T> {
    /// Creates a new `BoundingVolumeIntersectionsSimultaneousVisitor`.
    #[inline]
    pub fn new(callback: F) -> BoundingVolumeIntersectionsSimultaneousVisitor<T1, T2, F, T> {
        BoundingVolumeIntersectionsSimultaneousVisitor {
            pos12: None,
            callback,
            _phantom: PhantomData,
        }
    }

    /// Creates a new `BoundingVolumeIntersectionsSimultaneousVisitor`.
    #[inline]
    pub fn with_relative_pos(
        pos12: Isometry<T>,
        callback: F,
    ) -> BoundingVolumeIntersectionsSimultaneousVisitor<T1, T2, F, T> {
        BoundingVolumeIntersectionsSimultaneousVisitor {
            pos12: Some(Isometry::splat(pos12)),
            callback,
            _phantom: PhantomData,
        }
    }
}

impl<T1, T2, F, T: AD> SimdSimultaneousVisitor<T1, T2, SimdAabb<T>>
    for BoundingVolumeIntersectionsSimultaneousVisitor<T1, T2, F, T>
where
    F: FnMut(&T1, &T2) -> bool,
{
    #[inline]
    fn visit(
        &mut self,
        left_bv: &SimdAabb<T>,
        left_data: Option<[Option<&T1>; SIMD_WIDTH]>,
        right_bv: &SimdAabb<T>,
        right_data: Option<[Option<&T2>; SIMD_WIDTH]>,
    ) -> SimdSimultaneousVisitStatus {
        let mask = if let Some(pos12) = &self.pos12 {
            let transformed_right_bv = right_bv.transform_by(pos12);
            left_bv.intersects_permutations(&transformed_right_bv)
        } else {
            left_bv.intersects_permutations(right_bv)
        };

        if let (Some(data1), Some(data2)) = (left_data, right_data) {
            for ii in 0..SIMD_WIDTH {
                let bitmask = mask[ii].bitmask();

                for jj in 0..SIMD_WIDTH {
                    if (bitmask & (1 << jj)) != 0 && data1[ii].is_some() && data2[jj].is_some() {
                        if !(self.callback)(data1[ii].unwrap(), data2[jj].unwrap()) {
                            return SimdSimultaneousVisitStatus::ExitEarly;
                        }
                    }
                }
            }
        }

        SimdSimultaneousVisitStatus::MaybeContinue(mask)
    }
}

#[cfg(feature = "parallel")]
impl<LeafData1: Sync, LeafData2: Sync, F, T: AD>
    crate::partitioning::ParallelSimdSimultaneousVisitor<LeafData1, LeafData2, T>
    for BoundingVolumeIntersectionsSimultaneousVisitor<LeafData1, LeafData2, F, T>
where
    F: Sync + Fn(&LeafData1, &LeafData2) -> bool,
{
    type Data = ();

    #[inline]
    fn visit(
        &self,
        _: SimdNodeIndex,
        left_node: &QbvhNode<T>,
        left_data: Option<[Option<&LeafData1>; SIMD_WIDTH]>,
        _: SimdNodeIndex,
        right_node: &QbvhNode<T>,
        right_data: Option<[Option<&LeafData2>; SIMD_WIDTH]>,
        _: (),
    ) -> (SimdSimultaneousVisitStatus, ()) {
        let mask = if let Some(pos12) = &self.pos12 {
            let transformed_right_bv = right_node.simd_aabb.transform_by(pos12);
            left_node
                .simd_aabb
                .intersects_permutations(&transformed_right_bv)
        } else {
            left_node
                .simd_aabb
                .intersects_permutations(&right_node.simd_aabb)
        };

        if let (Some(data1), Some(data2)) = (left_data, right_data) {
            for ii in 0..SIMD_WIDTH {
                let bitmask = mask[ii].bitmask();

                for jj in 0..SIMD_WIDTH {
                    if (bitmask & (1 << jj)) != 0 && data1[ii].is_some() && data2[jj].is_some() {
                        if !(self.callback)(data1[ii].unwrap(), data2[jj].unwrap()) {
                            return (SimdSimultaneousVisitStatus::ExitEarly, ());
                        }
                    }
                }
            }
        }

        (SimdSimultaneousVisitStatus::MaybeContinue(mask), ())
    }
}
