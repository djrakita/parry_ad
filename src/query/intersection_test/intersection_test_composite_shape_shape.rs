#![allow(deprecated)] // Silence warning until we actually remove IntersectionCompositeShapeShapeBestFirstVisitor

use crate::bounding_volume::SimdAabb;
use crate::math::{Isometry, Vector, SIMD_WIDTH};
use crate::partitioning::{
    SimdBestFirstVisitStatus, SimdBestFirstVisitor, SimdVisitStatus, SimdVisitor,
};
use crate::query::QueryDispatcher;
use crate::shape::{Shape, TypedSimdCompositeShape};
use crate::utils::{DefaultStorage, IsometryOpt};
use simba::simd::{SimdBool as _, SimdValue};
use ad_trait::AD;

/// Intersection test between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn intersection_test_composite_shape_shape<D: ?Sized, G1: ?Sized, T: AD>(
    dispatcher: &D,
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &dyn Shape<T>,
) -> bool
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    let mut visitor = IntersectionCompositeShapeShapeVisitor::new(dispatcher, pos12, g1, g2);

    let _ = g1.typed_qbvh().traverse_depth_first(&mut visitor);
    visitor.found_intersection
}

/// Proximity between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn intersection_test_shape_composite_shape<D: ?Sized, G2: ?Sized, T: AD>(
    dispatcher: &D,
    pos12: &Isometry<T>,
    g1: &dyn Shape<T>,
    g2: &G2,
) -> bool
where
    D: QueryDispatcher<T>,
    G2: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    intersection_test_composite_shape_shape(dispatcher, &pos12.inverse(), g2, g1)
}

/// A visitor for checking if a composite-shape and a shape intersect.
pub struct IntersectionCompositeShapeShapeVisitor<'a, D: ?Sized, G1: ?Sized + 'a, T: AD> {
    ls_aabb2: SimdAabb<T>,

    dispatcher: &'a D,
    pos12: &'a Isometry<T>,
    g1: &'a G1,
    g2: &'a dyn Shape<T>,

    found_intersection: bool,
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> IntersectionCompositeShapeShapeVisitor<'a, D, G1, T>
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    /// Initialize a visitor for checking if a composite-shape and a shape intersect.
    pub fn new(
        dispatcher: &'a D,
        pos12: &'a Isometry<T>,
        g1: &'a G1,
        g2: &'a dyn Shape<T>,
    ) -> IntersectionCompositeShapeShapeVisitor<'a, D, G1, T> {
        let ls_aabb2 = g2.compute_aabb(&pos12);

        IntersectionCompositeShapeShapeVisitor {
            dispatcher,
            ls_aabb2: SimdAabb::splat(ls_aabb2),
            pos12,
            g1,
            g2,
            found_intersection: false,
        }
    }
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> SimdVisitor<G1::PartId, SimdAabb<T>>
    for IntersectionCompositeShapeShapeVisitor<'a, D, G1,T>
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    fn visit(
        &mut self,
        bv: &SimdAabb<T>,
        data: Option<[Option<&G1::PartId>; SIMD_WIDTH]>,
    ) -> SimdVisitStatus {
        let mask = self.ls_aabb2.intersects(bv);

        if let Some(data) = data {
            let bitmask = mask.bitmask();
            let mut found_intersection = false;

            for ii in 0..SIMD_WIDTH {
                if (bitmask & (1 << ii)) != 0 && data[ii].is_some() {
                    let part_id = *data[ii].unwrap();
                    self.g1.map_untyped_part_at(part_id, |part_pos1, g1| {
                        found_intersection = self.dispatcher.intersection_test(
                            &part_pos1.inv_mul(self.pos12),
                            g1,
                            self.g2,
                        ) == Ok(true);
                    });

                    if found_intersection {
                        self.found_intersection = true;
                        return SimdVisitStatus::ExitEarly;
                    }
                }
            }
        }

        SimdVisitStatus::MaybeContinue(mask)
    }
}

/// A visitor for checking if a composite-shape and a shape intersect.
#[deprecated(note = "Use IntersectionCompositeShapeShapeVisitor instead.")]
pub struct IntersectionCompositeShapeShapeBestFirstVisitor<'a, D: ?Sized, G1: ?Sized + 'a, T: AD> {
    msum_shift: Vector<T>,
    msum_margin: Vector<T>,

    dispatcher: &'a D,
    pos12: &'a Isometry<T>,
    g1: &'a G1,
    g2: &'a dyn Shape<T>,
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> IntersectionCompositeShapeShapeBestFirstVisitor<'a, D, G1, T>
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    /// Initialize a visitor for checking if a composite-shape and a shape intersect.
    pub fn new(
        dispatcher: &'a D,
        pos12: &'a Isometry<T>,
        g1: &'a G1,
        g2: &'a dyn Shape<T>,
    ) -> IntersectionCompositeShapeShapeBestFirstVisitor<'a, D, G1, T> {
        let ls_aabb2 = g2.compute_aabb(&pos12);

        IntersectionCompositeShapeShapeBestFirstVisitor {
            dispatcher,
            msum_shift: Vector::splat(-ls_aabb2.center().coords),
            msum_margin: Vector::splat(ls_aabb2.half_extents()),
            pos12,
            g1,
            g2,
        }
    }
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> SimdBestFirstVisitor<G1::PartId, SimdAabb<T>, T>
    for IntersectionCompositeShapeShapeBestFirstVisitor<'a, D, G1, T>
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    type Result = (G1::PartId, bool);

    fn visit(
        &mut self,
        best: T,
        bv: &SimdAabb<T>,
        data: Option<[Option<&G1::PartId>; SIMD_WIDTH]>,
    ) -> SimdBestFirstVisitStatus<Self::Result, T> {
        // Compute the minkowski sum of the two Aabbs.
        let msum = SimdAabb {
            mins: bv.mins + self.msum_shift + (-self.msum_margin),
            maxs: bv.maxs + self.msum_shift + self.msum_margin,
        };
        let dist = msum.distance_to_origin();
        let mask = dist.simd_lt(best);

        if let Some(data) = data {
            let bitmask = mask.bitmask();
            let mut found_intersection = false;

            for ii in 0..SIMD_WIDTH {
                if (bitmask & (1 << ii)) != 0 && data[ii].is_some() {
                    let part_id = *data[ii].unwrap();
                    self.g1.map_untyped_part_at(part_id, |part_pos1, g1| {
                        found_intersection = self.dispatcher.intersection_test(
                            &part_pos1.inv_mul(self.pos12),
                            g1,
                            self.g2,
                        ) == Ok(true);
                    });

                    if found_intersection {
                        return SimdBestFirstVisitStatus::ExitEarly(Some((part_id, true)));
                    }
                }
            }
        }

        SimdBestFirstVisitStatus::MaybeContinue {
            weights: dist,
            mask,
            results: [None; SIMD_WIDTH],
        }
    }
}
