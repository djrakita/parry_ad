use ad_trait::AD;
use crate::bounding_volume::SimdAabb;
use crate::math::{Isometry, Real, SimdBool, SimdReal, Vector, SIMD_WIDTH};
use crate::partitioning::{SimdBestFirstVisitStatus, SimdBestFirstVisitor};
use crate::query::{ClosestPoints, QueryDispatcher};
use crate::shape::{Shape, TypedSimdCompositeShape};
use crate::utils::{DefaultStorage, IsometryOpt};
use na;
use simba::simd::{SimdBool as _, SimdPartialOrd, SimdValue};

/// Closest points between a composite shape and any other shape.
pub fn closest_points_composite_shape_shape<D: ?Sized, G1: ?Sized, T: AD>(
    dispatcher: &D,
    pos12: &Isometry<Real>,
    g1: &G1,
    g2: &dyn Shape,
    margin: Real,
) -> ClosestPoints
where
    D: QueryDispatcher,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    let mut visitor =
        CompositeShapeAgainstShapeClosestPointsVisitor::new(dispatcher, pos12, g1, g2, margin);

    g1.typed_qbvh()
        .traverse_best_first(&mut visitor)
        .expect("The composite shape must not be empty.")
        .1
         .1
}

/// Closest points between a shape and a composite shape.
pub fn closest_points_shape_composite_shape<D: ?Sized, G2: ?Sized, T: AD>(
    dispatcher: &D,
    pos12: &Isometry<T>,
    g1: &dyn Shape,
    g2: &G2,
    margin: T,
) -> ClosestPoints
where
    D: QueryDispatcher,
    G2: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    closest_points_composite_shape_shape(dispatcher, &pos12.inverse(), g2, g1, margin).flipped()
}

/// A visitor for computing the closest points between a composite-shape and a shape.
pub struct CompositeShapeAgainstShapeClosestPointsVisitor<'a, D: ?Sized, G1: ?Sized + 'a, T: AD> {
    msum_shift: Vector<T>,
    msum_margin: Vector<T>,
    margin: T,

    dispatcher: &'a D,
    pos12: &'a Isometry<T>,
    g1: &'a G1,
    g2: &'a dyn Shape,
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> CompositeShapeAgainstShapeClosestPointsVisitor<'a, D, G1, T>
where
    D: QueryDispatcher,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    /// Initializes a visitor for computing the closest points between a composite-shape and a shape.
    pub fn new(
        dispatcher: &'a D,
        pos12: &'a Isometry<T>,
        g1: &'a G1,
        g2: &'a dyn Shape,
        margin: T,
    ) -> CompositeShapeAgainstShapeClosestPointsVisitor<'a, D, G1, T> {
        let ls_aabb2 = g2.compute_aabb(pos12);

        CompositeShapeAgainstShapeClosestPointsVisitor {
            msum_shift: Vector::splat(-ls_aabb2.center().coords),
            msum_margin: Vector::splat(ls_aabb2.half_extents()),
            margin,
            dispatcher,
            pos12,
            g1,
            g2,
        }
    }
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> SimdBestFirstVisitor<G1::PartId, SimdAabb<T>>
    for CompositeShapeAgainstShapeClosestPointsVisitor<'a, D, G1, T>
where
    D: QueryDispatcher,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    type Result = (G1::PartId, ClosestPoints);

    fn visit(
        &mut self,
        best: T,
        bv: &SimdAabb<T>,
        data: Option<[Option<&G1::PartId>; SIMD_WIDTH]>,
    ) -> SimdBestFirstVisitStatus<Self::Result> {
        // Compute the minkowski sum of the two Aabbs.
        let msum = SimdAabb {
            mins: bv.mins + self.msum_shift + (-self.msum_margin),
            maxs: bv.maxs + self.msum_shift + self.msum_margin,
        };
        let dist = msum.distance_to_origin();
        let mask = dist.simd_lt(SimdReal::splat(best));

        if let Some(data) = data {
            let bitmask = mask.bitmask();
            let mut weights = [0.0; SIMD_WIDTH];
            let mut mask = [false; SIMD_WIDTH];
            let mut results = [None; SIMD_WIDTH];
            let mut found_intersection = false;

            for ii in 0..SIMD_WIDTH {
                if (bitmask & (1 << ii)) != 0 && data[ii].is_some() {
                    let part_id = *data[ii].unwrap();
                    self.g1.map_untyped_part_at(part_id, |part_pos1, g1| {
                        let pts = self.dispatcher.closest_points(
                            &part_pos1.inv_mul(self.pos12),
                            g1,
                            self.g2,
                            self.margin,
                        );
                        match pts {
                            Ok(ClosestPoints::WithinMargin(ref p1, ref p2)) => {
                                let p1 = part_pos1.transform_point(p1);
                                let p2_1 = self.pos12 * p2;
                                weights[ii] = na::distance(&p1, &p2_1);
                                results[ii] = Some((part_id, ClosestPoints::WithinMargin(p1, *p2)));
                                mask[ii] = true;
                            }
                            Ok(ClosestPoints::Intersecting) => {
                                found_intersection = true;
                            }
                            Err(_) | Ok(ClosestPoints::Disjoint) => {}
                        };
                    });

                    if found_intersection {
                        return SimdBestFirstVisitStatus::ExitEarly(Some((
                            part_id,
                            ClosestPoints::Intersecting,
                        )));
                    }
                }
            }

            SimdBestFirstVisitStatus::MaybeContinue {
                weights: SimdReal::from(weights),
                mask: SimdBool::from(mask),
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
