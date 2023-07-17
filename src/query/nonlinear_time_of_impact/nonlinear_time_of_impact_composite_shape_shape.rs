use ad_trait::AD;
use crate::bounding_volume::{BoundingSphere, SimdAabb};
use crate::math::{SIMD_WIDTH};
use crate::partitioning::{SimdBestFirstVisitStatus, SimdBestFirstVisitor};
use crate::query::{self, details::NonlinearTOIMode, NonlinearRigidMotion, QueryDispatcher, TOI};
use crate::shape::{Ball, Shape, TypedSimdCompositeShape};
use crate::utils::DefaultStorage;
use simba::simd::SimdValue;

/// Time Of Impact of a composite shape with any other shape, under a rigid motion (translation + rotation).
pub fn nonlinear_time_of_impact_composite_shape_shape<D: ?Sized, G1: ?Sized, T: AD>(
    dispatcher: &D,
    motion1: &NonlinearRigidMotion<T>,
    g1: &G1,
    motion2: &NonlinearRigidMotion<T>,
    g2: &dyn Shape<T>,
    start_time: T,
    end_time: T,
    stop_at_penetration: bool,
) -> Option<TOI<T>>
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    let mut visitor = NonlinearTOICompositeShapeShapeBestFirstVisitor::new(
        dispatcher,
        motion1,
        g1,
        motion2,
        g2,
        start_time,
        end_time,
        stop_at_penetration,
    );

    g1.typed_qbvh()
        .traverse_best_first(&mut visitor)
        .map(|res| res.1 .1)
}

/// Time Of Impact of any shape with a composite shape, under a rigid motion (translation + rotation).
pub fn nonlinear_time_of_impact_shape_composite_shape<D: ?Sized, G2: ?Sized, T: AD>(
    dispatcher: &D,
    motion1: &NonlinearRigidMotion<T>,
    g1: &dyn Shape<T>,
    motion2: &NonlinearRigidMotion<T>,
    g2: &G2,
    start_time: T,
    end_time: T,
    stop_at_penetration: bool,
) -> Option<TOI<T>>
where
    D: QueryDispatcher<T>,
    G2: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    nonlinear_time_of_impact_composite_shape_shape(
        dispatcher,
        motion2,
        g2,
        motion1,
        g1,
        start_time,
        end_time,
        stop_at_penetration,
    )
    .map(|toi| toi.swapped())
}

/// A visitor used to determine the non-linear time of impact between a composite shape and another shape.
pub struct NonlinearTOICompositeShapeShapeBestFirstVisitor<'a, D: ?Sized, G1: ?Sized + 'a, T: AD> {
    sphere2: BoundingSphere<T>,
    start_time: T,
    end_time: T,
    stop_at_penetration: bool,

    dispatcher: &'a D,
    motion1: &'a NonlinearRigidMotion<T>,
    motion2: &'a NonlinearRigidMotion<T>,
    g1: &'a G1,
    g2: &'a dyn Shape<T>,
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> NonlinearTOICompositeShapeShapeBestFirstVisitor<'a, D, G1, T>
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    /// Initializes visitor used to determine the non-linear time of impact between
    /// a composite shape and another shape.
    pub fn new(
        dispatcher: &'a D,
        motion1: &'a NonlinearRigidMotion<T>,
        g1: &'a G1,
        motion2: &'a NonlinearRigidMotion<T>,
        g2: &'a dyn Shape<T>,
        start_time: T,
        end_time: T,
        stop_at_penetration: bool,
    ) -> NonlinearTOICompositeShapeShapeBestFirstVisitor<'a, D, G1, T> {
        NonlinearTOICompositeShapeShapeBestFirstVisitor {
            dispatcher,
            sphere2: g2.compute_local_bounding_sphere(),
            start_time,
            end_time,
            stop_at_penetration,
            motion1,
            motion2,
            g1,
            g2,
        }
    }
}

impl<'a, D: ?Sized, G1: ?Sized, T: AD> SimdBestFirstVisitor<G1::PartId, SimdAabb<T>, T>
    for NonlinearTOICompositeShapeShapeBestFirstVisitor<'a, D, G1, T>
where
    D: QueryDispatcher<T>,
    G1: TypedSimdCompositeShape<T, QbvhStorage = DefaultStorage>,
{
    type Result = (G1::PartId, TOI<T>);

    #[inline]
    fn visit(
        &mut self,
        best: T,
        bv: &SimdAabb<T>,
        data: Option<[Option<&G1::PartId>; SIMD_WIDTH]>,
    ) -> SimdBestFirstVisitStatus<Self::Result, T> {
        let mut weights = [T::zero(); SIMD_WIDTH];
        let mut mask = [false; SIMD_WIDTH];
        let mut results = [None; SIMD_WIDTH];

        // let centers1: [Point<Real>; SIMD_WIDTH] = bv.center().into();
        let centers1 = bv.center();
        let radius1: [T; SIMD_WIDTH] = [bv.radius()];

        for ii in 0..SIMD_WIDTH {
            let center1 = centers1.extract(ii);
            let ball1 = Ball::new(radius1[ii]);
            let ball2 = Ball::new(self.sphere2.radius());
            let ball_motion1 = self.motion1.prepend_translation(center1.coords);
            let ball_motion2 = self.motion2.prepend_translation(self.sphere2.center.coords);

            if let Some(toi) = query::details::nonlinear_time_of_impact_support_map_support_map(
                self.dispatcher,
                &ball_motion1,
                &ball1,
                &ball1,
                &ball_motion2,
                &ball2,
                &ball2,
                self.start_time,
                self.end_time,
                NonlinearTOIMode::StopAtPenetration,
            ) {
                if let Some(data) = data {
                    if toi.toi < best && data[ii].is_some() {
                        let part_id = *data[ii].unwrap();
                        self.g1.map_untyped_part_at(part_id, |part_pos1, g1| {
                            let toi = if let Some(part_pos1) = part_pos1 {
                                self.dispatcher
                                    .nonlinear_time_of_impact(
                                        &self.motion1.prepend(*part_pos1),
                                        g1,
                                        self.motion2,
                                        self.g2,
                                        self.start_time,
                                        self.end_time,
                                        self.stop_at_penetration,
                                    )
                                    .unwrap_or(None)
                                    .map(|toi| toi.transform1_by(part_pos1))
                            } else {
                                self.dispatcher
                                    .nonlinear_time_of_impact(
                                        self.motion1,
                                        g1,
                                        self.motion2,
                                        self.g2,
                                        self.start_time,
                                        self.end_time,
                                        self.stop_at_penetration,
                                    )
                                    .unwrap_or(None)
                            };

                            // println!("Found toi: {:?}", toi);

                            if let Some(toi) = toi {
                                weights[ii] = toi.toi;
                                mask[ii] = toi.toi < best;
                                results[ii] = Some((part_id, toi));
                            }
                        });
                    }
                } else {
                    weights[ii] = toi.toi;
                    mask[ii] = toi.toi < best;
                }
            }
        }

        SimdBestFirstVisitStatus::MaybeContinue {
            weights: weights[0],
            mask: mask[0],
            results,
        }
    }
}
