use crate::math::{Isometry, Point};
use crate::query::{ContactManifold, TrackedContact};
use crate::shape::{Ball, PackedFeatureId, Shape};
use na::Unit;
use ad_trait::AD;

/// Computes the contact manifold between a convex shape and a ball, both represented as a `Shape` trait-object.
pub fn contact_manifold_convex_ball_shapes<ManifoldData, ContactData, T: AD>(
    pos12: &Isometry<T>,
    shape1: &dyn Shape<T>,
    shape2: &dyn Shape<T>,
    prediction: T,
    manifold: &mut ContactManifold<ManifoldData, ContactData, T>,
) where
    ContactData: Default + Copy,
{
    if let Some(ball1) = shape1.as_ball() {
        contact_manifold_convex_ball(&pos12.inverse(), shape2, ball1, prediction, manifold, true);
    } else if let Some(ball2) = shape2.as_ball() {
        contact_manifold_convex_ball(pos12, shape1, ball2, prediction, manifold, false);
    }
}

/// Computes the contact manifold between a convex shape and a ball.
pub fn contact_manifold_convex_ball<'a, ManifoldData, ContactData, S1, T: AD>(
    pos12: &Isometry<T>,
    shape1: &'a S1,
    ball2: &'a Ball<T>,
    prediction: T,
    manifold: &mut ContactManifold<ManifoldData, ContactData, T>,
    flipped: bool,
) where
    S1: ?Sized + Shape<T>,
    ContactData: Default + Copy,
{
    let local_p2_1 = Point::from(pos12.translation.vector);
    let (proj, fid1) = shape1.project_local_point_and_get_feature(&local_p2_1);
    let dpos = local_p2_1 - proj.point;

    if let Some((mut local_n1, mut dist)) = Unit::try_new_and_get(dpos, T::zero()) {
        if proj.is_inside {
            local_n1 = -local_n1;
            dist = -dist;
        }

        if dist <= ball2.radius + prediction {
            let local_n2 = pos12.inverse_transform_vector(&-*local_n1);
            let local_p2 = (local_n2 * ball2.radius).into();
            let contact_point = TrackedContact::flipped(
                proj.point,
                local_p2,
                fid1.into(),
                PackedFeatureId::face(0),
                dist - ball2.radius,
                flipped,
            );

            if manifold.points.len() != 1 {
                manifold.clear();
                manifold.points.push(contact_point);
            } else {
                // Copy only the geometry so we keep the warmstart impulses.
                manifold.points[0].copy_geometry_from(contact_point);
            }

            if flipped {
                manifold.local_n1 = local_n2;
                manifold.local_n2 = *local_n1;
            } else {
                manifold.local_n1 = *local_n1;
                manifold.local_n2 = local_n2;
            }
        } else {
            manifold.clear();
        }
    }
}
