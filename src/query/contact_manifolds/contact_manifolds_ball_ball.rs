use crate::math::{Isometry, Vector};
use crate::query::{ContactManifold, TrackedContact};
use crate::shape::{Ball, PackedFeatureId, Shape};
use ad_trait::AD;

/// Computes the contact manifold between two balls given as `Shape` trait-objects.
pub fn contact_manifold_ball_ball_shapes<T: AD, ManifoldData, ContactData: Default + Copy>(
    pos12: &Isometry<T>,
    shape1: &dyn Shape<T>,
    shape2: &dyn Shape<T>,
    prediction: T,
    manifold: &mut ContactManifold<T, ManifoldData, ContactData>,
) {
    if let (Some(ball1), Some(ball2)) = (shape1.as_ball(), shape2.as_ball()) {
        contact_manifold_ball_ball(pos12, ball1, ball2, prediction, manifold);
    }
}

/// Computes the contact manifold between two balls.
pub fn contact_manifold_ball_ball<T: AD, ManifoldData, ContactData: Default + Copy>(
    pos12: &Isometry<T>,
    ball1: &Ball<T>,
    ball2: &Ball<T>,
    prediction: T,
    manifold: &mut ContactManifold<T, ManifoldData, ContactData>,
) {
    let radius_a = ball1.radius;
    let radius_b = ball2.radius;

    let dcenter = pos12.translation.vector;
    let center_dist = dcenter.magnitude();
    let dist = center_dist - radius_a - radius_b;

    if dist < prediction {
        let local_n1 = if center_dist != T::zero() {
            dcenter / center_dist
        } else {
            Vector::y()
        };

        let local_n2 = pos12.inverse_transform_vector(&-local_n1);
        let local_p1 = local_n1 * radius_a;
        let local_p2 = local_n2 * radius_b;
        let fid = PackedFeatureId::face(0);
        let contact = TrackedContact::new(local_p1.into(), local_p2.into(), fid, fid, dist);

        if manifold.points.len() != 0 {
            manifold.points[0].copy_geometry_from(contact);
        } else {
            manifold.points.push(contact);
        }

        manifold.local_n1 = local_n1;
        manifold.local_n2 = local_n2;
    } else {
        manifold.points.clear();
    }
}
