use ad_trait::AD;
use crate::math::{Isometry, Point, Vector};
use crate::shape::{Cuboid, SupportMap};

use na::Unit;

// NOTE: this only works with cuboid on the rhs because it has its symmetry origin at zero
// (therefore we can check only one normal direction).
/// Computes the separation between a point and a cuboid, along the given direction `normal1`.
pub fn point_cuboid_find_local_separating_normal_oneway<T: AD>(
    point1: Point<T>,
    normal1: Option<Unit<Vector<T>>>,
    shape2: &Cuboid<T>,
    pos12: &Isometry<T>,
) -> (T, Vector<T>) {
    let mut best_separation = T::constant(-f64::MAX);
    let mut best_dir = Vector::zeros();

    if let Some(normal1) = normal1 {
        let axis1 = if (pos12.translation.vector - point1.coords).dot(&normal1) >= T::zero() {
            normal1
        } else {
            -normal1
        };

        let pt2 = shape2.support_point_toward(&pos12, &-axis1);
        let separation = (pt2 - point1).dot(&axis1);

        if separation > best_separation {
            best_separation = separation;
            best_dir = *axis1;
        }
    }

    (best_separation, best_dir)
}
