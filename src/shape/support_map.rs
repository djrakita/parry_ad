//! Traits for support mapping based shapes.

use crate::math::{Isometry, Point, Vector};
use na::Unit;

use ad_trait::AD;

/// Traits of convex shapes representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returned point.
pub trait SupportMap<T: AD> {
    // Evaluates the support function of this shape.
    //
    // A support function is a function associating a vector to the shape point which maximizes
    // their dot product.
    fn local_support_point(&self, dir: &Vector<T>) -> Point<T>;

    /// Same as `self.local_support_point` except that `dir` is normalized.
    fn local_support_point_toward(&self, dir: &Unit<Vector<T>>) -> Point<T> {
        self.local_support_point(dir.as_ref())
    }

    // Evaluates the support function of this shape transformed by `transform`.
    //
    // A support function is a function associating a vector to the shape point which maximizes
    // their dot product.
    fn support_point(&self, transform: &Isometry<T>, dir: &Vector<T>) -> Point<T> {
        let local_dir = transform.inverse_transform_vector(dir);
        transform * self.local_support_point(&local_dir)
    }

    /// Same as `self.support_point` except that `dir` is normalized.
    fn support_point_toward(
        &self,
        transform: &Isometry<T>,
        dir: &Unit<Vector<T>>,
    ) -> Point<T> {
        let local_dir = Unit::new_unchecked(transform.inverse_transform_vector(dir));
        transform * self.local_support_point_toward(&local_dir)
    }
}
