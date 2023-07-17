use ad_trait::AD;
use crate::bounding_volume::Aabb;
use crate::math::{Isometry, Point};
use crate::num::Bounded;
use crate::shape::HalfSpace;

impl<T: AD> HalfSpace<T> {
    /// Computes the world-space Aabb of this half-space.
    #[inline]
    pub fn aabb(&self, _pos: &Isometry<T>) -> Aabb<T> {
        self.local_aabb()
    }

    /// Computes the local-space Aabb of this half-space.
    #[inline]
    pub fn local_aabb(&self) -> Aabb<T> {
        // We divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        // let max = Point::max_value() * T::constant(0.5f64);
        let max = Point::from_slice(&[T::constant(f64::max_value()); 3]) * T::constant(0.5f64);
        Aabb::new(-max, max)
    }
}
