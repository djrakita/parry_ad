use crate::math::{Point, Vector};
use crate::query::Ray;
use simba::simd::SimdValue;
use ad_trait::AD;

/// A structure representing 4 rays in an SIMD SoA fashion.
#[derive(Debug, Copy, Clone)]
pub struct SimdRay<T: AD> {
    /// The origin of the rays represented as a single SIMD point.
    pub origin: Point<T>,
    /// The direction of the rays represented as a single SIMD vector.
    pub dir: Vector<T>,
}

impl<T: AD> SimdRay<T> {
    /// Creates a new SIMD ray with all its lanes filled with the same ray.
    pub fn splat(ray: Ray<T>) -> Self {
        Self {
            origin: ray.origin,
            dir: ray.dir,
        }
    }
}
