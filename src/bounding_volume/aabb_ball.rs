use crate::bounding_volume::Aabb;
use crate::math::{Isometry, Point, Vector};
use crate::shape::Ball;
use ad_trait::AD;

/// Computes the Axis-Aligned Bounding Box of a ball transformed by `center`.
#[inline]
pub fn ball_aabb<T: AD>(center: &Point<T>, radius: T) -> Aabb<T> {
    Aabb::new(
        *center + Vector::repeat(-radius),
        *center + Vector::repeat(radius),
    )
}

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn local_ball_aabb<T: AD>(radius: T) -> Aabb<T> {
    let half_extents = Point::from(Vector::repeat(radius));

    Aabb::new(-half_extents, half_extents)
}

impl<T: AD> Ball<T> {
    /// Computes the world-space Aabb of this ball transformed by `pos`.
    #[inline]
    pub fn aabb(&self, pos: &Isometry<T>) -> Aabb<T> {
        ball_aabb(&Point::<T>::from(pos.translation.vector), self.radius)
    }

    /// Computes the local-space Aabb of this ball.
    #[inline]
    pub fn local_aabb(&self) -> Aabb<T> {
        local_ball_aabb(self.radius)
    }
}
