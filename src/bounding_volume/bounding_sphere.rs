//! Bounding sphere.

use crate::bounding_volume::BoundingVolume;
use crate::math::{Isometry, Point};
use na;
use num::Zero;
use ad_trait::AD;

/// A Bounding Sphere.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(as = "Self"),
    archive(check_bytes)
)]
#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(C)]
pub struct BoundingSphere<T: AD> {
    pub center: Point<T>,
    pub radius: T,
}

impl<T: AD> BoundingSphere<T> {
    /// Creates a new bounding sphere.
    pub fn new(center: Point<T>, radius: T) -> BoundingSphere<T> {
        BoundingSphere { center, radius }
    }

    /// The bounding sphere center.
    #[inline]
    pub fn center(&self) -> &Point<T> {
        &self.center
    }

    /// The bounding sphere radius.
    #[inline]
    pub fn radius(&self) -> T {
        self.radius
    }

    /// Transforms this bounding sphere by `m`.
    #[inline]
    pub fn transform_by(&self, m: &Isometry<T>) -> BoundingSphere<T> {
        BoundingSphere::new(m * self.center, self.radius)
    }
}

impl<T: AD> BoundingVolume for BoundingSphere<T> {
    #[inline]
    fn center(&self) -> Point<T> {
        *self.center()
    }

    #[inline]
    fn intersects(&self, other: &BoundingSphere<T>) -> bool {
        // FIXME: refactor that with the code from narrow_phase::ball_ball::collide(...) ?
        let delta_pos = other.center - self.center;
        let distance_squared = delta_pos.norm_squared();
        let sum_radius = self.radius + other.radius;

        distance_squared <= sum_radius * sum_radius
    }

    #[inline]
    fn contains(&self, other: &BoundingSphere<T>) -> bool {
        let delta_pos = other.center - self.center;
        let distance = delta_pos.norm();

        distance + other.radius <= self.radius
    }

    #[inline]
    fn merge(&mut self, other: &BoundingSphere<T>) {
        let mut dir = *other.center() - *self.center();
        let norm = dir.normalize_mut();

        if norm.is_zero() {
            if other.radius > self.radius {
                self.radius = other.radius
            }
        } else {
            let s_center_dir = self.center.coords.dot(&dir);
            let o_center_dir = other.center.coords.dot(&dir);

            let right;
            let left;

            if s_center_dir + self.radius > o_center_dir + other.radius {
                right = self.center + dir * self.radius;
            } else {
                right = other.center + dir * other.radius;
            }

            if -s_center_dir + self.radius > -o_center_dir + other.radius {
                left = self.center - dir * self.radius;
            } else {
                left = other.center - dir * other.radius;
            }

            self.center = na::center(&left, &right);
            self.radius = na::distance(&right, &self.center);
        }
    }

    #[inline]
    fn merged(&self, other: &BoundingSphere<T>) -> BoundingSphere<T> {
        let mut res = self.clone();

        res.merge(other);

        res
    }

    #[inline]
    fn loosen(&mut self, amount: T) {
        assert!(amount >= T::zero(), "The loosening margin must be positive.");
        self.radius = self.radius + amount
    }

    #[inline]
    fn loosened(&self, amount: T) -> BoundingSphere<T> {
        assert!(amount >= T::zero(), "The loosening margin must be positive.");
        BoundingSphere::new(self.center, self.radius + amount)
    }

    #[inline]
    fn tighten(&mut self, amount: T) {
        assert!(amount >= T::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        self.radius = self.radius - amount
    }

    #[inline]
    fn tightened(&self, amount: T) -> BoundingSphere<T> {
        assert!(amount >= T::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        BoundingSphere::new(self.center, self.radius - amount)
    }
}
