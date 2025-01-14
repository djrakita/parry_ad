use crate::math::{Isometry, Point, Translation, Vector};
use ad_trait::AD;

/// A nonlinear motion from a starting isometry traveling at constant translational and rotational velocity.
#[derive(Debug, Copy, Clone)]
pub struct NonlinearRigidMotion<T: AD> {
    /// The starting isometry at `t = 0`.
    pub start: Isometry<T>,
    /// The local-space point at which the rotational part of this motion is applied.
    pub local_center: Point<T>,
    /// The translational velocity of this motion.
    pub linvel: Vector<T>,
    /// The angular velocity of this motion.
    #[cfg(feature = "dim2")]
    pub angvel: T,
    /// The angular velocity of this motion.
    #[cfg(feature = "dim3")]
    pub angvel: Vector<T>,
}

impl<T: AD> NonlinearRigidMotion<T> {
    /// Initialize a motion from a starting isometry and linear and angular velocities.
    #[cfg(feature = "dim2")]
    pub fn new(
        start: Isometry<T>,
        local_center: Point<T>,
        linvel: Vector<T>,
        angvel: T,
    ) -> Self {
        NonlinearRigidMotion {
            start,
            local_center,
            linvel,
            angvel,
        }
    }

    /// Initialize a motion from a starting isometry and linear and angular velocities.
    #[cfg(feature = "dim3")]
    pub fn new(
        start: Isometry<T>,
        local_center: Point<T>,
        linvel: Vector<T>,
        angvel: Vector<T>,
    ) -> Self {
        NonlinearRigidMotion {
            start,
            local_center,
            linvel,
            angvel,
        }
    }

    /// Create a `NonlinearRigidMotion` that always returns the identity matrix.
    pub fn identity() -> Self {
        Self::constant_position(Isometry::identity())
    }

    /// Create a `NonlinearRigidMotion` that always return `pos`.
    pub fn constant_position(pos: Isometry<T>) -> Self {
        Self {
            start: pos,
            linvel: na::zero(),
            angvel: na::zero(),
            local_center: Point::origin(),
        }
    }

    fn set_start(&mut self, new_start: Isometry<T>) {
        // NOTE: we need to adjust the local_center so that the angular
        // velocity is still expressed wrt. the original center.
        self.local_center = new_start.inverse_transform_point(&(self.start * self.local_center));
        self.start = new_start;
    }

    /// Freezes this motion at the time `t`.
    ///
    /// After calling this, any further calls to `self.position_at_time`
    /// will always return `self.position_at_time(t)` (where `t` is the value given
    /// to this method). This sets the linear velocity and angular velocity
    /// of `self` to zero.
    pub fn freeze(&mut self, t: T) {
        self.start = self.position_at_time(t);
        self.linvel = na::zero();
        self.angvel = na::zero();
    }

    /// Appends a constant translation to this rigid-motion.
    #[must_use]
    pub fn append_translation(&self, tra: Vector<T>) -> Self {
        let mut result = self.clone();
        result.set_start(Translation::from(tra) * result.start);
        result
    }

    /// Prepends a constant translation to this rigid-motion.
    #[must_use]
    pub fn prepend_translation(&self, tra: Vector<T>) -> Self {
        let mut result = self.clone();
        result.set_start(result.start * Translation::from(tra));
        result
    }

    /// Appends a constant isometry to this rigid-motion.
    #[must_use]
    pub fn append(&self, iso: Isometry<T>) -> Self {
        let mut result = self.clone();
        result.set_start(iso * result.start);
        result
    }

    /// Prepends a constant translation to this rigid-motion.
    #[must_use]
    pub fn prepend(&self, iso: Isometry<T>) -> Self {
        let mut result = self.clone();
        result.set_start(result.start * iso);
        result
    }

    /// Computes the position at time `t` of a rigid-body following the motion described by `self`.
    pub fn position_at_time(&self, t: T) -> Isometry<T> {
        let center = self.start * self.local_center;
        let shift = Translation::from(center.coords);
        (shift * Isometry::new(self.linvel * t, self.angvel * t)) * (shift.inverse() * self.start)
    }
}
