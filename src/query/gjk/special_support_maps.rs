use na::Unit;

use crate::math::{Isometry, Point, Vector};
use crate::shape::SupportMap;
use ad_trait::AD;

/// A support mapping that is a single point.
pub struct ConstantPoint<T: AD>(pub Point<T>);

impl<T: AD> SupportMap<T> for ConstantPoint<T> {
    #[inline]
    fn support_point(&self, m: &Isometry<T>, _: &Vector<T>) -> Point<T> {
        m * self.0
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<T>, _: &Unit<Vector<T>>) -> Point<T> {
        m * self.0
    }

    #[inline]
    fn local_support_point(&self, _: &Vector<T>) -> Point<T> {
        self.0
    }

    #[inline]
    fn local_support_point_toward(&self, _: &Unit<Vector<T>>) -> Point<T> {
        self.0
    }
}

/// A support mapping that is the point at (0.0, 0.0, 0.0).
pub struct ConstantOrigin;

impl<T: AD> SupportMap<T> for ConstantOrigin {
    #[inline]
    fn support_point(&self, m: &Isometry<T>, _: &Vector<T>) -> Point<T> {
        m.translation.vector.into()
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<T>, _: &Unit<Vector<T>>) -> Point<T> {
        m.translation.vector.into()
    }

    #[inline]
    fn local_support_point(&self, _: &Vector<T>) -> Point<T> {
        Point::origin()
    }

    #[inline]
    fn local_support_point_toward(&self, _: &Unit<Vector<T>>) -> Point<T> {
        Point::origin()
    }
}

/// The Minkowski sum of a shape and a ball.
pub struct DilatedShape<'a, S: ?Sized + SupportMap<T>, T: AD> {
    /// The shape involved in the Minkowski sum.
    pub shape: &'a S,
    /// The radius of the ball involved in the Minkoski sum.
    pub radius: T,
}

impl<'a, S: ?Sized + SupportMap<T>, T: AD> SupportMap<T> for DilatedShape<'a, S, T> {
    #[inline]
    fn support_point(&self, m: &Isometry<T>, dir: &Vector<T>) -> Point<T> {
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<T>, dir: &Unit<Vector<T>>) -> Point<T> {
        self.shape.support_point_toward(m, dir) + **dir * self.radius
    }

    #[inline]
    fn local_support_point(&self, dir: &Vector<T>) -> Point<T> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    #[inline]
    fn local_support_point_toward(&self, dir: &Unit<Vector<T>>) -> Point<T> {
        self.shape.local_support_point_toward(dir) + **dir * self.radius
    }
}
