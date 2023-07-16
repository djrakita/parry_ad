use crate::math::{Isometry, Point, Vector};
use crate::shape::SupportMap;
use na::Unit;
use std::ops::Sub;
use ad_trait::AD;

/// A point of a Configuration-Space Obstacle.
///
/// A Configuration-Space Obstacle (CSO) is the result of the
/// Minkowski Difference of two solids. In other words, each of its
/// points correspond to the difference of two point, each belonging
/// to a different solid.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CSOPoint<T: AD> {
    /// The point on the CSO. This is equal to `self.orig1 - self.orig2`, unless this CSOPoint
    /// has been translated with self.translate.
    pub point: Point<T>,
    /// The original point on the first shape used to compute `self.point`.
    pub orig1: Point<T>,
    /// The original point on the second shape used to compute `self.point`.
    pub orig2: Point<T>,
}

impl<T: AD> CSOPoint<T> {
    /// Initializes a CSO point with `orig1 - orig2`.
    pub fn new(orig1: Point<T>, orig2: Point<T>) -> Self {
        let point = Point::from(orig1 - orig2);
        Self::new_with_point(point, orig1, orig2)
    }

    /// Initializes a CSO point with all information provided.
    ///
    /// It is assumed, but not checked, that `point == orig1 - orig2`.
    pub fn new_with_point(point: Point<T>, orig1: Point<T>, orig2: Point<T>) -> Self {
        CSOPoint {
            point,
            orig1,
            orig2,
        }
    }

    /// Initializes a CSO point where both original points are equal.
    pub fn single_point(point: Point<T>) -> Self {
        Self::new_with_point(point, point, Point::origin())
    }

    /// CSO point where all components are set to zero.
    pub fn origin() -> Self {
        CSOPoint::new(Point::origin(), Point::origin())
    }

    /// Computes the support point of the CSO of `g1` and `g2` toward the unit direction `dir`.
    pub fn from_shapes_toward<G1: ?Sized, G2: ?Sized>(
        pos12: &Isometry<T>,
        g1: &G1,
        g2: &G2,
        dir: &Unit<Vector<T>>,
    ) -> Self
    where
        G1: SupportMap<T>,
        G2: SupportMap<T>,
    {
        let sp1 = g1.local_support_point_toward(dir);
        let sp2 = g2.support_point_toward(pos12, &-*dir);

        CSOPoint::new(sp1, sp2)
    }

    /// Computes the support point of the CSO of `g1` and `g2` toward the direction `dir`.
    pub fn from_shapes<G1: ?Sized, G2: ?Sized>(
        pos12: &Isometry<T>,
        g1: &G1,
        g2: &G2,
        dir: &Vector<T>,
    ) -> Self
    where
        G1: SupportMap<T>,
        G2: SupportMap<T>,
    {
        let sp1 = g1.local_support_point(dir);
        let sp2 = g2.support_point(pos12, &-*dir);

        CSOPoint::new(sp1, sp2)
    }

    /// Translate the CSO point.
    pub fn translate(&self, dir: &Vector<T>) -> Self {
        CSOPoint::new_with_point(self.point + dir, self.orig1, self.orig2)
    }

    /// Translate in-place the CSO point.
    pub fn translate_mut(&mut self, dir: &Vector<T>) {
        self.point += dir;
    }
}

impl<T> Sub<CSOPoint<T>> for CSOPoint<T> {
    type Output = Vector<T>;

    #[inline]
    fn sub(self, rhs: CSOPoint<T>) -> Vector<T> {
        self.point - rhs.point
    }
}
