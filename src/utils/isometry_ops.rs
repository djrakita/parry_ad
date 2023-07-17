use crate::math::{Isometry, Point, Vector};
use na::Unit; // for .abs()
use ad_trait::AD;

#[cfg(not(feature = "std"))]
use na::ComplexField;

/// Extra operations with isometries.
pub trait IsometryOps<T: AD> {
    /// Transform a vector by the absolute value of the homogeneous matrix
    /// equivalent to `self`.
    fn absolute_transform_vector(&self, v: &Vector<T>) -> Vector<T>;
}

impl<T: AD> IsometryOps<T> for Isometry<T> {
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<T>) -> Vector<T> {
        self.rotation.to_rotation_matrix().into_inner().abs() * *v
    }
}

/*
impl<T: AD> IsometryOps<T> for Isometry<T> {
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<T>) -> Vector<T> {
        self.rotation
            .to_rotation_matrix()
            .into_inner()
            .map(|e| e.simd_abs())
            * *v
    }
}
*/

/// Various operations usable with `Option<Isometry>` and `Option<&Isometry>`
/// where `None` is assumed to be equivalent to the identity.
pub trait IsometryOpt<T: AD> {
    /// Computes `self.inverse() * rhs`.
    fn inv_mul(self, rhs: &Isometry<T>) -> Isometry<T>;
    /// Computes `rhs * self`.
    fn prepend_to(self, rhs: &Isometry<T>) -> Isometry<T>;
    /// Computes `self * p`.
    fn transform_point(self, p: &Point<T>) -> Point<T>;
    /// Computes `self * v`.
    fn transform_vector(self, v: &Vector<T>) -> Vector<T>;
    /// Computes `self * v`.
    fn transform_unit_vector(self, v: &Unit<Vector<T>>) -> Unit<Vector<T>>;
    /// Computes `self.inverse() * p`.
    fn inverse_transform_point(self, p: &Point<T>) -> Point<T>;
    /// Computes `self.inverse() * v`.
    fn inverse_transform_vector(self, v: &Vector<T>) -> Vector<T>;
    /// Computes `self.inverse() * v`.
    fn inverse_transform_unit_vector(self, v: &Unit<Vector<T>>) -> Unit<Vector<T>>;
}

impl<'a, T: AD> IsometryOpt<T> for Option<&'a Isometry<T>> {
    #[inline]
    fn inv_mul(self, rhs: &Isometry<T>) -> Isometry<T> {
        if let Some(iso) = self {
            iso.inv_mul(&rhs)
        } else {
            *rhs
        }
    }

    #[inline]
    fn prepend_to(self, rhs: &Isometry<T>) -> Isometry<T> {
        if let Some(iso) = self {
            rhs * iso
        } else {
            *rhs
        }
    }

    #[inline]
    fn transform_point(self, p: &Point<T>) -> Point<T> {
        if let Some(iso) = self {
            iso * p
        } else {
            *p
        }
    }

    #[inline]
    fn transform_vector(self, v: &Vector<T>) -> Vector<T> {
        if let Some(iso) = self {
            iso * v
        } else {
            *v
        }
    }

    #[inline]
    fn transform_unit_vector(self, v: &Unit<Vector<T>>) -> Unit<Vector<T>> {
        if let Some(iso) = self {
            iso * v
        } else {
            *v
        }
    }

    #[inline]
    fn inverse_transform_point(self, p: &Point<T>) -> Point<T> {
        if let Some(iso) = self {
            iso.inverse_transform_point(p)
        } else {
            *p
        }
    }

    #[inline]
    fn inverse_transform_vector(self, v: &Vector<T>) -> Vector<T> {
        if let Some(iso) = self {
            iso.inverse_transform_vector(v)
        } else {
            *v
        }
    }

    #[inline]
    fn inverse_transform_unit_vector(self, v: &Unit<Vector<T>>) -> Unit<Vector<T>> {
        if let Some(iso) = self {
            iso.inverse_transform_unit_vector(v)
        } else {
            *v
        }
    }
}

impl<T: AD> IsometryOpt<T> for Option<Isometry<T>> {
    #[inline]
    fn inv_mul(self, rhs: &Isometry<T>) -> Isometry<T> {
        if let Some(iso) = self {
            iso.inv_mul(&rhs)
        } else {
            *rhs
        }
    }

    #[inline]
    fn prepend_to(self, rhs: &Isometry<T>) -> Isometry<T> {
        if let Some(iso) = self {
            rhs * iso
        } else {
            *rhs
        }
    }

    #[inline]
    fn transform_point(self, p: &Point<T>) -> Point<T> {
        if let Some(iso) = self {
            iso * p
        } else {
            *p
        }
    }

    #[inline]
    fn transform_vector(self, v: &Vector<T>) -> Vector<T> {
        if let Some(iso) = self {
            iso * v
        } else {
            *v
        }
    }

    #[inline]
    fn transform_unit_vector(self, v: &Unit<Vector<T>>) -> Unit<Vector<T>> {
        if let Some(iso) = self {
            iso * v
        } else {
            *v
        }
    }

    #[inline]
    fn inverse_transform_point(self, p: &Point<T>) -> Point<T> {
        if let Some(iso) = self {
            iso.inverse_transform_point(p)
        } else {
            *p
        }
    }

    #[inline]
    fn inverse_transform_vector(self, v: &Vector<T>) -> Vector<T> {
        if let Some(iso) = self {
            iso.inverse_transform_vector(v)
        } else {
            *v
        }
    }

    #[inline]
    fn inverse_transform_unit_vector(self, v: &Unit<Vector<T>>) -> Unit<Vector<T>> {
        if let Some(iso) = self {
            iso.inverse_transform_unit_vector(v)
        } else {
            *v
        }
    }
}
