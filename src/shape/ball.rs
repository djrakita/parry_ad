#[cfg(feature = "std")]
use either::Either;
use na::Unit;

use crate::math::{Isometry, Point, Real, Vector};
use crate::shape::SupportMap;

use ad_trait::AD;

/// A Ball shape.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(check_bytes)
)]
#[cfg_attr(feature = "cuda", derive(cust_core::DeviceCopy))]
#[derive(PartialEq, Debug, Copy, Clone)]
#[repr(C)]
pub struct Ball<T: AD> {
    /// The radius of the ball.
    pub radius: T,
}

impl<T: AD> Ball<T> {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: T) -> Ball<T> {
        Ball { radius }
    }

    /// Computes a scaled version of this ball.
    ///
    /// If the scaling factor is non-uniform, then it can’t be represented as
    /// ball. Instead, a convex polygon approximation (with `nsubdivs`
    /// subdivisions) is returned. Returns `None` if that approximation had degenerate
    /// normals (for example if the scaling factor along one axis is zero).
    #[cfg(all(feature = "dim2", feature = "std"))]
    #[inline]
    pub fn scaled(
        self,
        scale: &Vector<T>,
        nsubdivs: u32,
    ) -> Option<Either<Self, super::ConvexPolygon>> {
        if scale.x != scale.y {
            // The scaled shape isn’t a ball.
            let mut vtx = self.to_polyline(nsubdivs);
            vtx.iter_mut()
                .for_each(|pt| pt.coords = pt.coords.component_mul(&scale));
            Some(Either::Right(super::ConvexPolygon::from_convex_polyline(
                vtx,
            )?))
        } else {
            let uniform_scale = scale.x;
            Some(Either::Left(Self::new(self.radius * uniform_scale.abs())))
        }
    }

    /// Computes a scaled version of this ball.
    ///
    /// If the scaling factor is non-uniform, then it can’t be represented as
    /// ball. Instead, a convex polygon approximation (with `nsubdivs`
    /// subdivisions) is returned. Returns `None` if that approximation had degenerate
    /// normals (for example if the scaling factor along one axis is zero).
    #[cfg(all(feature = "dim3", feature = "std"))]
    #[inline]
    pub fn scaled(
        self,
        scale: &Vector<T>,
        nsubdivs: u32,
    ) -> Option<Either<Self, super::ConvexPolyhedron>> {
        if scale.x != scale.y || scale.x != scale.z || scale.y != scale.z {
            // The scaled shape isn’t a ball.
            let (mut vtx, idx) = self.to_trimesh(nsubdivs, nsubdivs);
            vtx.iter_mut()
                .for_each(|pt| pt.coords = pt.coords.component_mul(&scale));
            Some(Either::Right(super::ConvexPolyhedron::from_convex_mesh(
                vtx, &idx,
            )?))
        } else {
            let uniform_scale = scale.x;
            Some(Either::Left(Self::new(self.radius * uniform_scale.abs())))
        }
    }
}

impl<T: AD> SupportMap for Ball<T> {
    #[inline]
    fn support_point(&self, m: &Isometry<T>, dir: &Vector<T>) -> Point<T> {
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<T>, dir: &Unit<Vector<T>>) -> Point<T> {
        Point::from(m.translation.vector) + **dir * self.radius
    }

    #[inline]
    fn local_support_point(&self, dir: &Vector<T>) -> Point<T> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    #[inline]
    fn local_support_point_toward(&self, dir: &Unit<Vector<T>>) -> Point<T> {
        Point::from(**dir * self.radius)
    }
}
