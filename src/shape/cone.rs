//! Support mapping based Cone shape.

use ad_trait::AD;
use crate::math::{Point, Real, Vector};
use crate::shape::SupportMap;
use na;
use num::Zero;

#[cfg(feature = "std")]
use either::Either;

#[cfg(not(feature = "std"))]
use na::RealField; // for .copysign()

/// Cone shape with its principal axis aligned with the `y` axis.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(as = "Self"),
    archive(check_bytes)
)]
#[cfg_attr(feature = "cuda", derive(cust_core::DeviceCopy))]
#[derive(PartialEq, Debug, Copy, Clone)]
#[repr(C)]
pub struct Cone<T: AD> {
    /// The half-height of the cone.
    pub half_height: T,
    /// The base radius of the cone.
    pub radius: T,
}

impl<T: AD> Cone<T> {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `y` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: T, radius: T) -> Cone<T> {
        Cone {
            half_height,
            radius,
        }
    }

    /// Computes a scaled version of this cone.
    ///
    /// If the scaling factor is non-uniform, then it can’t be represented as
    /// cone. Instead, a convex polyhedral approximation (with `nsubdivs`
    /// subdivisions) is returned. Returns `None` if that approximation had degenerate
    /// normals (for example if the scaling factor along one axis is zero).
    #[cfg(feature = "std")]
    #[inline]
    pub fn scaled(
        self,
        scale: &Vector<T>,
        nsubdivs: u32,
    ) -> Option<Either<Self, super::ConvexPolyhedron>> {
        // NOTE: if the y scale is negative, the result cone points downwards,
        //       which can’t be represented with this Cone (without a transform).
        if scale.x != scale.z || scale.y < T::zero() {
            // The scaled shape isn’t a cone.
            let (mut vtx, idx) = self.to_trimesh(nsubdivs);
            vtx.iter_mut()
                .for_each(|pt| pt.coords = pt.coords.component_mul(&scale));
            Some(Either::Right(super::ConvexPolyhedron::from_convex_mesh(
                vtx, &idx,
            )?))
        } else {
            Some(Either::Left(Self::new(
                self.half_height * scale.y,
                self.radius * scale.x,
            )))
        }
    }
}

impl<T: AD> SupportMap for Cone<T> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<T>) -> Point<T> {
        let mut vres = *dir;

        vres[1] = T::zero();

        if vres.normalize_mut().is_zero() {
            vres = na::zero();
            vres[1] = self.half_height.copysign(dir[1]);
        } else {
            vres = vres * self.radius;
            vres[1] = -self.half_height;

            if dir.dot(&vres) < dir[1] * self.half_height {
                vres = na::zero();
                vres[1] = self.half_height
            }
        }

        Point::from(vres)
    }
}
