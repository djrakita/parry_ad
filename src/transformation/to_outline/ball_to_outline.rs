use crate::math::{Isometry, Point, Vector};
use crate::shape::Ball;
use crate::transformation::utils;
use na::{self, Point3, RealField};
use ad_trait::AD;

#[cfg(not(feature = "std"))]
use na::ComplexField;

impl<T: AD> Ball<T> {
    /// Outlines this ball’s shape using polylines.
    pub fn to_outline(&self, nsubdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 2]>) {
        let diameter = self.radius * T::constant(2.0);
        let (vtx, idx) = unit_sphere_outline(nsubdiv);
        (utils::scaled(vtx, Vector::repeat(diameter)), idx)
    }
}

fn unit_sphere_outline<T: AD>(nsubdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 2]>) {
    let two_pi = T::constant(f64::two_pi());
    let dtheta = two_pi / T::constant(nsubdiv as f64);
    let mut coords = Vec::new();
    let mut indices = Vec::new();

    utils::push_circle(T::constant(0.5), nsubdiv, dtheta, T::zero(), &mut coords);
    utils::push_circle(T::constant(0.5), nsubdiv, dtheta, T::zero(), &mut coords);
    utils::push_circle(T::constant(0.5), nsubdiv, dtheta, T::zero(), &mut coords);

    let n = nsubdiv as usize;
    utils::transform(
        &mut coords[n..n * 2],
        Isometry::rotation(Vector::x() * T::constant(f64::frac_pi_2())),
    );
    utils::transform(
        &mut coords[n * 2..n * 3],
        Isometry::rotation(Vector::z() * T::constant(f64::frac_pi_2())),
    );

    utils::push_circle_outline_indices(&mut indices, 0..nsubdiv);
    utils::push_circle_outline_indices(&mut indices, nsubdiv..nsubdiv * 2);
    utils::push_circle_outline_indices(&mut indices, nsubdiv * 2..nsubdiv * 3);

    (coords, indices)
}

/// Creates an hemisphere with a radius of T::constant(0.5.
pub(crate) fn push_unit_hemisphere_outline<T: AD>(
    nsubdiv: u32,
    pts: &mut Vec<Point<T>>,
    idx: &mut Vec<[u32; 2]>,
) {
    let base_idx = pts.len() as u32;
    let dtheta = T::constant(f64::pi() / (nsubdiv as f64));
    let npoints = nsubdiv + 1;

    utils::push_circle(T::constant(0.5), npoints, dtheta, T::zero(), pts);
    utils::push_circle(T::constant(0.5), npoints, dtheta, T::zero(), pts);

    let n = npoints as usize;
    utils::transform(
        &mut pts[base_idx as usize..base_idx as usize + n],
        Isometry::rotation(Vector::x() * T::constant(-f64::frac_pi_2())),
    );
    utils::transform(
        &mut pts[base_idx as usize + n..base_idx as usize + n * 2],
        Isometry::rotation(Vector::z() * T::constant(f64::frac_pi_2()))
            * Isometry::rotation(Vector::y() * T::constant(f64::frac_pi_2())),
    );

    utils::push_open_circle_outline_indices(idx, base_idx..base_idx + npoints);
    utils::push_open_circle_outline_indices(idx, base_idx + npoints..base_idx + npoints * 2);
}
