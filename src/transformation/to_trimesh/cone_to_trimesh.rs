use crate::shape::Cone;
use crate::transformation::utils;
use na::{self, Point3, RealField, Vector3};
use ad_trait::AD;

impl<T: AD> Cone<T> {
    /// Discretize the boundary of this cone as a triangle-mesh.
    pub fn to_trimesh(&self, nsubdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 3]>) {
        let diameter = self.radius * T::constant(2.0);
        let height = self.half_height * T::constant(2.0);
        let scale = Vector3::new(diameter, height, diameter);
        let (vtx, idx) = unit_cone(nsubdiv);
        (utils::scaled(vtx, scale), idx)
    }
}

/// Generates a cone with unit height and diameter.
fn unit_cone<T: AD>(nsubdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 3]>) {
    let two_pi = T::constant(f64::two_pi());
    let dtheta = two_pi / T::constant(nsubdiv as f64);
    let mut coords = Vec::new();
    let mut indices = Vec::new();

    utils::push_circle(
        na::convert(T::constant(0.5)),
        nsubdiv,
        dtheta,
        na::convert(T::constant(-0.5)),
        &mut coords,
    );

    coords.push(Point3::new(T::zero(), T::constant(0.5), T::zero()));

    utils::push_degenerate_top_ring_indices(0, coords.len() as u32 - 1, nsubdiv, &mut indices);
    utils::push_filled_circle_indices(0, nsubdiv, &mut indices);

    (coords, indices)
}
