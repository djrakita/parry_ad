use crate::shape::Cylinder;
use crate::transformation::utils;
use na::{self, Point3, RealField, Vector3};
use ad_trait::AD;

impl<T: AD> Cylinder<T> {
    /// Discretize the boundary of this cylinder as a triangle-mesh.
    pub fn to_trimesh(&self, nsubdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 3]>) {
        let diameter = self.radius * T::constant(2.0);
        let height = self.half_height * T::constant(2.0);
        let scale = Vector3::new(diameter, height, diameter);
        let (vtx, idx) = unit_cylinder(nsubdiv);
        (utils::scaled(vtx, scale), idx)
    }
}

/// Generates a cylinder with unit height and diameter.
fn unit_cylinder<T: AD>(nsubdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 3]>) {
    let two_pi = T::constant(f64::two_pi());
    let invsubdiv = T::constant(1.0 / (nsubdiv as f64));
    let dtheta = two_pi * invsubdiv;
    let mut coords = Vec::new();
    let mut indices = Vec::new();

    utils::push_circle(
        na::convert(T::constant(0.5)),
        nsubdiv,
        dtheta,
        na::convert(-T::constant(0.5)),
        &mut coords,
    );

    utils::push_circle(
        na::convert(T::constant(0.5)),
        nsubdiv,
        dtheta,
        na::convert(T::constant(0.5)),
        &mut coords,
    );

    utils::push_ring_indices(0, nsubdiv, nsubdiv, &mut indices);
    utils::push_filled_circle_indices(0, nsubdiv, &mut indices);
    utils::push_filled_circle_indices(nsubdiv, nsubdiv, &mut indices);

    let len = indices.len();
    let bottom_start_id = len - (nsubdiv as usize - 2);
    utils::reverse_clockwising(&mut indices[bottom_start_id..]);

    (coords, indices)
}
