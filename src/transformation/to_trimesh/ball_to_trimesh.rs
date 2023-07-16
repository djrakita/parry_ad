use crate::math::{Point, Vector, DIM};
use crate::shape::Ball;
use crate::transformation::utils;
use na::{self, ComplexField, Point3, RealField};
use ad_trait::AD;

impl<T: AD> Ball<T> {
    /// Discretize the boundary of this ball as a triangle-mesh.
    pub fn to_trimesh(
        &self,
        ntheta_subdiv: u32,
        nphi_subdiv: u32,
    ) -> (Vec<Point3<T>>, Vec<[u32; 3]>) {
        let diameter = self.radius * T::constant(2.0);
        let (vtx, idx) = unit_sphere(ntheta_subdiv, nphi_subdiv);
        (utils::scaled(vtx, Vector::repeat(diameter)), idx)
    }
}

fn unit_sphere<T: AD>(ntheta_subdiv: u32, nphi_subdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 3]>) {
    let dtheta = T::constant(f64::two_pi() / (ntheta_subdiv as f64));
    let dphi = T::constant(f64::pi() / (nphi_subdiv as f64));

    let mut coords = Vec::new();
    let mut curr_phi = T::constant(-f64::frac_pi_2()) + dphi;

    coords.push(Point::new(T::zero(), T::constant(-1.0), T::zero()));

    for _ in 1..nphi_subdiv {
        utils::push_circle(
            ComplexField::cos(curr_phi),
            ntheta_subdiv,
            dtheta,
            ComplexField::sin(curr_phi),
            &mut coords,
        );
        curr_phi = curr_phi + dphi;
    }

    coords.push(Point::new(T::zero(), T::one(), T::zero()));

    let mut idx = Vec::new();

    utils::push_degenerate_top_ring_indices(1, 0, ntheta_subdiv, &mut idx);
    utils::reverse_clockwising(&mut idx);

    for i in 0..nphi_subdiv - 2 {
        utils::push_ring_indices(
            1 + i * ntheta_subdiv,
            1 + (i + 1) * ntheta_subdiv,
            ntheta_subdiv,
            &mut idx,
        );
    }

    utils::push_degenerate_top_ring_indices(
        coords.len() as u32 - 1 - ntheta_subdiv,
        coords.len() as u32 - 1,
        ntheta_subdiv,
        &mut idx,
    );

    (utils::scaled(coords, Vector::repeat(T::constant(0.5))), idx)
}

/// Creates an hemisphere with a diameter of 1.
pub(crate) fn unit_hemisphere<T: AD>(
    ntheta_subdiv: u32,
    nphi_subdiv: u32,
) -> (Vec<Point<T>>, Vec<[u32; DIM]>) {
    let two_pi = T::constant(f64::two_pi());
    let pi_two = T::constant(f64::frac_pi_2());
    let dtheta = two_pi / T::constant(ntheta_subdiv as f64);
    let dphi = pi_two / T::constant(nphi_subdiv as f64);

    let mut coords = Vec::new();
    let mut curr_phi = T::zero();

    for _ in 0..nphi_subdiv {
        utils::push_circle(
            ComplexField::cos(curr_phi),
            ntheta_subdiv,
            dtheta,
            ComplexField::sin(curr_phi),
            &mut coords,
        );
        curr_phi = curr_phi + dphi;
    }

    coords.push(Point::new(T::zero(), T::one(), T::zero()));

    let mut idx = Vec::new();

    for i in 0..nphi_subdiv - 1 {
        utils::push_ring_indices(
            i * ntheta_subdiv,
            (i + 1) * ntheta_subdiv,
            ntheta_subdiv,
            &mut idx,
        );
    }

    utils::push_degenerate_top_ring_indices(
        (nphi_subdiv - 1) * ntheta_subdiv,
        coords.len() as u32 - 1,
        ntheta_subdiv,
        &mut idx,
    );

    (utils::scaled(coords, Vector::repeat(T::constant(0.5))), idx)
}
