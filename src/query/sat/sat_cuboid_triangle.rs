use ad_trait::AD;
// #[cfg(feature = "dim3")]
// use crate::approx::AbsDiffEq;
use crate::math::{Isometry, Vector};
#[cfg(feature = "dim3")]
use crate::query::sat;
#[cfg(feature = "dim2")]
use crate::query::sat::support_map_support_map_compute_separation;
use crate::shape::{Cuboid, SupportMap, Triangle};

/// Finds the best separating edge between a cuboid and a triangle.
///
/// All combinations of edges from the cuboid and the triangle are taken into
/// account.
#[cfg(feature = "dim3")]
#[inline(always)]
pub fn cuboid_triangle_find_local_separating_edge_twoway<T: AD>(
    cube1: &Cuboid<T>,
    triangle2: &Triangle<T>,
    pos12: &Isometry<T>,
) -> (T, Vector<T>) {
    // NOTE: everything in this method will be expressed
    // in the local-space of the first triangle. So we
    // don't bother adding 2_1 suffixes (e.g. `a2_1`) to everything in
    // order to keep the code more readable.
    let a = pos12 * triangle2.a;
    let b = pos12 * triangle2.b;
    let c = pos12 * triangle2.c;

    let ab = b - a;
    let bc = c - b;
    let ca = a - c;

    // We have 3 * 3 = 9 axes to test.
    let axes = [
        // Vector::{x, y ,z}().cross(ab)
        Vector::new(T::zero(), -ab.z, ab.y),
        Vector::new(ab.z, T::zero(), -ab.x),
        Vector::new(-ab.y, ab.x, T::zero()),
        // Vector::{x, y ,z}().cross(bc)
        Vector::new(T::zero(), -bc.z, bc.y),
        Vector::new(bc.z, T::zero(), -bc.x),
        Vector::new(-bc.y, bc.x, T::zero()),
        // Vector::{x, y ,z}().cross(ca)
        Vector::new(T::zero(), -ca.z, ca.y),
        Vector::new(ca.z, T::zero(), -ca.x),
        Vector::new(-ca.y, ca.x, T::zero()),
    ];

    let tri_dots = [
        (axes[0].dot(&a.coords), axes[0].dot(&c.coords)),
        (axes[1].dot(&a.coords), axes[1].dot(&c.coords)),
        (axes[2].dot(&a.coords), axes[2].dot(&c.coords)),
        (axes[3].dot(&a.coords), axes[3].dot(&c.coords)),
        (axes[4].dot(&a.coords), axes[4].dot(&c.coords)),
        (axes[5].dot(&a.coords), axes[5].dot(&c.coords)),
        (axes[6].dot(&a.coords), axes[6].dot(&b.coords)),
        (axes[7].dot(&a.coords), axes[7].dot(&b.coords)),
        (axes[8].dot(&a.coords), axes[8].dot(&b.coords)),
    ];

    let mut best_sep = T::constant(-f64::MAX);
    let mut best_axis = axes[0];

    for (i, axis) in axes.iter().enumerate() {
        let axis_norm_squared = axis.norm_squared();

        if axis_norm_squared > T::constant(f64::EPSILON) {
            let axis_norm = na::ComplexField::sqrt(axis_norm_squared);

            // NOTE: for both axis and -axis, the dot1 will have the same
            // value because of the cuboid's symmetry.
            let local_pt1 = cube1.local_support_point(&axis);
            let dot1 = local_pt1.coords.dot(&axis) / axis_norm;

            let (dot2_min, dot2_max) = crate::utils::sort2(tri_dots[i].0, tri_dots[i].1);

            let separation_a = dot2_min / axis_norm - dot1; // separation on axis
            let separation_b = -dot2_max / axis_norm - dot1; // separation on -axis

            if separation_a > best_sep {
                best_sep = separation_a;
                best_axis = *axis / axis_norm;
            }

            if separation_b > best_sep {
                best_sep = separation_b;
                best_axis = -*axis / axis_norm;
            }
        }
    }

    (best_sep, best_axis)
}

/// Finds the best separating normal between a triangle and a convex shape implementing the `SupportMap` trait.
///
/// Only the normals of `triangle1` are tested.
#[cfg(feature = "dim2")]
pub fn triangle_support_map_find_local_separating_normal_oneway<T: AD>(
    triangle1: &Triangle<T>,
    shape2: &impl SupportMap,
    pos12: &Isometry<T>,
) -> (T, Vector<T>) {
    let mut best_sep = T::constant(-f64::MAX);
    let mut best_normal = Vector::zeros();

    for edge in &triangle1.edges() {
        if let Some(normal) = edge.normal() {
            let sep = support_map_support_map_compute_separation(triangle1, shape2, pos12, &normal);

            if sep > best_sep {
                best_sep = sep;
                best_normal = *normal;
            }
        }
    }

    (best_sep, best_normal)
}

/// Finds the best separating normal between a triangle and a cuboid.
///
/// Only the normals of `triangle1` are tested.
#[cfg(feature = "dim2")]
pub fn triangle_cuboid_find_local_separating_normal_oneway<T: AD>(
    triangle1: &Triangle<T>,
    shape2: &Cuboid,
    pos12: &Isometry<T>,
) -> (T, Vector<T>) {
    triangle_support_map_find_local_separating_normal_oneway(triangle1, shape2, pos12)
}

/// Finds the best separating normal a triangle and a cuboid.
///
/// Only the normals of `triangle1` are tested.
#[cfg(feature = "dim3")]
#[inline(always)]
pub fn triangle_cuboid_find_local_separating_normal_oneway<T: AD>(
    triangle1: &Triangle<T>,
    shape2: &Cuboid<T>,
    pos12: &Isometry<T>,
) -> (T, Vector<T>) {
    sat::point_cuboid_find_local_separating_normal_oneway(
        triangle1.a,
        triangle1.normal(),
        shape2,
        pos12,
    )
}
