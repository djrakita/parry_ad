use crate::shape::{Capsule, Cylinder};
use crate::transformation::utils;
use na::{self, Point3};
use ad_trait::AD;

impl<T: AD> Capsule<T> {
    /// Outlines this capsule’s shape using polylines.
    pub fn to_outline(&self, nsubdiv: u32) -> (Vec<Point3<T>>, Vec<[u32; 2]>) {
        let (vtx, idx) = canonical_capsule_outline(self.radius, self.half_height(), nsubdiv);
        (utils::transformed(vtx, self.canonical_transform()), idx)
    }
}

/// Generates a capsule.
pub(crate) fn canonical_capsule_outline<T: AD>(
    caps_radius: T,
    cylinder_half_height: T,
    nsubdiv: u32,
) -> (Vec<Point3<T>>, Vec<[u32; 2]>) {
    let (mut vtx, mut idx) = Cylinder::new(cylinder_half_height, caps_radius).to_outline(nsubdiv);
    let shift = vtx.len() as u32;

    // Generate the hemispheres.
    super::ball_to_outline::push_unit_hemisphere_outline(nsubdiv / 2, &mut vtx, &mut idx);
    super::ball_to_outline::push_unit_hemisphere_outline(nsubdiv / 2, &mut vtx, &mut idx);

    let ncap_pts = (nsubdiv / 2 + 1) * 2;
    vtx[shift as usize..(shift + ncap_pts) as usize]
        .iter_mut()
        .for_each(|pt| {
            *pt *= caps_radius * T::constant(2.0);
            pt.y += cylinder_half_height
        });

    vtx[(shift + ncap_pts) as usize..]
        .iter_mut()
        .for_each(|pt| {
            *pt *= caps_radius * T::constant(2.0);
            pt.y = -pt.y - cylinder_half_height
        });

    (vtx, idx)
}
