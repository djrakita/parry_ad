use ad_trait::AD;
use crate::bounding_volume::BoundingVolume;
use crate::math::{Isometry};
use crate::query::visitors::BoundingVolumeIntersectionsVisitor;
use crate::query::{Contact, QueryDispatcher};
use crate::shape::{Shape, SimdCompositeShape};
use crate::utils::IsometryOpt;

/// Best contact between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn contact_composite_shape_shape<D: ?Sized, G1: ?Sized, T: AD>(
    dispatcher: &D,
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &dyn Shape<T>,
    prediction: T,
) -> Option<Contact<T>>
where
    D: QueryDispatcher<T>,
    G1: SimdCompositeShape<T>,
{
    // Find new collisions
    let ls_aabb2 = g2.compute_aabb(pos12).loosened(prediction);
    let mut res = None::<Contact<T>>;

    let mut leaf_callback = |i: &_| {
        g1.map_part_at(*i, &mut |part_pos1, part1| {
            if let Ok(Some(mut c)) =
                dispatcher.contact(&part_pos1.inv_mul(pos12), part1, g2, prediction)
            {
                let replace = res.map_or(true, |cbest| c.dist < cbest.dist);

                if replace {
                    if let Some(part_pos1) = part_pos1 {
                        c.transform1_by_mut(part_pos1);
                    }
                    res = Some(c)
                }
            }
        });

        true
    };

    let mut visitor = BoundingVolumeIntersectionsVisitor::new(&ls_aabb2, &mut leaf_callback);
    let _ = g1.qbvh().traverse_depth_first(&mut visitor);
    res
}

/// Best contact between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn contact_shape_composite_shape<D: ?Sized, G2: ?Sized, T: AD>(
    dispatcher: &D,
    pos12: &Isometry<T>,
    g1: &dyn Shape<T>,
    g2: &G2,
    prediction: T,
) -> Option<Contact<T>>
where
    D: QueryDispatcher<T>,
    G2: SimdCompositeShape<T>,
{
    contact_composite_shape_shape(dispatcher, &pos12.inverse(), g2, g1, prediction)
        .map(|c| c.flipped())
}
