use std::iter::IntoIterator;

use crate::bounding_volume::Aabb;
use crate::math::{Isometry, Point, Vector, DIM};
use crate::shape::SupportMap;
use na;
use ad_trait::AD;

/// Computes the Aabb of an support mapped shape.
#[cfg(feature = "dim3")]
pub fn support_map_aabb<G, T: AD>(m: &Isometry<T>, i: &G) -> Aabb<T>
where
    G: SupportMap<T>,
{
    let mut min = na::zero::<Vector<T>>();
    let mut max = na::zero::<Vector<T>>();
    let mut basis = na::zero::<Vector<T>>();

    for d in 0..DIM {
        // FIXME: this could be further improved iterating on `m`'s columns, and passing
        // Id as the transformation matrix.
        basis[d] = 1.0;
        max[d] = i.support_point(m, &basis)[d];

        basis[d] = -1.0;
        min[d] = i.support_point(m, &basis)[d];

        basis[d] = 0.0;
    }

    Aabb::new(Point::from(min), Point::from(max))
}

/// Computes the Aabb of an support mapped shape.
pub fn local_support_map_aabb<G, T: AD>(i: &G) -> Aabb<T>
where
    G: SupportMap<T>,
{
    let mut min = na::zero::<Vector<T>>();
    let mut max = na::zero::<Vector<T>>();
    let mut basis = na::zero::<Vector<T>>();

    for d in 0..DIM {
        // FIXME: this could be further improved iterating on `m`'s columns, and passing
        // Id as the transformation matrix.
        basis[d] = 1.0;
        max[d] = i.local_support_point(&basis)[d];

        basis[d] = -1.0;
        min[d] = i.local_support_point(&basis)[d];

        basis[d] = 0.0;
    }

    Aabb::new(Point::from(min), Point::from(max))
}

/// Computes the Aabb of a set of points transformed by `m`.
pub fn point_cloud_aabb<'a, I, T: AD>(m: &Isometry<T>, pts: I) -> Aabb<T>
where
    I: IntoIterator<Item = &'a Point<T>>,
{
    let mut it = pts.into_iter();

    let p0 = it.next().expect(
        "Point cloud Aabb construction: the input iterator should yield at least one point.",
    );
    let wp0 = m.transform_point(&p0);
    let mut min: Point<T> = wp0;
    let mut max: Point<T> = wp0;

    for pt in it {
        let wpt = m * pt;
        min = min.inf(&wpt);
        max = max.sup(&wpt);
    }

    Aabb::new(min, max)
}

/// Computes the Aabb of a set of points.
pub fn local_point_cloud_aabb<'a, I, T: AD>(pts: I) -> Aabb<T>
where
    I: IntoIterator<Item = &'a Point<T>>,
{
    let mut it = pts.into_iter();

    let p0 = it.next().expect(
        "Point cloud Aabb construction: the input iterator should yield at least one point.",
    );
    let mut min: Point<T> = *p0;
    let mut max: Point<T> = *p0;

    for pt in it {
        min = min.inf(&pt);
        max = max.sup(&pt);
    }

    Aabb::new(min, max)
}
