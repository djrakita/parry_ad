use ad_trait::AD;
use crate::num::Bounded;
use na;
#[cfg(feature = "dim3")]
use {crate::bounding_volume, crate::math::Point};

/// Returns the index of the support point of a list of points.
pub fn support_point_id<const D: usize, T: AD>(
    direction: &na::SVector<T, D>,
    points: &[na::Point<T, D>],
) -> Option<usize> {
    let mut argmax = None;
    let _max = T::constant(Bounded::max_value());
    let mut max = -_max;

    for (id, pt) in points.iter().enumerate() {
        let dot = direction.dot(&pt.coords);

        if dot > max {
            argmax = Some(id);
            max = dot;
        }
    }

    argmax
}

/// Returns the index of the support point of an indexed list of points.
pub fn indexed_support_point_id<I, const D: usize, T: AD>(
    direction: &na::SVector<T, D>,
    points: &[na::Point<T, D>],
    idx: I,
) -> Option<usize>
where
    I: Iterator<Item = usize>,
{
    let mut argmax = None;
    let mut max = T::constant(-f64::MAX);

    for i in idx.into_iter() {
        let dot = direction.dot(&points[i].coords);

        if dot > max {
            argmax = Some(i);
            max = dot;
        }
    }

    argmax
}

/// Returns the number `n` such that `points[idx.nth(n)]` is the support point.
#[cfg(feature = "dim3")] // We only use this in 3D right now.
pub fn indexed_support_point_nth<I, const D: usize, T: AD>(
    direction: &na::SVector<T, D>,
    points: &[na::Point<T, D>],
    idx: I,
) -> Option<usize>
where
    I: Iterator<Item = usize>,
{
    let mut argmax = None;
    let mut max = T::constant(-f64::MAX);

    for (k, i) in idx.into_iter().enumerate() {
        let dot = direction.dot(&points[i].coords);

        if dot > max {
            argmax = Some(k);
            max = dot;
        }
    }

    argmax
}

/// Scale and center the given set of point depending on their Aabb.
#[cfg(feature = "dim3")]
pub fn normalize<T: AD>(coords: &mut [Point<T>]) -> (Point<T>, T) {
    let aabb = bounding_volume::details::local_point_cloud_aabb(&coords[..]);
    let diag = na::distance(&aabb.mins, &aabb.maxs);
    let center = aabb.center();

    for c in coords.iter_mut() {
        *c = (*c + (-center.coords)) / diag;
    }

    (center, diag)
}
