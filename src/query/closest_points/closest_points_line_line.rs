use crate::math::{Point, Vector};
use crate::na::{Point as SPoint, SVector};
use ad_trait::AD;

/// Closest points between two lines.
///
/// The result, say `res`, is such that the closest points between both lines are
/// `orig1 + dir1 * res.0` and `orig2 + dir2 * res.1`.
#[inline]
pub fn closest_points_line_line_parameters<T: AD>(
    orig1: &Point<T>,
    dir1: &Vector<T>,
    orig2: &Point<T>,
    dir2: &Vector<T>,
) -> (T, T) {
    let res = closest_points_line_line_parameters_eps(
        orig1,
        dir1,
        orig2,
        dir2,
        crate::math::DEFAULT_EPSILON,
    );
    (res.0, res.1)
}

/// Closest points between two lines with a custom tolerance epsilon.
///
/// The result, say `res`, is such that the closest points between both lines are
/// `orig1 + dir1 * res.0` and `orig2 + dir2 * res.1`. If the lines are parallel
/// then `res.2` is set to `true` and the returned closest points are `orig1` and
/// its projection on the second line.
#[inline]
pub fn closest_points_line_line_parameters_eps<T: AD, const D: usize>(
    orig1: &SPoint<T, D>,
    dir1: &SVector<T, D>,
    orig2: &SPoint<T, D>,
    dir2: &SVector<T, D>,
    eps: T,
) -> (T, T, bool) {
    // Inspired by RealField-time collision detection by Christer Ericson.
    let r = orig1 - orig2;

    let a = dir1.norm_squared();
    let e = dir2.norm_squared();
    let f = dir2.dot(&r);

    if a <= eps && e <= eps {
        (T::zero(), T::zero(), false)
    } else if a <= eps {
        (T::zero(), f / e, false)
    } else {
        let c = dir1.dot(&r);
        if e <= eps {
            (-c / a, T::zero(), false)
        } else {
            let b = dir1.dot(dir2);
            let ae = a * e;
            let bb = b * b;
            let denom = ae - bb;

            // Use absolute and ulps error to test collinearity.
            let parallel = denom <= eps || ulps_eq!(ae, bb);

            let s = if !parallel {
                (b * f - c * e) / denom
            } else {
                T::zero()
            };

            (s, (b * s + f) / e, parallel)
        }
    }
}

// FIXME: can we re-used this for the segment/segment case?
/// Closest points between two segments.
#[inline]
pub fn closest_points_line_line<T: AD>(
    orig1: &Point<T>,
    dir1: &Vector<T>,
    orig2: &Point<T>,
    dir2: &Vector<T>,
) -> (Point<T>, Point<T>) {
    let (s, t) = closest_points_line_line_parameters(orig1, dir1, orig2, dir2);
    (*orig1 + *dir1 * s, *orig2 + *dir2 * t)
}
