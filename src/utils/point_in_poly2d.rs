use na::Point2;
use ad_trait::AD;

/// Tests if the given point is inside of a polygon with arbitrary orientation.
pub fn point_in_poly2d<T: AD>(pt: &Point2<T>, poly: &[Point2<T>]) -> bool {
    if poly.len() == 0 {
        false
    } else {
        let mut sign = T::zero();

        for i1 in 0..poly.len() {
            let i2 = (i1 + 1) % poly.len();
            let seg_dir = poly[i2] - poly[i1];
            let dpt = pt - poly[i1];
            let perp = dpt.perp(&seg_dir);

            if sign.is_zero() {
                sign = perp;
            } else if sign * perp < T::zero() {
                return false;
            }
        }

        return true;
    }
}
