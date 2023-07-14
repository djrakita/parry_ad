use crate::math::{Matrix, Point};
use ad_trait::AD;

/// Computes the covariance matrix of a set of points.
pub fn cov<T: AD>(pts: &[Point<T>]) -> Matrix<T> {
    center_cov(pts).1
}

/// Computes the center and the covariance matrix of a set of points.
pub fn center_cov<T: AD>(pts: &[Point<T>]) -> (Point<T>, Matrix<T>) {
    let center = crate::utils::center(pts);
    let mut cov: Matrix<T> = na::zero();
    let normalizer = T::constant(1.0 / (pts.len() as f64));

    for p in pts.iter() {
        let cp = *p - center;
        // NOTE: this is more numerically stable than using cov.syger.
        cov += cp * (cp * normalizer).transpose();
    }

    (center, cov)
}
