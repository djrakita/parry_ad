use crate::math::{Isometry, Point, Rotation, Translation, Vector, DIM};
use crate::shape::Cuboid;
use ad_trait::AD;

/// Computes an oriented bounding box for the given set of points.
///
/// The returned OBB is not guaranteed to be the smallest enclosing OBB.
/// Though it should be a pretty good on for most purposes.
pub fn obb<T: AD>(pts: &[Point<T>]) -> (Isometry<T>, Cuboid<T>) {
    let cov = crate::utils::cov(pts);
    let mut eigv = cov.symmetric_eigen().eigenvectors;

    if eigv.determinant() < T::zero() {
        eigv = -eigv;
    }

    let mut mins = Vector::repeat(T::constant(f64::MAX));
    let mut maxs = Vector::repeat(T::constant(-f64::MAX));

    for pt in pts {
        for i in 0..DIM {
            let dot = eigv.column(i).dot(&pt.coords);
            mins[i] = mins[i].min(dot);
            maxs[i] = maxs[i].max(dot);
        }
    }

    #[cfg(feature = "dim2")]
    let rot = Rotation::from_rotation_matrix(&na::Rotation2::from_matrix_unchecked(eigv));
    #[cfg(feature = "dim3")]
    let rot = Rotation::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(eigv));

    (
        rot * Translation::from((maxs + mins) / T::constant(2.0)),
        Cuboid::new((maxs - mins) / T::constant(2.0)),
    )
}
