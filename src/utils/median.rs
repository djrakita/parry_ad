
use ad_trait::AD;

/// Computes the median of a set of values.
#[inline]
pub fn median<T: AD>(vals: &mut [T]) -> T {
    assert!(vals.len() > 0, "Cannot compute the median of zero values.");

    vals.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = vals.len();

    if n % 2 == 0 {
        (vals[n / 2 - 1] + vals[n / 2]) / T::constant(2.0)
    } else {
        vals[n / 2]
    }
}
