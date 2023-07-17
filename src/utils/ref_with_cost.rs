//! A reference packed with a cost value.

use std::cmp::Ordering;
use ad_trait::AD;

/// A reference packed with a cost value.
pub struct RefWithCost<'a, T: 'a, A: AD> {
    /// The reference to an object.
    pub object: &'a T,
    /// The cost of the object.
    pub cost: A,
}

impl<'a, A: PartialEq + AD, T: AD> PartialEq for RefWithCost<'a, A, T> {
    #[inline]
    fn eq(&self, other: &RefWithCost<'a, A, T>) -> bool {
        self.cost.eq(&other.cost)
    }
}

impl<'a, A: PartialEq + AD, T: AD> Eq for RefWithCost<'a, A, T> {}

impl<'a, A: PartialOrd + AD, T: AD> PartialOrd for RefWithCost<'a, A, T> {
    #[inline]
    fn partial_cmp(&self, other: &RefWithCost<'a, A, T>) -> Option<Ordering> {
        self.cost.partial_cmp(&other.cost)
    }
}

impl<'a, A: PartialOrd + AD, T: AD> Ord for RefWithCost<'a, A, T> {
    #[inline]
    fn cmp(&self, other: &RefWithCost<'a, A, T>) -> Ordering {
        if self.cost < other.cost {
            Ordering::Less
        } else if self.cost > other.cost {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}
