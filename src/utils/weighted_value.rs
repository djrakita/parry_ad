use std::cmp::{Eq, Ord, Ordering, PartialEq, PartialOrd};
use ad_trait::AD;

#[derive(Copy, Clone)]
pub struct WeightedValue<T, A: AD> {
    pub value: T,
    pub cost: A,
}

impl<T, A: AD> WeightedValue<T, A> {
    /// Creates a new reference packed with a cost value.
    #[inline]
    pub fn new(value: T, cost: A) -> WeightedValue<T, A> {
        WeightedValue { value, cost }
    }
}

impl<T, A: AD> PartialEq for WeightedValue<T, A> {
    #[inline]
    fn eq(&self, other: &WeightedValue<T, A>) -> bool {
        self.cost.eq(&other.cost)
    }
}

impl<T, A: AD> Eq for WeightedValue<T, A> {}

impl<T, A: AD> PartialOrd for WeightedValue<T, A> {
    #[inline]
    fn partial_cmp(&self, other: &WeightedValue<T, A>) -> Option<Ordering> {
        self.cost.partial_cmp(&other.cost)
    }
}

impl<T, A: AD> Ord for WeightedValue<T, A> {
    #[inline]
    fn cmp(&self, other: &WeightedValue<T, A>) -> Ordering {
        if self.cost < other.cost {
            Ordering::Less
        } else if self.cost > other.cost {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}
