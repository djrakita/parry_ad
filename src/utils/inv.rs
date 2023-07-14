use ad_trait::AD;

pub fn inv<T: AD>(val: T) -> T {
    if val == T::constant(0.0) {
        T::constant(0.0)
    } else {
        T::constant(1.0) / val
    }
}
