//! Spatial partitioning tools.

#[cfg(feature = "std")]
pub use self::qbvh::{
    CenterDataSplitter, QbvhDataGenerator, QbvhNonOverlappingDataSplitter, QbvhUpdateWorkspace,
};
pub use self::qbvh::{
    GenericQbvh, IndexedData, NodeIndex, Qbvh, QbvhNode, QbvhProxy, QbvhStorage, SimdNodeIndex,
};
#[cfg(feature = "parallel")]
pub use self::visitor::{ParallelSimdSimultaneousVisitor, ParallelSimdVisitor};
pub use self::visitor::{
    SimdBestFirstVisitStatus, SimdBestFirstVisitor, SimdSimultaneousVisitStatus,
    SimdSimultaneousVisitor, SimdVisitStatus, SimdVisitor,
};


/// A quaternary bounding-volume-hierarchy.
#[deprecated(note = "Renamed to Qbvh")]
pub type SimdQbvh<T, A> = Qbvh<T, A>;

mod qbvh;
mod visitor;
