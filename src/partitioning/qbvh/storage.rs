use crate::partitioning::{QbvhNode, QbvhProxy};
use crate::utils::{Array1, DefaultStorage};

#[cfg(all(feature = "std", feature = "cuda"))]
use crate::utils::CudaArray1;
#[cfg(feature = "cuda")]
use crate::utils::{CudaArrayPointer1, CudaStorage, CudaStoragePtr};

use ad_trait::AD;

/// Trait describing all the types needed for storing a Qbvh’s data.
pub trait QbvhStorage<LeafData, T: AD> {
    /// Type of the array containing the Qbvh nodes.
    type Nodes: Array1<QbvhNode<T>>;
    /// Type of an array containing u32.
    type ArrayU32: Array1<u32>;
    /// Type of the array containing the Qbvh leaves.
    type ArrayProxies: Array1<QbvhProxy<LeafData>>;
}

#[cfg(feature = "std")]
impl<LeafData, T: AD> QbvhStorage<LeafData, T> for DefaultStorage {
    type Nodes = Vec<QbvhNode<T>>;
    type ArrayU32 = Vec<u32>;
    type ArrayProxies = Vec<QbvhProxy<LeafData>>;
}

#[cfg(all(feature = "std", feature = "cuda"))]
impl<LeafData: cust_core::DeviceCopy, T: AD> QbvhStorage<LeafData, T> for CudaStorage {
    type Nodes = CudaArray1<QbvhNode<T>>;
    type ArrayU32 = CudaArray1<u32>;
    type ArrayProxies = CudaArray1<QbvhProxy<LeafData>>;
}

#[cfg(feature = "cuda")]
impl<LeafData: cust_core::DeviceCopy, T: AD> QbvhStorage<LeafData, T> for CudaStoragePtr {
    type Nodes = CudaArrayPointer1<QbvhNode<T>>;
    type ArrayU32 = CudaArrayPointer1<u32>;
    type ArrayProxies = CudaArrayPointer1<QbvhProxy<LeafData>>;
}
