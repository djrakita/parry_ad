use crate::math::{Isometry};
use crate::partitioning::{GenericQbvh, IndexedData, Qbvh, QbvhStorage};
use crate::shape::Shape;
use crate::utils::DefaultStorage;
use crate::shape::AD;

/// Trait implemented by shapes composed of multiple simpler shapes.
///
/// A composite shape is composed of several shapes. For example, this can
/// be a convex decomposition of a concave shape; or a triangle-mesh.
#[cfg(feature = "std")]
pub trait SimdCompositeShape<T: AD> {
    /// Applies a function to one sub-shape of this composite shape.
    fn map_part_at(&self, shape_id: u32, f: &mut dyn FnMut(Option<&Isometry<T>>, &dyn Shape<T>));

    /// Gets the acceleration structure of the composite shape.
    fn qbvh(&self) -> &Qbvh<u32, T>;
}

pub trait TypedSimdCompositeShape<T: AD> {
    type PartShape: ?Sized + Shape<T>;
    type PartId: IndexedData;
    type QbvhStorage: QbvhStorage<Self::PartId, T>;

    fn map_typed_part_at(
        &self,
        shape_id: Self::PartId,
        f: impl FnMut(Option<&Isometry<T>>, &Self::PartShape),
    );

    // TODO: we need this method because the compiler won't want
    // to cast `&Self::PartShape` to `&dyn Shape` because it complains
    // that `PairtShape` is not `Sized`.
    fn map_untyped_part_at(
        &self,
        shape_id: Self::PartId,
        f: impl FnMut(Option<&Isometry<T>>, &dyn Shape<T>),
    );

    fn typed_qbvh(&self) -> &GenericQbvh<Self::PartId, T, Self::QbvhStorage>;
}

#[cfg(feature = "std")]
impl<'a, T: AD> TypedSimdCompositeShape<T> for dyn SimdCompositeShape<T> + 'a {
    type PartShape = dyn Shape<T>;
    type PartId = u32;
    type QbvhStorage = DefaultStorage;

    fn map_typed_part_at(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Isometry<T>>, &Self::PartShape),
    ) {
        self.map_part_at(shape_id, &mut f)
    }

    fn map_untyped_part_at(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Isometry<T>>, &dyn Shape<T>),
    ) {
        self.map_part_at(shape_id, &mut f)
    }

    fn typed_qbvh(&self) -> &GenericQbvh<Self::PartId, T, Self::QbvhStorage> {
        self.qbvh()
    }
}
