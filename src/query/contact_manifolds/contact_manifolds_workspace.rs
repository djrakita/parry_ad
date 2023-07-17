use downcast_rs::{impl_downcast, DowncastSync};

use crate::query::contact_manifolds::{
    CompositeShapeCompositeShapeContactManifoldsWorkspace,
    CompositeShapeShapeContactManifoldsWorkspace,
    HeightFieldCompositeShapeContactManifoldsWorkspace, HeightFieldShapeContactManifoldsWorkspace,
    TriMeshShapeContactManifoldsWorkspace,
};

use ad_trait::AD;

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize))]
/// Enum representing workspace data of a specific type.
pub enum TypedWorkspaceData<'a, T: AD> {
    /// A trimesh workspace.
    TriMeshShapeContactManifoldsWorkspace(&'a TriMeshShapeContactManifoldsWorkspace<T>),
    /// A heightfield vs. shape workspace.
    HeightfieldShapeContactManifoldsWorkspace(&'a HeightFieldShapeContactManifoldsWorkspace),
    /// A heightfield vs. composite shape workspace.
    HeightfieldCompositeShapeContactManifoldsWorkspace(
        &'a HeightFieldCompositeShapeContactManifoldsWorkspace,
    ),
    /// A composite shape vs. composite shape workspace.
    CompositeShapeCompositeShapeContactManifoldsWorkspace(
        &'a CompositeShapeCompositeShapeContactManifoldsWorkspace,
    ),
    /// A composite shape vs. shape workspace.
    CompositeShapeShapeContactManifoldsWorkspace(&'a CompositeShapeShapeContactManifoldsWorkspace),
    /// A custom workspace.
    Custom(u32),
}

// NOTE: must match the TypedWorkspaceData enum.
#[cfg(feature = "serde-serialize")]
#[derive(Deserialize)]
enum DeserializableWorkspaceData<T: AD> {
    TriMeshShapeContactManifoldsWorkspace(TriMeshShapeContactManifoldsWorkspace<T>),
    HeightfieldShapeContactManifoldsWorkspace(HeightFieldShapeContactManifoldsWorkspace),
    HeightfieldCompositeShapeContactManifoldsWorkspace(
        HeightFieldCompositeShapeContactManifoldsWorkspace,
    ),
    CompositeShapeCompositeShapeContactManifoldsWorkspace(
        CompositeShapeCompositeShapeContactManifoldsWorkspace,
    ),
    CompositeShapeShapeContactManifoldsWorkspace(CompositeShapeShapeContactManifoldsWorkspace),
    Custom(u32),
}

#[cfg(feature = "serde-serialize")]
impl<T: AD> DeserializableWorkspaceData<T> {
    pub fn into_contact_manifold_workspace(self) -> Option<ContactManifoldsWorkspace> {
        match self {
            DeserializableWorkspaceData::TriMeshShapeContactManifoldsWorkspace(w) => {
                Some(ContactManifoldsWorkspace(Box::new(w)))
            }
            DeserializableWorkspaceData::HeightfieldShapeContactManifoldsWorkspace(w) => {
                Some(ContactManifoldsWorkspace(Box::new(w)))
            }
            DeserializableWorkspaceData::HeightfieldCompositeShapeContactManifoldsWorkspace(w) => {
                Some(ContactManifoldsWorkspace(Box::new(w)))
            }
            DeserializableWorkspaceData::CompositeShapeCompositeShapeContactManifoldsWorkspace(
                w,
            ) => Some(ContactManifoldsWorkspace(Box::new(w))),
            DeserializableWorkspaceData::CompositeShapeShapeContactManifoldsWorkspace(w) => {
                Some(ContactManifoldsWorkspace(Box::new(w)))
            }
            DeserializableWorkspaceData::Custom(_) => None,
        }
    }
}

/// Data from a [`ContactManifoldsWorkspace`].
pub trait WorkspaceData<T: AD>: DowncastSync {
    /// Gets the underlying workspace as an enum.
    fn as_typed_workspace_data(&self) -> TypedWorkspaceData<T>;

    /// Clones `self`.
    fn clone_dyn(&self) -> Box<dyn WorkspaceData<T>>;
}

// impl_downcast!(sync WorkspaceData);
impl_downcast!(sync WorkspaceData<T> where T: AD);

// Note we have this newtype because it simplifies the serialization/deserialization code.
/// A serializable workspace used by some contact-manifolds computation algorithms.
pub struct ContactManifoldsWorkspace<T: AD>(pub Box<dyn WorkspaceData<T>>);

impl<T: AD> Clone for ContactManifoldsWorkspace<T> {
    fn clone(&self) -> Self {
        ContactManifoldsWorkspace(self.0.clone_dyn())
    }
}

/*
impl<T: WorkspaceData<A>, A: AD> From<T> for ContactManifoldsWorkspace<A> {
    fn from(data: T) -> Self {
        Self(Box::new(data) as Box<dyn WorkspaceData<A>>)
    }
}
*/

#[cfg(feature = "serde-serialize")]
impl<T: AD> serde::Serialize for ContactManifoldsWorkspace<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.0.as_typed_workspace_data().serialize(serializer)
    }
}

#[cfg(feature = "serde-serialize")]
impl<'de, T: AD> serde::Deserialize<'de> for ContactManifoldsWorkspace<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use crate::serde::de::Error;
        DeserializableWorkspaceData::deserialize(deserializer)?
            .into_contact_manifold_workspace()
            .ok_or(D::Error::custom("Cannot deserialize custom shape."))
    }
}
