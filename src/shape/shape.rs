use crate::bounding_volume::{Aabb, BoundingSphere, BoundingVolume};
use crate::mass_properties::MassProperties;
use crate::math::{Isometry, Point, Vector};
use crate::query::{PointQuery, RayCast};
#[cfg(feature = "serde-serialize")]
use crate::shape::SharedShape;
#[cfg(feature = "std")]
use crate::shape::{composite_shape::SimdCompositeShape, Compound, HeightField, Polyline, TriMesh};
use crate::shape::{
    Ball, Capsule, Cuboid, FeatureId, HalfSpace, PolygonalFeatureMap, RoundCuboid, RoundShape,
    RoundTriangle, Segment, SupportMap, Triangle,
};
#[cfg(feature = "dim3")]
use crate::shape::{Cone, Cylinder, RoundCone, RoundCylinder};

#[cfg(feature = "dim3")]
#[cfg(feature = "std")]
use crate::shape::{ConvexPolyhedron, RoundConvexPolyhedron};

#[cfg(feature = "dim2")]
#[cfg(feature = "std")]
use crate::shape::{ConvexPolygon, RoundConvexPolygon};
use downcast_rs::{impl_downcast, DowncastSync};
use na::{RealField, Unit};
use num::Zero;
use num_derive::FromPrimitive;

use ad_trait::AD;

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, Eq, Hash)]
/// Enum representing the type of a shape.
pub enum ShapeType {
    /// A ball shape.
    Ball = 0,
    /// A cuboid shape.
    Cuboid,
    /// A capsule shape.
    Capsule,
    /// A segment shape.
    Segment,
    /// A triangle shape.
    Triangle,
    /// A triangle mesh shape.
    TriMesh,
    /// A set of segments.
    Polyline,
    /// A shape representing a full half-space.
    HalfSpace,
    /// A heightfield shape.
    HeightField,
    /// A Compound shape.
    Compound,
    #[cfg(feature = "dim2")]
    ConvexPolygon,
    #[cfg(feature = "dim3")]
    /// A convex polyhedron.
    ConvexPolyhedron,
    #[cfg(feature = "dim3")]
    /// A cylindrical shape.
    Cylinder,
    #[cfg(feature = "dim3")]
    /// A cone shape.
    Cone,
    // /// A custom shape type.
    // Custom(u8),
    /// A cuboid with rounded corners.
    RoundCuboid,
    /// A triangle with rounded corners.
    RoundTriangle,
    // /// A triangle-mesh with rounded corners.
    // RoundedTriMesh,
    // /// An heightfield with rounded corners.
    // RoundedHeightField,
    /// A cylinder with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCylinder,
    /// A cone with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCone,
    /// A convex polyhedron with rounded corners.
    #[cfg(feature = "dim3")]
    RoundConvexPolyhedron,
    /// A convex polygon with rounded corners.
    #[cfg(feature = "dim2")]
    RoundConvexPolygon,
    /// A custom user-defined shape.
    Custom,
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize))]
/// Enum representing the shape with its actual type
pub enum TypedShape<'a, T: AD> {
    /// A ball shape.
    Ball(&'a Ball<T>),
    /// A cuboid shape.
    Cuboid(&'a Cuboid<T>),
    /// A capsule shape.
    Capsule(&'a Capsule<T>),
    /// A segment shape.
    Segment(&'a Segment<T>),
    /// A triangle shape.
    Triangle(&'a Triangle<T>),
    /// A triangle mesh shape.
    #[cfg(feature = "std")]
    TriMesh(&'a TriMesh<T>),
    /// A set of segments.
    #[cfg(feature = "std")]
    Polyline(&'a Polyline<T>),
    /// A shape representing a full half-space.
    HalfSpace(&'a HalfSpace<T>),
    /// A heightfield shape.
    #[cfg(feature = "std")]
    HeightField(&'a HeightField<T>),
    /// A Compound shape.
    #[cfg(feature = "std")]
    Compound(&'a Compound<T>),
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    ConvexPolygon(&'a ConvexPolygon),
    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    /// A convex polyhedron.
    ConvexPolyhedron(&'a ConvexPolyhedron<T>),
    #[cfg(feature = "dim3")]
    /// A cylindrical shape.
    Cylinder(&'a Cylinder<T>),
    #[cfg(feature = "dim3")]
    /// A cone shape.
    Cone(&'a Cone<T>),
    // /// A custom shape type.
    // Custom(u8),
    /// A cuboid with rounded corners.
    RoundCuboid(&'a RoundCuboid<T>),
    /// A triangle with rounded corners.
    RoundTriangle(&'a RoundTriangle<T>),
    // /// A triangle-mesh with rounded corners.
    // RoundedTriMesh,
    // /// An heightfield with rounded corners.
    // RoundedHeightField,
    /// A cylinder with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCylinder(&'a RoundCylinder<T>),
    /// A cone with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCone(&'a RoundCone<T>),
    /// A convex polyhedron with rounded corners.
    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    RoundConvexPolyhedron(&'a RoundConvexPolyhedron<T>),
    /// A convex polygon with rounded corners.
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    RoundConvexPolygon(&'a RoundConvexPolygon),
    /// A custom user-defined shape with a type identified by a number.
    Custom(u32),
}

#[cfg(feature = "serde-serialize")]
#[derive(Deserialize)]
// NOTE: tha this enum MUST match the `TypedShape` enum.
/// Enum representing the shape with its actual type
pub(crate) enum DeserializableTypedShape<T: AD> {
    /// A ball shape.
    Ball(Ball<T>),
    /// A cuboid shape.
    Cuboid(Cuboid<T>),
    /// A capsule shape.
    Capsule(Capsule<T>),
    /// A segment shape.
    Segment(Segment<T>),
    /// A triangle shape.
    Triangle(Triangle<T>),
    /// A triangle mesh shape.
    #[cfg(feature = "std")]
    TriMesh(TriMesh<T>),
    /// A set of segments.
    #[cfg(feature = "std")]
    Polyline(Polyline<T>),
    /// A shape representing a full half-space.
    HalfSpace(HalfSpace<T>),
    /// A heightfield shape.
    #[cfg(feature = "std")]
    HeightField(HeightField<T>),
    /// A Compound shape.
    #[cfg(feature = "std")]
    Compound(Compound<T>),
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    ConvexPolygon(ConvexPolygon),
    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    /// A convex polyhedron.
    ConvexPolyhedron(ConvexPolyhedron<T>),
    #[cfg(feature = "dim3")]
    /// A cylindrical shape.
    Cylinder(Cylinder<T>),
    #[cfg(feature = "dim3")]
    /// A cone shape.
    Cone(Cone<T>),
    // /// A custom shape type.
    // Custom(u8),
    /// A cuboid with rounded corners.
    RoundCuboid(RoundCuboid<T>),
    /// A triangle with rounded corners.
    RoundTriangle(RoundTriangle<T>),
    // /// A triangle-mesh with rounded corners.
    // RoundedTriMesh,
    // /// An heightfield with rounded corners.
    // RoundedHeightField,
    /// A cylinder with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCylinder(RoundCylinder<T>),
    /// A cone with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCone(RoundCone<T>),
    /// A convex polyhedron with rounded corners.
    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    RoundConvexPolyhedron(RoundConvexPolyhedron<T>),
    /// A convex polygon with rounded corners.
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    RoundConvexPolygon(RoundConvexPolygon),
    /// A custom user-defined shape identified by a number.
    Custom(u32),
}

#[cfg(feature = "serde-serialize")]
impl<T: AD> DeserializableTypedShape<T> {
    /// Converts `self` to a `SharedShape` if `self` isn't `Custom`.
    pub fn into_shared_shape(self) -> Option<SharedShape<T>> {
        match self {
            DeserializableTypedShape::Ball(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::Cuboid(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::Capsule(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::Segment(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::Triangle(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "std")]
            DeserializableTypedShape::TriMesh(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "std")]
            DeserializableTypedShape::Polyline(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::HalfSpace(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "std")]
            DeserializableTypedShape::HeightField(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "std")]
            DeserializableTypedShape::Compound(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim2")]
            #[cfg(feature = "std")]
            DeserializableTypedShape::ConvexPolygon(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim3")]
            #[cfg(feature = "std")]
            DeserializableTypedShape::ConvexPolyhedron(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim3")]
            DeserializableTypedShape::Cylinder(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim3")]
            DeserializableTypedShape::Cone(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::RoundCuboid(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::RoundTriangle(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim3")]
            DeserializableTypedShape::RoundCylinder(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim3")]
            DeserializableTypedShape::RoundCone(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim3")]
            #[cfg(feature = "std")]
            DeserializableTypedShape::RoundConvexPolyhedron(s) => Some(SharedShape::new(s)),
            #[cfg(feature = "dim2")]
            #[cfg(feature = "std")]
            DeserializableTypedShape::RoundConvexPolygon(s) => Some(SharedShape::new(s)),
            DeserializableTypedShape::Custom(_) => None,
        }
    }
}

/// Trait implemented by shapes usable by Rapier.
pub trait Shape<T: AD>: RayCast<T> + PointQuery<T> + DowncastSync {
    /// Computes the Aabb of this shape.
    fn compute_local_aabb(&self) -> Aabb<T>;
    /// Computes the bounding-sphere of this shape.
    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T>;

    /// Clones this shape into a boxed trait-object.
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>>;

    /// Computes the Aabb of this shape with the given position.
    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.compute_local_aabb().transform_by(position)
    }
    /// Computes the bounding-sphere of this shape with the given position.
    fn compute_bounding_sphere(&self, position: &Isometry<T>) -> BoundingSphere<T> {
        self.compute_local_bounding_sphere().transform_by(position)
    }

    /// Compute the mass-properties of this shape given its uniform density.
    fn mass_properties(&self, density: T) -> MassProperties<T>;

    /// Gets the type tag of this shape.
    fn shape_type(&self) -> ShapeType;

    /// Gets the underlying shape as an enum.
    fn as_typed_shape(&self) -> TypedShape<T>;

    fn ccd_thickness(&self) -> T;

    // TODO: document this.
    // This should probably be the largest sharp edge angle (in radians) in [0; PI].
    // Though this isn't a very good description considering this is PI / 2
    // for capsule (which doesn't have any sharp angle). I guess a better way
    // to phrase this is: "the smallest angle such that rotating the shape by
    // that angle may result in different contact points".
    fn ccd_angular_thickness(&self) -> T;

    /// Is this shape known to be convex?
    ///
    /// If this returns `true` then `self` is known to be convex.
    /// If this returns `false` then it is not known whether or
    /// not `self` is convex.
    fn is_convex(&self) -> bool {
        false
    }

    /// Convents this shape into its support mapping, if it has one.
    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        None
    }

    #[cfg(feature = "std")]
    fn as_composite_shape(&self) -> Option<&dyn SimdCompositeShape<T>> {
        None
    }

    /// Converts this shape to a polygonal feature-map, if it is one.
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        None
    }

    // fn as_rounded(&self) -> Option<&Rounded<Box<AnyShape>>> {
    //     None
    // }

    /// The shape's normal at the given point located on a specific feature.
    fn feature_normal_at_point(
        &self,
        _feature: FeatureId,
        _point: &Point<T>,
    ) -> Option<Unit<Vector<T>>> {
        None
    }

    /// Computes the swept Aabb of this shape, i.e., the space it would occupy by moving from
    /// the given start position to the given end position.
    fn compute_swept_aabb(&self, start_pos: &Isometry<T>, end_pos: &Isometry<T>) -> Aabb<T> {
        let aabb1 = self.compute_aabb(start_pos);
        let aabb2 = self.compute_aabb(end_pos);
        aabb1.merged(&aabb2)
    }
}

// impl_downcast!(sync Shape);
impl_downcast!(sync Shape<T> where T: AD);
// impl_downcast!(Base<T> assoc H where T: Clone, H: Copy);

impl<A: AD> dyn Shape<A> {
    /// Converts this abstract shape to the given shape, if it is one.
    pub fn as_shape<T: Shape<A>>(&self) -> Option<&T> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to the given mutable shape, if it is one.
    pub fn as_shape_mut<T: Shape<A>>(&mut self) -> Option<&mut T> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a ball, if it is one.
    pub fn as_ball(&self) -> Option<&Ball<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable ball, if it is one.
    pub fn as_ball_mut(&mut self) -> Option<&mut Ball<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a cuboid, if it is one.
    pub fn as_cuboid(&self) -> Option<&Cuboid<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable cuboid, if it is one.
    pub fn as_cuboid_mut(&mut self) -> Option<&mut Cuboid<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a halfspace, if it is one.
    pub fn as_halfspace(&self) -> Option<&HalfSpace<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a halfspace, if it is one.
    pub fn as_halfspace_mut(&mut self) -> Option<&mut HalfSpace<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a segment, if it is one.
    pub fn as_segment(&self) -> Option<&Segment<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable segment, if it is one.
    pub fn as_segment_mut(&mut self) -> Option<&mut Segment<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a capsule, if it is one.
    pub fn as_capsule(&self) -> Option<&Capsule<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable capsule, if it is one.
    pub fn as_capsule_mut(&mut self) -> Option<&mut Capsule<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a triangle, if it is one.
    pub fn as_triangle(&self) -> Option<&Triangle<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable triangle, if it is one.
    pub fn as_triangle_mut(&mut self) -> Option<&mut Triangle<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a compound shape, if it is one.
    #[cfg(feature = "std")]
    pub fn as_compound(&self) -> Option<&Compound<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable compound shape, if it is one.
    #[cfg(feature = "std")]
    pub fn as_compound_mut(&mut self) -> Option<&mut Compound<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a triangle mesh, if it is one.
    #[cfg(feature = "std")]
    pub fn as_trimesh(&self) -> Option<&TriMesh<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable triangle mesh, if it is one.
    #[cfg(feature = "std")]
    pub fn as_trimesh_mut(&mut self) -> Option<&mut TriMesh<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a polyline, if it is one.
    #[cfg(feature = "std")]
    pub fn as_polyline(&self) -> Option<&Polyline<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable polyline, if it is one.
    #[cfg(feature = "std")]
    pub fn as_polyline_mut(&mut self) -> Option<&mut Polyline<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a heightfield, if it is one.
    #[cfg(feature = "std")]
    pub fn as_heightfield(&self) -> Option<&HeightField<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable heightfield, if it is one.
    #[cfg(feature = "std")]
    pub fn as_heightfield_mut(&mut self) -> Option<&mut HeightField<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a round cuboid, if it is one.
    pub fn as_round_cuboid(&self) -> Option<&RoundCuboid<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable round cuboid, if it is one.
    pub fn as_round_cuboid_mut(&mut self) -> Option<&mut RoundCuboid<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a round triangle, if it is one.
    pub fn as_round_triangle(&self) -> Option<&RoundTriangle<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a round triangle, if it is one.
    pub fn as_round_triangle_mut(&mut self) -> Option<&mut RoundTriangle<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a convex polygon, if it is one.
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    pub fn as_convex_polygon(&self) -> Option<&ConvexPolygon> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable convex polygon, if it is one.
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    pub fn as_convex_polygon_mut(&mut self) -> Option<&mut ConvexPolygon> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a round convex polygon, if it is one.
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    pub fn as_round_convex_polygon(&self) -> Option<&RoundConvexPolygon> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable round convex polygon, if it is one.
    #[cfg(feature = "dim2")]
    #[cfg(feature = "std")]
    pub fn as_round_convex_polygon_mut(&mut self) -> Option<&mut RoundConvexPolygon> {
        self.downcast_mut()
    }

    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    pub fn as_convex_polyhedron(&self) -> Option<&ConvexPolyhedron<A>> {
        self.downcast_ref()
    }
    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    pub fn as_convex_polyhedron_mut(&mut self) -> Option<&mut ConvexPolyhedron<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder(&self) -> Option<&Cylinder<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder_mut(&mut self) -> Option<&mut Cylinder<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone(&self) -> Option<&Cone<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone_mut(&mut self) -> Option<&mut Cone<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a round cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_round_cylinder(&self) -> Option<&RoundCylinder<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable round cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_round_cylinder_mut(&mut self) -> Option<&mut RoundCylinder<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a round cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_round_cone(&self) -> Option<&RoundCone<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable round cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_round_cone_mut(&mut self) -> Option<&mut RoundCone<A>> {
        self.downcast_mut()
    }

    /// Converts this abstract shape to a round convex polyhedron, if it is one.
    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    pub fn as_round_convex_polyhedron(&self) -> Option<&RoundConvexPolyhedron<A>> {
        self.downcast_ref()
    }
    /// Converts this abstract shape to a mutable round convex polyhedron, if it is one.
    #[cfg(feature = "dim3")]
    #[cfg(feature = "std")]
    pub fn as_round_convex_polyhedron_mut(&mut self) -> Option<&mut RoundConvexPolyhedron<A>> {
        self.downcast_mut()
    }
}

impl<T: AD> Shape<T> for Ball<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_ball(density, self.radius)
    }

    fn ccd_thickness(&self) -> T {
        self.radius
    }

    fn ccd_angular_thickness(&self) -> T {
        T::constant(f64::pi())
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Ball
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Ball(self)
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    /// The shape's normal at the given point located on a specific feature.
    #[inline]
    fn feature_normal_at_point(
        &self,
        _: FeatureId,
        point: &Point<T>,
    ) -> Option<Unit<Vector<T>>> {
        Unit::try_new(point.coords, T::constant(crate::math::DEFAULT_EPSILON))
    }
}

impl<T: AD> Shape<T> for Cuboid<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_cuboid(density, self.half_extents)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Cuboid
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Cuboid(self)
    }

    fn ccd_thickness(&self) -> T {
        self.half_extents.min()
    }

    fn ccd_angular_thickness(&self) -> T {
        T::constant(f64::frac_pi_2())
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        Some((self as &dyn PolygonalFeatureMap<T>, T::zero()))
    }

    fn feature_normal_at_point(
        &self,
        feature: FeatureId,
        _point: &Point<T>,
    ) -> Option<Unit<Vector<T>>> {
        self.feature_normal(feature)
    }
}

impl<T: AD> Shape<T> for Capsule<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_capsule(density, self.segment.a, self.segment.b, self.radius)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Capsule
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Capsule(self)
    }

    fn ccd_thickness(&self) -> T {
        self.radius
    }

    fn ccd_angular_thickness(&self) -> T {
        T::constant(f64::frac_pi_2())
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        Some((&self.segment as &dyn PolygonalFeatureMap<T>, self.radius))
    }
}

impl<T: AD> Shape<T> for Triangle<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, _density: T) -> MassProperties<T> {
        #[cfg(feature = "dim2")]
        return MassProperties::from_triangle(_density, &self.a, &self.b, &self.c);
        #[cfg(feature = "dim3")]
        return MassProperties::zero();
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Triangle
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Triangle(self)
    }

    fn ccd_thickness(&self) -> T {
        // TODO: in 2D use the smallest height of the triangle.
        T::zero()
    }

    fn ccd_angular_thickness(&self) -> T {
        T::constant(f64::frac_pi_2())
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        Some((self as &dyn PolygonalFeatureMap<T>, T::zero()))
    }

    fn feature_normal_at_point(
        &self,
        feature: FeatureId,
        _point: &Point<T>,
    ) -> Option<Unit<Vector<T>>> {
        self.feature_normal(feature)
    }
}

impl<T: AD> Shape<T> for Segment<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, _density: T) -> MassProperties<T> {
        MassProperties::zero()
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn ccd_thickness(&self) -> T {
        T::zero()
    }

    fn ccd_angular_thickness(&self) -> T {
        T::constant(f64::frac_pi_2())
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Segment
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Segment(self)
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        Some((self as &dyn PolygonalFeatureMap<T>, T::zero()))
    }

    fn feature_normal_at_point(
        &self,
        feature: FeatureId,
        _point: &Point<T>,
    ) -> Option<Unit<Vector<T>>> {
        self.feature_normal(feature)
    }
}

#[cfg(feature = "std")]
impl<T: AD> Shape<T> for Compound<T> {
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        *self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.local_aabb().transform_by(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_compound(density, self.shapes())
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Compound
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Compound(self)
    }

    fn ccd_thickness(&self) -> T {
        self.shapes()
            .iter()
            .fold(T::constant(f64::MAX), |curr, (_, s)| curr.min(s.ccd_thickness()))
    }

    fn ccd_angular_thickness(&self) -> T {
        self.shapes().iter().fold(T::constant(f64::MAX), |curr, (_, s)| {
            curr.max(s.ccd_angular_thickness())
        })
    }

    #[cfg(feature = "std")]
    fn as_composite_shape(&self) -> Option<&dyn SimdCompositeShape<T>> {
        Some(self as &dyn SimdCompositeShape<T>)
    }
}

#[cfg(feature = "std")]
impl<T: AD> Shape<T> for Polyline<T> {
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        *self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, _density: T) -> MassProperties<T> {
        MassProperties::zero()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Polyline
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Polyline(self)
    }

    fn ccd_thickness(&self) -> T {
        T::zero()
    }

    fn ccd_angular_thickness(&self) -> T {
        // TODO: the value should depend on the angles between
        // adjacent segments of the polyline.
        T::constant(f64::frac_pi_4())
    }

    #[cfg(feature = "std")]
    fn as_composite_shape(&self) -> Option<&dyn SimdCompositeShape<T>> {
        Some(self as &dyn SimdCompositeShape<T>)
    }
}

#[cfg(feature = "std")]
impl<T: AD> Shape<T> for TriMesh<T> {
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        *self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_trimesh(density, self.vertices(), self.indices())
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::TriMesh
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::TriMesh(self)
    }

    fn ccd_thickness(&self) -> T {
        // TODO: in 2D, return the smallest CCD thickness among triangles?
        T::zero()
    }

    fn ccd_angular_thickness(&self) -> T {
        // TODO: the value should depend on the angles between
        // adjacent triangles of the trimesh.
        T::constant(f64::frac_pi_4())
    }

    #[cfg(feature = "std")]
    fn as_composite_shape(&self) -> Option<&dyn SimdCompositeShape<T>> {
        Some(self as &dyn SimdCompositeShape<T>)
    }
}

#[cfg(feature = "std")]
impl<T: AD> Shape<T> for HeightField<T> {
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, _density: T) -> MassProperties<T> {
        MassProperties::zero()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::HeightField
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::HeightField(self)
    }

    fn ccd_thickness(&self) -> T {
        T::zero()
    }

    fn ccd_angular_thickness(&self) -> T {
        // TODO: the value should depend on the angles between
        // adjacent triangles of the heightfield.
        T::constant(f64::frac_pi_4())
    }
}

#[cfg(feature = "dim2")]
#[cfg(feature = "std")]
impl<T: AD> Shape<T> for ConvexPolygon<T> {
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_convex_polygon(density, &self.points())
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::ConvexPolygon
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::ConvexPolygon(self)
    }

    fn ccd_thickness(&self) -> T {
        // TODO: we should use the OBB instead.
        self.compute_local_aabb().half_extents().min()
    }

    fn ccd_angular_thickness(&self) -> T {
        // TODO: the value should depend on the angles between
        // adjacent segments of the convex polygon.
        T::constant(f64::frac_pi_4())
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap> {
        Some(self as &dyn SupportMap)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, T)> {
        Some((self as &dyn PolygonalFeatureMap, T::zero()))
    }

    fn feature_normal_at_point(
        &self,
        feature: FeatureId,
        _point: &Point<T>,
    ) -> Option<Unit<Vector<T>>> {
        self.feature_normal(feature)
    }
}

#[cfg(feature = "dim3")]
#[cfg(feature = "std")]
impl<T: AD> Shape<T> for ConvexPolyhedron<T> {
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        let (vertices, indices) = self.to_trimesh();
        MassProperties::from_convex_polyhedron(density, &vertices, &indices)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::ConvexPolyhedron
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::ConvexPolyhedron(self)
    }

    fn ccd_thickness(&self) -> T {
        // TODO: we should use the OBB instead.
        self.compute_local_aabb().half_extents().min()
    }

    fn ccd_angular_thickness(&self) -> T {
        // TODO: the value should depend on the angles between
        // adjacent segments of the convex polyhedron.
        T::constant(f64::frac_pi_4())
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        Some((self as &dyn PolygonalFeatureMap<T>, T::zero()))
    }

    fn feature_normal_at_point(
        &self,
        feature: FeatureId,
        _point: &Point<T>,
    ) -> Option<Unit<Vector<T>>> {
        self.feature_normal(feature)
    }
}

#[cfg(feature = "dim3")]
impl<T: AD> Shape<T> for Cylinder<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_cylinder(density, self.half_height, self.radius)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Cylinder
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Cylinder(self)
    }

    fn ccd_thickness(&self) -> T {
        self.radius
    }

    fn ccd_angular_thickness(&self) -> T {
        T::constant(f64::frac_pi_2())
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        Some((self as &dyn PolygonalFeatureMap<T>, T::zero()))
    }
}

#[cfg(feature = "dim3")]
impl<T: AD> Shape<T> for Cone<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: T) -> MassProperties<T> {
        MassProperties::from_cone(density, self.half_height, self.radius)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Cone
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::Cone(self)
    }

    fn ccd_thickness(&self) -> T {
        self.radius
    }

    fn ccd_angular_thickness(&self) -> T {
        let apex_half_angle = self.radius.atan2(self.half_height);
        assert!(apex_half_angle >= T::zero());
        let basis_angle = T::constant(f64::frac_pi_2()) - apex_half_angle;
        basis_angle.min(apex_half_angle * T::constant(2.0))
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
        Some(self as &dyn SupportMap<T>)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
        Some((self as &dyn PolygonalFeatureMap<T>, T::zero()))
    }
}

impl<T: AD> Shape<T> for HalfSpace<T> {
    #[cfg(feature = "std")]
    fn clone_box(&self) -> Box<dyn Shape<T>> {
        Box::new(self.clone())
    }

    fn compute_local_aabb(&self) -> Aabb<T> {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
        self.aabb(position)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn ccd_thickness(&self) -> T {
        T::constant(f64::MAX)
    }

    fn ccd_angular_thickness(&self) -> T {
        T::constant(f64::pi())
    }

    fn mass_properties(&self, _: T) -> MassProperties<T> {
        MassProperties::zero()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::HalfSpace
    }

    fn as_typed_shape(&self) -> TypedShape<T> {
        TypedShape::HalfSpace(self)
    }
}

macro_rules! impl_shape_for_round_shape(
    ($($S: ty, $Tag: ident);*) => {$(
        impl<T: AD> Shape<T> for RoundShape<$S, T> {
            #[cfg(feature = "std")]
            fn clone_box(&self) -> Box<dyn Shape<T>> {
                Box::new(self.clone())
            }

            fn compute_local_aabb(&self) -> Aabb<T> {
                self.inner_shape.local_aabb().loosened(self.border_radius)
            }

            fn compute_local_bounding_sphere(&self) -> BoundingSphere<T> {
                self.inner_shape.local_bounding_sphere().loosened(self.border_radius)
            }

            fn compute_aabb(&self, position: &Isometry<T>) -> Aabb<T> {
                self.inner_shape.aabb(position).loosened(self.border_radius)
            }

            fn mass_properties(&self, density: T) -> MassProperties<T> {
                self.inner_shape.mass_properties(density)
            }

            fn is_convex(&self) -> bool {
                self.inner_shape.is_convex()
            }

            fn shape_type(&self) -> ShapeType {
                ShapeType::$Tag
            }

            fn as_typed_shape(&self) -> TypedShape<T> {
                TypedShape::$Tag(self)
            }

            fn ccd_thickness(&self) -> T {
                self.inner_shape.ccd_thickness() + self.border_radius
            }

            fn ccd_angular_thickness(&self) -> T {
                // The fact that the shape is round doesn't change anything
                // to the CCD angular thickness.
                self.inner_shape.ccd_angular_thickness()
            }

            fn as_support_map(&self) -> Option<&dyn SupportMap<T>> {
                Some(self as &dyn SupportMap<T>)
            }

            fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap<T>, T)> {
                Some((&self.inner_shape as &dyn PolygonalFeatureMap<T>, self.border_radius))
            }
        }
    )*}
);

impl_shape_for_round_shape!(
    Cuboid<T>, RoundCuboid;
    Triangle<T>, RoundTriangle
);

#[cfg(feature = "dim2")]
#[cfg(feature = "std")]
impl_shape_for_round_shape!(ConvexPolygon, RoundConvexPolygon);
#[cfg(feature = "dim3")]
impl_shape_for_round_shape!(
    Cylinder<T>, RoundCylinder;
    Cone<T>, RoundCone
);

#[cfg(feature = "dim3")]
#[cfg(feature = "std")]
impl_shape_for_round_shape!(ConvexPolyhedron<T>, RoundConvexPolyhedron);
