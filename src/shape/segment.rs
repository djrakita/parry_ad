//! Definition of the segment shape.

use crate::math::{Isometry, Point, Vector};
use crate::shape::{FeatureId, SupportMap};

use na::{self, Unit};
use std::mem;

use ad_trait::AD;

/// A segment shape.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(as = "Self"),
    archive(check_bytes)
)]
#[cfg_attr(feature = "cuda", derive(cust_core::DeviceCopy))]
#[derive(PartialEq, Debug, Copy, Clone)]
#[repr(C)]
pub struct Segment<T: AD> {
    /// The segment first point.
    pub a: Point<T>,
    /// The segment second point.
    pub b: Point<T>,
}

/// Logical description of the location of a point on a triangle.
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum SegmentPointLocation<T: AD> {
    /// The point lies on a vertex.
    OnVertex(u32),
    /// The point lies on the segment interior.
    OnEdge([T; 2]),
}

impl<T: AD> SegmentPointLocation<T> {
    /// The barycentric coordinates corresponding to this point location.
    pub fn barycentric_coordinates(&self) -> [T; 2] {
        let mut bcoords = [T::zero(); 2];

        match self {
            SegmentPointLocation::OnVertex(i) => bcoords[*i as usize] = T::constant(1.0),
            SegmentPointLocation::OnEdge(uv) => {
                bcoords[0] = uv[0];
                bcoords[1] = uv[1];
            }
        }

        bcoords
    }
}

impl<T: AD> Segment<T> {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: Point<T>, b: Point<T>) -> Segment<T> {
        Segment { a, b }
    }

    /// Creates the reference to a segment from the reference to an array of two points.
    pub fn from_array(arr: &[Point<T>; 2]) -> &Segment<T> {
        unsafe { mem::transmute(arr) }
    }

    /// Computes a scaled version of this segment.
    pub fn scaled(self, scale: &Vector<T>) -> Self {
        Self::new(
            na::Scale::from(*scale) * self.a,
            na::Scale::from(*scale) * self.b,
        )
    }

    /// The direction of this segment scaled by its length.
    ///
    /// Points from `self.a` toward `self.b`.
    pub fn scaled_direction(&self) -> Vector<T> {
        self.b - self.a
    }

    /// The length of this segment.
    pub fn length(&self) -> T {
        self.scaled_direction().norm()
    }

    /// Swaps the two vertices of this segment.
    pub fn swap(&mut self) {
        mem::swap(&mut self.a, &mut self.b)
    }

    /// The unit direction of this segment.
    ///
    /// Points from `self.a()` toward `self.b()`.
    /// Returns `None` is both points are equal.
    pub fn direction(&self) -> Option<Unit<Vector<T>>> {
        Unit::try_new(self.scaled_direction(), T::constant(crate::math::DEFAULT_EPSILON))
    }

    /// In 2D, the not-normalized counterclockwise normal of this segment.
    #[cfg(feature = "dim2")]
    pub fn scaled_normal(&self) -> Vector<T> {
        let dir = self.scaled_direction();
        Vector::new(dir.y, -dir.x)
    }

    /// The not-normalized counterclockwise normal of this segment, assuming it lies on the plane
    /// with the normal collinear to the given axis (0 = X, 1 = Y, 2 = Z).
    #[cfg(feature = "dim3")]
    pub fn scaled_planar_normal(&self, plane_axis: u8) -> Vector<T> {
        let dir = self.scaled_direction();
        match plane_axis {
            0 => Vector::new(T::zero(), dir.z, -dir.y),
            1 => Vector::new(-dir.z, T::zero(), dir.x),
            2 => Vector::new(dir.y, -dir.x, T::zero()),
            _ => panic!("Invalid axis given: must be 0 (X axis), 1 (Y axis) or 2 (Z axis)"),
        }
    }

    /// In 2D, the normalized counterclockwise normal of this segment.
    #[cfg(feature = "dim2")]
    pub fn normal(&self) -> Option<Unit<Vector<T>>> {
        Unit::try_new(self.scaled_normal(), crate::math::DEFAULT_EPSILON)
    }

    /// Returns `None`. Exists only for API similarity with the 2D parry.
    #[cfg(feature = "dim3")]
    pub fn normal(&self) -> Option<Unit<Vector<T>>> {
        None
    }

    /// The normalized counterclockwise normal of this segment, assuming it lies on the plane
    /// with the normal collinear to the given axis (0 = X, 1 = Y, 2 = Z).
    #[cfg(feature = "dim3")]
    pub fn planar_normal(&self, plane_axis: u8) -> Option<Unit<Vector<T>>> {
        Unit::try_new(
            self.scaled_planar_normal(plane_axis),
            T::constant(crate::math::DEFAULT_EPSILON),
        )
    }

    /// Applies the isometry `m` to the vertices of this segment and returns the resulting segment.
    pub fn transformed(&self, m: &Isometry<T>) -> Self {
        Segment::new(m * self.a, m * self.b)
    }

    /// Computes the point at the given location.
    pub fn point_at(&self, location: &SegmentPointLocation<T>) -> Point<T> {
        match *location {
            SegmentPointLocation::OnVertex(0) => self.a,
            SegmentPointLocation::OnVertex(1) => self.b,
            SegmentPointLocation::OnEdge(bcoords) => {
                self.a * bcoords[0] + self.b.coords * bcoords[1]
            }
            _ => panic!(),
        }
    }

    /// The normal of the given feature of this shape.
    pub fn feature_normal(&self, feature: FeatureId) -> Option<Unit<Vector<T>>> {
        if let Some(direction) = self.direction() {
            match feature {
                FeatureId::Vertex(id) => {
                    if id == 0 {
                        Some(direction)
                    } else {
                        Some(-direction)
                    }
                }
                #[cfg(feature = "dim3")]
                FeatureId::Edge(_) => {
                    let iamin = direction.iamin();
                    let mut normal = Vector::zeros();
                    normal[iamin] = T::constant(1.0);
                    normal -= *direction * direction[iamin];
                    Some(Unit::new_normalize(normal))
                }
                FeatureId::Face(id) => {
                    let mut dir = Vector::zeros();
                    if id == 0 {
                        dir[0] = direction[1];
                        dir[1] = -direction[0];
                    } else {
                        dir[0] = -direction[1];
                        dir[1] = direction[0];
                    }
                    Some(Unit::new_unchecked(dir))
                }
                _ => None,
            }
        } else {
            Some(Vector::y_axis())
        }
    }
}

impl<T: AD> SupportMap<T> for Segment<T> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<T>) -> Point<T> {
        if self.a.coords.dot(dir) > self.b.coords.dot(dir) {
            self.a
        } else {
            self.b
        }
    }
}

impl<T: AD> From<[Point<T>; 2]> for Segment<T> {
    fn from(arr: [Point<T>; 2]) -> Self {
        *Self::from_array(&arr)
    }
}

/*
impl ConvexPolyhedron for Segment {
    fn vertex(&self, id: FeatureId) -> Point<Real> {
        if id.unwrap_vertex() == 0 {
            self.a
        } else {
            self.b
        }
    }

    #[cfg(feature = "dim3")]
    fn edge(&self, _: FeatureId) -> (Point<Real>, Point<Real>, FeatureId, FeatureId) {
        (self.a, self.b, FeatureId::Vertex(0), FeatureId::Vertex(1))
    }

    #[cfg(feature = "dim3")]
    fn face(&self, _: FeatureId, _: &mut ConvexPolygonalFeature) {
        panic!("A segment does not have any face in dimensions higher than 2.")
    }

    #[cfg(feature = "dim2")]
    fn face(&self, id: FeatureId, face: &mut ConvexPolygonalFeature) {
        face.clear();

        if let Some(normal) = utils::ccw_face_normal([&self.a, &self.b]) {
            face.set_feature_id(id);

            match id.unwrap_face() {
                0 => {
                    face.push(self.a, FeatureId::Vertex(0));
                    face.push(self.b, FeatureId::Vertex(1));
                    face.set_normal(normal);
                }
                1 => {
                    face.push(self.b, FeatureId::Vertex(1));
                    face.push(self.a, FeatureId::Vertex(0));
                    face.set_normal(-normal);
                }
                _ => unreachable!(),
            }
        } else {
            face.push(self.a, FeatureId::Vertex(0));
            face.set_feature_id(FeatureId::Vertex(0));
        }
    }

    #[cfg(feature = "dim2")]
    fn support_face_toward(
        &self,
        m: &Isometry<Real>,
        dir: &Unit<Vector<Real>>,
        face: &mut ConvexPolygonalFeature,
    ) {
        let seg_dir = self.scaled_direction();

        if dir.perp(&seg_dir) >= T::zero() {
            self.face(FeatureId::Face(0), face);
        } else {
            self.face(FeatureId::Face(1), face);
        }
        face.transform_by(m)
    }

    #[cfg(feature = "dim3")]
    fn support_face_toward(
        &self,
        m: &Isometry<Real>,
        _: &Unit<Vector<Real>>,
        face: &mut ConvexPolygonalFeature,
    ) {
        face.clear();
        face.push(self.a, FeatureId::Vertex(0));
        face.push(self.b, FeatureId::Vertex(1));
        face.push_edge_feature_id(FeatureId::Edge(0));
        face.set_feature_id(FeatureId::Edge(0));
        face.transform_by(m)
    }

    fn support_feature_toward(
        &self,
        transform: &Isometry<Real>,
        dir: &Unit<Vector<Real>>,
        eps: Real,
        face: &mut ConvexPolygonalFeature,
    ) {
        face.clear();
        let seg = self.transformed(transform);
        let ceps = ComplexField::sin(eps);

        if let Some(seg_dir) = seg.direction() {
            let cang = dir.dot(&seg_dir);

            if cang > ceps {
                face.set_feature_id(FeatureId::Vertex(1));
                face.push(seg.b, FeatureId::Vertex(1));
            } else if cang < -ceps {
                face.set_feature_id(FeatureId::Vertex(0));
                face.push(seg.a, FeatureId::Vertex(0));
            } else {
                #[cfg(feature = "dim3")]
                {
                    face.push(seg.a, FeatureId::Vertex(0));
                    face.push(seg.b, FeatureId::Vertex(1));
                    face.push_edge_feature_id(FeatureId::Edge(0));
                    face.set_feature_id(FeatureId::Edge(0));
                }
                #[cfg(feature = "dim2")]
                {
                    if dir.perp(&seg_dir) >= T::zero() {
                        seg.face(FeatureId::Face(0), face);
                    } else {
                        seg.face(FeatureId::Face(1), face);
                    }
                }
            }
        }
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<Real>>) -> FeatureId {
        if let Some(seg_dir) = self.direction() {
            let eps: Real = na::convert::<f64, Real>(f64::consts::PI / 18T::zero());
            let seps = ComplexField::sin(eps);
            let dot = seg_dir.dot(local_dir.as_ref());

            if dot <= seps {
                #[cfg(feature = "dim2")]
                {
                    if local_dir.perp(seg_dir.as_ref()) >= T::zero() {
                        FeatureId::Face(0)
                    } else {
                        FeatureId::Face(1)
                    }
                }
                #[cfg(feature = "dim3")]
                {
                    FeatureId::Edge(0)
                }
            } else if dot >= T::zero() {
                FeatureId::Vertex(1)
            } else {
                FeatureId::Vertex(0)
            }
        } else {
            FeatureId::Vertex(0)
        }
    }
}
*/
