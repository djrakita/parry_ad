#[cfg(not(feature = "std"))]
use na::ComplexField;
#[cfg(feature = "std")]
use na::DVector;
use std::ops::Range;

use ad_trait::AD;

use crate::utils::DefaultStorage;

#[cfg(feature = "cuda")]
use crate::utils::{CudaArrayPointer1, CudaStorage, CudaStoragePtr};

#[cfg(all(feature = "std", feature = "cuda"))]
use {crate::utils::CudaArray1, cust::error::CudaResult};

use na::Point2;

use crate::bounding_volume::Aabb;
use crate::math::{Vector};

use crate::shape::Segment;
use crate::utils::Array1;

/// Indicates if a cell of an heightfield is removed or not. Set this to `false` for
/// a removed cell.
pub type HeightFieldCellStatus = bool;

/// Trait describing all the types needed for storing an heightfield’s data.
pub trait HeightFieldStorage<T: AD> {
    /// Type of the array containing the heightfield’s heights.
    type Heights: Array1<T>;
    /// Type of the array containing the heightfield’s cells status.
    type Status: Array1<HeightFieldCellStatus>;
}

#[cfg(feature = "std")]
impl<T: AD> HeightFieldStorage<T> for DefaultStorage {
    type Heights = DVector<T>;
    type Status = DVector<HeightFieldCellStatus>;
}

#[cfg(all(feature = "std", feature = "cuda"))]
impl<T: AD> HeightFieldStorage<T> for CudaStorage {
    type Heights = CudaArray1<T>;
    type Status = CudaArray1<HeightFieldCellStatus>;
}

#[cfg(feature = "cuda")]
impl<T: AD> HeightFieldStorage<T> for CudaStoragePtr {
    type Heights = CudaArrayPointer1<T>;
    type Status = CudaArrayPointer1<HeightFieldCellStatus>;
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(check_bytes)
)]
#[derive(Debug)]
#[repr(C)] // Needed for Cuda.
/// A 2D heightfield with a generic storage buffer for its heights.
pub struct GenericHeightField<T: AD, Storage: HeightFieldStorage<T>> {
    heights: Storage::Heights,
    status: Storage::Status,

    scale: Vector<T>,
    aabb: Aabb,
}

impl<Storage, T: AD> Clone for GenericHeightField<Storage, T>
where
    Storage: HeightFieldStorage<T>,
    Storage::Heights: Clone,
    Storage::Status: Clone,
{
    fn clone(&self) -> Self {
        Self {
            heights: self.heights.clone(),
            status: self.status.clone(),
            scale: self.scale,
            aabb: self.aabb,
        }
    }
}

impl<Storage, T: AD> Copy for GenericHeightField<Storage, T>
where
    Storage: HeightFieldStorage<T>,
    Storage::Heights: Copy,
    Storage::Status: Copy,
{
}

#[cfg(feature = "cuda")]
unsafe impl<Storage, T: AD> cust_core::DeviceCopy for GenericHeightField<Storage, T>
where
    Storage: HeightFieldStorage<T>,
    Storage::Heights: cust_core::DeviceCopy + Copy,
    Storage::Status: cust_core::DeviceCopy + Copy,
{
}

/// A 2D heightfield.
#[cfg(feature = "std")]
pub type HeightField<T: AD> = GenericHeightField<DefaultStorage, T>;

/// A 2D heightfield stored in the CUDA memory, initializable from the host.
#[cfg(all(feature = "std", feature = "cuda"))]
pub type CudaHeightField<T: AD> = GenericHeightField<CudaStorage, T>;

/// A 2D heightfield stored in the CUDA memory, accessible from within a Cuda kernel.
#[cfg(feature = "cuda")]
pub type CudaHeightFieldPtr<T: AD> = GenericHeightField<CudaStoragePtr, T>;

#[cfg(feature = "std")]
impl<T: AD> HeightField<T> {
    /// Creates a new 2D heightfield with the given heights and scale factor.
    pub fn new(heights: DVector<T>, scale: Vector<T>) -> Self {
        assert!(
            heights.len() > 1,
            "A heightfield heights must have at least 2 elements."
        );

        let max = heights.max();
        let min = heights.min();
        let hscale = scale * T::constant(0.5);
        let aabb = Aabb::new(
            Point2::new(-hscale.x, min * scale.y),
            Point2::new(hscale.x, max * scale.y),
        );
        let num_segments = heights.len() - 1;

        HeightField {
            heights,
            status: DVector::repeat(num_segments, true),
            scale,
            aabb,
        }
    }

    /// Converts this RAM-based heightfield to an heightfield based on CUDA memory.
    #[cfg(all(feature = "cuda"))]
    pub fn to_cuda(&self) -> CudaResult<CudaHeightField<T>> {
        Ok(CudaHeightField {
            heights: CudaArray1::from_vector(&self.heights)?,
            status: CudaArray1::from_vector(&self.status)?,
            scale: self.scale,
            aabb: self.aabb,
        })
    }
}

#[cfg(all(feature = "std", feature = "cuda"))]
impl<T: AD> CudaHeightField<T> {
    /// Returns the heightfield usable from within a CUDA kernel.
    pub fn as_device_ptr(&self) -> CudaHeightFieldPtr<T> {
        CudaHeightFieldPtr {
            heights: self.heights.as_device_ptr(),
            status: self.status.as_device_ptr(),
            aabb: self.aabb,
            scale: self.scale,
        }
    }
}

impl<T: AD, Storage: HeightFieldStorage<T>> GenericHeightField<Storage, T> {
    /// The number of cells of this heightfield.
    pub fn num_cells(&self) -> usize {
        self.heights.len() - 1
    }

    /// The height at each cell endpoint.
    pub fn heights(&self) -> &Storage::Heights {
        &self.heights
    }

    /// The scale factor applied to this heightfield.
    pub fn scale(&self) -> &Vector<T> {
        &self.scale
    }

    /// Sets the scale factor applied to this heightfield.
    pub fn set_scale(&mut self, new_scale: Vector<T>) {
        let ratio = new_scale.component_div(&self.scale);
        self.aabb.mins.coords.component_mul_assign(&ratio);
        self.aabb.maxs.coords.component_mul_assign(&ratio);
        self.scale = new_scale;
    }

    /// Returns a scaled version of this heightfield.
    pub fn scaled(mut self, scale: &Vector<T>) -> Self {
        self.set_scale(self.scale.component_mul(&scale));
        self
    }

    /// The Aabb of this heightfield.
    pub fn root_aabb(&self) -> &Aabb {
        &self.aabb
    }

    /// The width of a single cell of this heightfield.
    pub fn cell_width(&self) -> T {
        self.unit_cell_width() * self.scale.x
    }

    /// The width of a single cell of this heightfield, without taking the scale factor into account.
    pub fn unit_cell_width(&self) -> T {
        T::constant(1.0 / (self.heights.len() as f64 - 1.0))
    }

    /// The left-most x-coordinate of this heightfield.
    pub fn start_x(&self) -> T {
        self.scale.x * T::constant(-0.5)
    }

    fn quantize_floor_unclamped(&self, val: T, seg_length: T) -> isize {
        ((val + T::constant(0.5)) / seg_length).floor() as isize
    }

    fn quantize_ceil_unclamped(&self, val: T, seg_length: T) -> isize {
        ((val + T::constant(0.5)) / seg_length).ceil() as isize
    }

    fn quantize_floor(&self, val: T, seg_length: T) -> usize {
        na::clamp(
            ((val + T::constant(0.5)) / seg_length).floor(),
            T::zero(),
            T::constant((self.num_cells() - 1) as f64),
        ) as usize
    }

    fn quantize_ceil(&self, val: T, seg_length: T) -> usize {
        na::clamp(
            ((val + T::constant(0.5)) / seg_length).ceil(),
            T::zero(),
            T::constant(self.num_cells() as f64),
        ) as usize
    }

    /// Index of the cell a point is on after vertical projection.
    pub fn cell_at_point(&self, pt: &Point2<T>) -> Option<usize> {
        let scaled_pt = pt.coords.component_div(&self.scale);
        let seg_length = self.unit_cell_width();

        if scaled_pt.x < T::constant(-0.5) || scaled_pt.x > T::constant(0.5) {
            // Outside of the heightfield bounds.
            None
        } else {
            Some(self.quantize_floor(scaled_pt.x, seg_length))
        }
    }

    /// Height of the heightfield a the given point after vertical projection on the heightfield surface.
    pub fn height_at_point(&self, pt: &Point2<T>) -> Option<T> {
        let cell = self.cell_at_point(pt)?;
        let seg = self.segment_at(cell)?;
        let inter = crate::query::details::closest_points_line_line_parameters(
            &seg.a,
            &seg.scaled_direction(),
            &pt,
            &Vector::y(),
        );
        Some(seg.a.y + inter.1)
    }

    /// Iterator through all the segments of this heightfield.
    pub fn segments<'a>(&'a self) -> impl Iterator<Item = Segment> + 'a {
        // FIXME: this is not very efficient since this wil
        // recompute shared points twice.
        (0..self.num_cells()).filter_map(move |i| self.segment_at(i))
    }

    /// The i-th segment of the heightfield if it has not been removed.
    pub fn segment_at(&self, i: usize) -> Option<Segment> {
        if i >= self.num_cells() || self.is_segment_removed(i) {
            return None;
        }

        let seg_length = T::constant(1.0 / (self.heights.len() as f64 - 1.0));

        let x0 = T::constant(-0.5) + seg_length * T::constant(i as f64);
        let x1 = x0 + seg_length;

        let y0 = self.heights[i + 0];
        let y1 = self.heights[i + 1];

        let mut p0 = Point2::new(x0, y0);
        let mut p1 = Point2::new(x1, y1);

        // Apply scales:
        p0.coords.component_mul_assign(&self.scale);
        p1.coords.component_mul_assign(&self.scale);

        Some(Segment::new(p0, p1))
    }

    /// Mark the i-th segment of this heightfield as removed or not.
    pub fn set_segment_removed(&mut self, i: usize, removed: bool) {
        self.status[i] = !removed
    }

    /// Checks if the i-th segment has been removed.
    pub fn is_segment_removed(&self, i: usize) -> bool {
        !self.status[i]
    }

    /// The range of segment ids that may intersect the given local Aabb.
    pub fn unclamped_elements_range_in_local_aabb(&self, aabb: &Aabb) -> Range<isize> {
        let ref_mins = aabb.mins.coords.component_div(&self.scale);
        let ref_maxs = aabb.maxs.coords.component_div(&self.scale);
        let seg_length = T::constant(1.0 / (self.heights.len() as f64 - 1.0));

        let min_x = self.quantize_floor_unclamped(ref_mins.x, seg_length);
        let max_x = self.quantize_ceil_unclamped(ref_maxs.x, seg_length);
        min_x..max_x
    }

    /// Applies `f` to each segment of this heightfield that intersects the given `aabb`.
    pub fn map_elements_in_local_aabb(&self, aabb: &Aabb, f: &mut impl FnMut(u32, &Segment)) {
        let ref_mins = aabb.mins.coords.component_div(&self.scale);
        let ref_maxs = aabb.maxs.coords.component_div(&self.scale);
        let seg_length = T::constant(1.0 / (self.heights.len() as f64 - 1.0));

        if ref_maxs.x < T::constant(-0.5) || ref_mins.x > T::constant(0.5) {
            // Outside of the heightfield bounds.
            return;
        }

        let min_x = self.quantize_floor(ref_mins.x, seg_length);
        let max_x = self.quantize_ceil(ref_maxs.x, seg_length);

        // FIXME: find a way to avoid recomputing the same vertices
        // multiple times.
        for i in min_x..max_x {
            if self.is_segment_removed(i) {
                continue;
            }

            let x0 = T::constant(-0.5) + seg_length * T::constant(i as f64);
            let x1 = x0 + seg_length;

            let y0 = self.heights[i + 0];
            let y1 = self.heights[i + 1];

            if (y0 > ref_maxs.y && y1 > ref_maxs.y) || (y0 < ref_mins.y && y1 < ref_mins.y) {
                continue;
            }

            let mut p0 = Point2::new(x0, y0);
            let mut p1 = Point2::new(x1, y1);

            // Apply scales:
            p0.coords.component_mul_assign(&self.scale);
            p1.coords.component_mul_assign(&self.scale);

            // Build the segment.
            let seg = Segment::new(p0, p1);

            // Call the callback.
            f(i as u32, &seg);
        }
    }
}
