use crate::bounding_volume::Aabb;
use crate::math::{Isometry, Point, Vector, DIM, SIMD_WIDTH};
use crate::query::SimdRay;
use crate::utils::{self, IsometryOps};
use simba::simd::{SimdValue};
use ad_trait::AD;

/// Four Aabb represented as a single SoA Aabb with SIMD components.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(check_bytes)
)]
#[cfg_attr(feature = "cuda", derive(cust_core::DeviceCopy))]
pub struct SimdAabb<T: AD> {
    /// The min coordinates of the Aabbs.
    pub mins: Point<T>,
    /// The max coordinates the Aabbs.
    pub maxs: Point<T>,
}

#[cfg(feature = "serde-serialize")]
impl<T: AD> serde::Serialize for SimdAabb<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;

        let mins: Point<[T; SIMD_WIDTH]> = Point::from(
            self.mins
                .coords
                .map(|e| array![|ii| e.extract(ii); SIMD_WIDTH]),
        );
        let maxs: Point<[T; SIMD_WIDTH]> = Point::from(
            self.maxs
                .coords
                .map(|e| array![|ii| e.extract(ii); SIMD_WIDTH]),
        );

        let mut simd_aabb = serializer.serialize_struct("SimdAabb", 2)?;
        simd_aabb.serialize_field("mins", &mins)?;
        simd_aabb.serialize_field("maxs", &maxs)?;
        simd_aabb.end()
    }
}

#[cfg(feature = "serde-serialize")]
impl<'de, T: AD> serde::Deserialize<'de> for SimdAabb<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct Visitor<T: AD> {}

        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        enum Field {
            Mins,
            Maxs,
        }

        impl<'de, T: AD> serde::de::Visitor<'de> for Visitor<T> {
            type Value = SimdAabb<T>;
            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(
                    formatter,
                    "two arrays containing at least {} floats",
                    SIMD_WIDTH * DIM * 2
                )
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::MapAccess<'de>,
            {
                let mut mins: Option<Point<[T; SIMD_WIDTH]>> = None;
                let mut maxs: Option<Point<[T; SIMD_WIDTH]>> = None;

                while let Some(key) = map.next_key()? {
                    match key {
                        Field::Mins => {
                            if mins.is_some() {
                                return Err(serde::de::Error::duplicate_field("mins"));
                            }
                            mins = Some(map.next_value()?);
                        }
                        Field::Maxs => {
                            if maxs.is_some() {
                                return Err(serde::de::Error::duplicate_field("maxs"));
                            }
                            maxs = Some(map.next_value()?);
                        }
                    }
                }

                let mins = mins.ok_or_else(|| serde::de::Error::missing_field("mins"))?;
                let maxs = maxs.ok_or_else(|| serde::de::Error::missing_field("maxs"))?;
                let mins = mins; // mins.map(SimdReal::from);
                let maxs = maxs; // maxs.map(SimdReal::from);
                Ok(SimdAabb { mins, maxs })
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mins: Point<[T; SIMD_WIDTH]> = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;
                let maxs: Point<[T; SIMD_WIDTH]> = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(1, &self))?;
                // let mins = mins.map(SimdReal::from);
                // let maxs = maxs.map(SimdReal::from);
                Ok(SimdAabb { mins, maxs })
            }
        }

        deserializer.deserialize_struct("SimdAabb", &["mins", "maxs"], Visitor {})
    }
}

impl<T: AD> SimdAabb<T> {
    /// An invalid Aabb.
    pub fn new_invalid() -> Self {
        Self::splat(Aabb::new_invalid())
    }

    /// Builds an SIMD aabb composed of four identical aabbs.
    pub fn splat(aabb: Aabb<T>) -> Self {
        Self {
            mins: Point::splat(aabb.mins),
            maxs: Point::splat(aabb.maxs),
        }
    }

    /// The center of all the Aabbs represented by `self``.
    pub fn center(&self) -> Point<T> {
        na::center(&self.mins, &self.maxs)
    }

    /// The half-extents of all the Aabbs represented by `self``.
    pub fn half_extents(&self) -> Vector<T> {
        (self.maxs - self.mins) * T::constant(0.5)
    }

    /// The radius of all the Aabbs represented by `self``.
    pub fn radius(&self) -> T {
        (self.maxs - self.mins).norm()
    }

    /// Return the Aabb of the `self` transformed by the given isometry.
    pub fn transform_by(&self, transform: &Isometry<T>) -> Self {
        let ls_center = self.center();
        let center = transform * ls_center;
        let ws_half_extents = transform.absolute_transform_vector(&self.half_extents());
        Self {
            mins: center + (-ws_half_extents),
            maxs: center + ws_half_extents,
        }
    }

    /// Returns a scaled version of this Aabb.
    #[inline]
    pub fn scaled(self, scale: &Vector<T>) -> Self {
        let a = self.mins.coords.component_mul(&scale);
        let b = self.maxs.coords.component_mul(&scale);
        Self {
            mins: a.inf(&b).into(),
            maxs: a.sup(&b).into(),
        }
    }

    /// Enlarges this bounding volume by the given margin.
    pub fn loosen(&mut self, margin: T) {
        self.mins -= Vector::repeat(margin);
        self.maxs += Vector::repeat(margin);
    }

    /// Dilate all the Aabbs represented by `self`` by their extents multiplied
    /// by the given scale `factor`.
    pub fn dilate_by_factor(&mut self, factor: T) {
        // If some of the Aabbs on this SimdAabb are invalid,
        // don't, dilate them.
        let is_valid = self.mins.x.simd_le(self.maxs.x);
        let factor = factor.select(is_valid, T::zero());

        // NOTE: we multiply each by factor instead of doing
        // (maxs - mins) * factor. That's to avoid overflows (and
        // therefore NaNs if this SimdAabb contains some invalid
        // Aabbs initialised with Real::MAX
        let dilation = self.maxs * factor - self.mins * factor;
        self.mins -= dilation;
        self.maxs += dilation;
    }

    /// Replace the `i-th` Aabb of this SIMD AAAB by the given value.
    pub fn replace(&mut self, i: usize, aabb: Aabb<T>) {
        self.mins.replace(i, aabb.mins);
        self.maxs.replace(i, aabb.maxs);
    }

    /// Casts a ray on all the Aabbs represented by `self`.
    pub fn cast_local_ray(&self, ray: &SimdRay<T>, max_toi: T) -> (bool, T) {
        let zero = T::zero();
        let one = T::one();
        let infinity = T::constant(f64::MAX);

        let mut hit = true;
        let mut tmin = T::zero();
        let mut tmax = max_toi;

        // TODO: could this be optimized more considering we really just need a boolean answer?
        for i in 0usize..DIM {
            let is_not_zero = ray.dir[i].simd_ne(zero);
            let is_zero_test =
                ray.origin[i].simd_ge(self.mins[i]) & ray.origin[i].simd_le(self.maxs[i]);
            let is_not_zero_test = {
                let denom = one / ray.dir[i];
                let tmp1: T = (self.mins[i] - ray.origin[i]) * denom;
                let mut inter_with_near_plane =
                    tmp1.select(is_not_zero, -infinity);
                let tmp2: T = (self.maxs[i] - ray.origin[i]) * denom;
                let mut inter_with_far_plane =
                    tmp2.select(is_not_zero, infinity);

                let gt = inter_with_near_plane.simd_gt(inter_with_far_plane);
                utils::simd_swap(gt, &mut inter_with_near_plane, &mut inter_with_far_plane);

                tmin = tmin.simd_max(inter_with_near_plane);
                tmax = tmax.simd_min(inter_with_far_plane);

                tmin.simd_le(tmax)
            };

            hit = hit & is_not_zero_test.select(is_not_zero, is_zero_test);
        }

        (hit, tmin)
    }

    /// Computes the distances between a point and all the Aabbs represented by `self`.
    pub fn distance_to_local_point(&self, point: &Point<T>) -> T {
        let mins_point = self.mins - point;
        let point_maxs = point - self.maxs;
        let shift = mins_point.sup(&point_maxs).sup(&na::zero());
        shift.norm()
    }

    /// Computes the distances between the origin and all the Aabbs represented by `self`.
    pub fn distance_to_origin(&self) -> T {
        self.mins
            .coords
            .sup(&-self.maxs.coords)
            .sup(&Vector::zeros())
            .norm()
    }

    /// Check which Aabb represented by `self` contains the given `point`.
    pub fn contains_local_point(&self, point: &Point<T>) -> bool {
        #[cfg(feature = "dim2")]
        return self.mins.x.simd_le(point.x)
            & self.mins.y.simd_le(point.y)
            & self.maxs.x.simd_ge(point.x)
            & self.maxs.y.simd_ge(point.y);

        #[cfg(feature = "dim3")]
        return self.mins.x.simd_le(point.x)
            & self.mins.y.simd_le(point.y)
            & self.mins.z.simd_le(point.z)
            & self.maxs.x.simd_ge(point.x)
            & self.maxs.y.simd_ge(point.y)
            & self.maxs.z.simd_ge(point.z);
    }

    /// Lanewise check which Aabb represented by `self` contains the given set of `other` aabbs.
    /// The check is performed lane-wise.
    #[cfg(feature = "dim2")]
    pub fn contains(&self, other: &SimdAabb) -> bool {
        self.mins.x.simd_le(other.mins.x)
            & self.mins.y.simd_le(other.mins.y)
            & self.maxs.x.simd_ge(other.maxs.x)
            & self.maxs.y.simd_ge(other.maxs.y)
    }

    /// Lanewise check which Aabb represented by `self` contains the given set of `other` aabbs.
    /// The check is performed lane-wise.
    #[cfg(feature = "dim3")]
    pub fn contains(&self, other: &SimdAabb<T>) -> bool {
        self.mins.x.simd_le(other.mins.x)
            & self.mins.y.simd_le(other.mins.y)
            & self.mins.z.simd_le(other.mins.z)
            & self.maxs.x.simd_ge(other.maxs.x)
            & self.maxs.y.simd_ge(other.maxs.y)
            & self.maxs.z.simd_ge(other.maxs.z)
    }

    /// Lanewise check which Aabb represented by `self` intersects the given set of `other` aabbs.
    /// The check is performed lane-wise.
    #[cfg(feature = "dim2")]
    pub fn intersects(&self, other: &SimdAabb<T>) -> bool {
        self.mins.x.simd_le(other.maxs.x)
            & other.mins.x.simd_le(self.maxs.x)
            & self.mins.y.simd_le(other.maxs.y)
            & other.mins.y.simd_le(self.maxs.y)
    }

    /// Check which Aabb represented by `self` contains the given set of `other` aabbs.
    /// The check is performed lane-wise.
    #[cfg(feature = "dim3")]
    pub fn intersects(&self, other: &SimdAabb<T>) -> bool {
        self.mins.x.simd_le(other.maxs.x)
            & other.mins.x.simd_le(self.maxs.x)
            & self.mins.y.simd_le(other.maxs.y)
            & other.mins.y.simd_le(self.maxs.y)
            & self.mins.z.simd_le(other.maxs.z)
            & other.mins.z.simd_le(self.maxs.z)
    }

    /// Checks intersections between all the lanes combination between `self` and `other`.
    ///
    /// The result is an array such that `result[i].extract(j)` contains the intersection
    /// result between `self.extract(i)` and `other.extract(j)`.
    pub fn intersects_permutations(&self, other: &SimdAabb<T>) -> [bool; SIMD_WIDTH] {
        let mut result = [false; SIMD_WIDTH];
        for ii in 0..SIMD_WIDTH {
            // TODO: use SIMD-accelerated shuffling?
            let extracted = SimdAabb::splat(self.extract(ii));
            result[ii] = extracted.intersects(other);
        }

        result
    }

    /// Merge all the Aabb represented by `self` into a single one.
    pub fn to_merged_aabb(&self) -> Aabb<T> {
        Aabb::new(
            self.mins.coords.map(|e| e.simd_horizontal_min()).into(),
            self.maxs.coords.map(|e| e.simd_horizontal_max()).into(),
        )
    }

    /// Extracts the Aabb stored in the given SIMD lane of the SIMD Aabb:
    pub fn extract(&self, lane: usize) -> Aabb<T> {
        Aabb::new(self.mins.extract(lane), self.maxs.extract(lane))
    }
}

impl<T: AD> From<[Aabb<T>; SIMD_WIDTH]> for SimdAabb<T> {
    fn from(aabbs: [Aabb<T>; SIMD_WIDTH]) -> Self {
        // const SIMD_WIDTH: usize = SIMD_WIDTH;
        // let mins = array![|ii| aabbs[ii].mins; SIMD_WIDTH];
        // let maxs = array![|ii| aabbs[ii].maxs; SIMD_WIDTH];

        let mins = aabbs[0].mins;
        let maxs = aabbs[0].maxs;

        SimdAabb {
            mins: mins,
            maxs: maxs,
        }
    }
}
