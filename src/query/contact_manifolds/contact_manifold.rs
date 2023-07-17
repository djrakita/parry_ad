use crate::math::{Isometry, Point, Vector};
use crate::shape::PackedFeatureId;
use ad_trait::AD;

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize),
    archive(check_bytes)
)]
/// A single contact between two shape.
pub struct TrackedContact<T: AD, Data> {
    /// The contact point in the local-space of the first shape.
    pub local_p1: Point<T>,
    /// The contact point in the local-space of the second shape.
    pub local_p2: Point<T>,
    /// The distance between the two contact points.
    pub dist: T,

    /// The feature ID of the first shape involved in the contact.
    pub fid1: PackedFeatureId,
    /// The feature ID of the second shape involved in the contact.
    pub fid2: PackedFeatureId,
    /// User-data associated to this contact.
    pub data: Data,
}

impl<T: AD, Data: Default + Copy> TrackedContact<T, Data> {
    /// Creates a new tracked contact.
    pub fn new(
        local_p1: Point<T>,
        local_p2: Point<T>,
        fid1: PackedFeatureId,
        fid2: PackedFeatureId,
        dist: T,
    ) -> Self {
        Self {
            local_p1,
            local_p2,
            fid1,
            fid2,
            dist,
            data: Data::default(),
        }
    }

    /// Creates a new tracked contact where its input may need to be flipped.
    pub fn flipped(
        local_p1: Point<T>,
        local_p2: Point<T>,
        fid1: PackedFeatureId,
        fid2: PackedFeatureId,
        dist: T,
        flipped: bool,
    ) -> Self {
        if !flipped {
            Self::new(local_p1, local_p2, fid1, fid2, dist)
        } else {
            Self::new(local_p2, local_p1, fid2, fid1, dist)
        }
    }

    /// Copy to `self` the geometric information from `contact`.
    pub fn copy_geometry_from(&mut self, contact: Self) {
        self.local_p1 = contact.local_p1;
        self.local_p2 = contact.local_p2;
        self.fid1 = contact.fid1;
        self.fid2 = contact.fid2;
        self.dist = contact.dist;
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A contact manifold between two shapes.
///
/// A contact manifold describes a set of contacts between two shapes. All the contact
/// part of the same contact manifold share the same contact normal and contact kinematics.
pub struct ContactManifold<ManifoldData, ContactData, T: AD> {
    // NOTE: use a SmallVec instead?
    // And for 2D use an ArrayVec since there will never be more than 2 contacts anyways.
    /// The contacts points.
    #[cfg(feature = "dim2")]
    pub points: arrayvec::ArrayVec<TrackedContact<ContactData>, 2>,
    /// The contacts points.
    #[cfg(feature = "dim3")]
    pub points: Vec<TrackedContact<T, ContactData>>,
    /// The contact normal of all the contacts of this manifold, expressed in the local space of the first shape.
    pub local_n1: Vector<T>,
    /// The contact normal of all the contacts of this manifold, expressed in the local space of the second shape.
    pub local_n2: Vector<T>,
    /// The first subshape involved in this contact manifold.
    ///
    /// This is zero if the first shape is not a composite shape.
    pub subshape1: u32,
    /// The second subshape involved in this contact manifold.
    ///
    /// This is zero if the second shape is not a composite shape.
    pub subshape2: u32,
    /// If the first shape involved is a composite shape, this contains the position of its subshape
    /// involved in this contact.
    pub subshape_pos1: Option<Isometry<T>>,
    /// If the second shape involved is a composite shape, this contains the position of its subshape
    /// involved in this contact.
    pub subshape_pos2: Option<Isometry<T>>,
    /// Additional tracked data associated to this contact manifold.
    pub data: ManifoldData,
}

impl<T: AD, ManifoldData, ContactData: Default + Copy> ContactManifold<ManifoldData, ContactData, T> {
    /// Create a new empty contact-manifold.
    pub fn new() -> Self
    where
        ManifoldData: Default,
    {
        Self::with_data(0, 0, ManifoldData::default())
    }

    /// Create a new empty contact-manifold with the given associated data.
    pub fn with_data(subshape1: u32, subshape2: u32, data: ManifoldData) -> Self {
        Self {
            #[cfg(feature = "dim2")]
            points: arrayvec::ArrayVec::new(),
            #[cfg(feature = "dim3")]
            points: Vec::new(),
            local_n1: Vector::zeros(),
            local_n2: Vector::zeros(),
            subshape1,
            subshape2,
            subshape_pos1: None,
            subshape_pos2: None,
            data,
        }
    }

    /// Clones `self` and then remove all contact points from `self`.
    pub fn take(&mut self) -> Self
    where
        ManifoldData: Clone,
    {
        #[cfg(feature = "dim2")]
        let points = self.points.clone();
        #[cfg(feature = "dim3")]
        let points = std::mem::replace(&mut self.points, Vec::new());
        self.points.clear();

        ContactManifold {
            points,
            local_n1: self.local_n1,
            local_n2: self.local_n2,
            subshape1: self.subshape1,
            subshape2: self.subshape2,
            subshape_pos1: self.subshape_pos1,
            subshape_pos2: self.subshape_pos2,
            data: self.data.clone(),
        }
    }

    /*
    pub(crate) fn single_manifold<'a, 'b>(
        manifolds: &mut Vec<Self>,
        data: &dyn Fn() -> ManifoldData,
    ) -> &'a mut Self {
        if manifolds.len() == 0 {
            let manifold_data = data();
            manifolds.push(ContactManifold::with_data((0, 0), manifold_data));
        }

        &mut manifolds[0]
    }
    */

    /// The slice of all the contacts, active or not, on this contact manifold.
    #[inline]
    pub fn contacts(&self) -> &[TrackedContact<T, ContactData>] {
        &self.points
    }

    /// Attempts to use spatial coherence to update contacts points.
    #[inline]
    pub fn try_update_contacts(&mut self, pos12: &Isometry<T>) -> bool {
        // const DOT_THRESHOLD: T = 0.crate::COS_10_DEGREES;
        // const DOT_THRESHOLD: T = crate::utils::COS_5_DEGREES;
        let dot_threshold = T::constant(crate::utils::COS_1_DEGREES);
        let dist_sq_threshold = T::constant(1.0e-6); // FIXME: this should not be hard-coded.
        self.try_update_contacts_eps(pos12, dot_threshold, dist_sq_threshold)
    }

    /// Attempts to use spatial coherence to update contacts points, using user-defined tolerances.
    #[inline]
    pub fn try_update_contacts_eps(
        &mut self,
        pos12: &Isometry<T>,
        angle_dot_threshold: T,
        dist_sq_threshold: T,
    ) -> bool {
        if self.points.len() == 0 {
            return false;
        }

        let local_n2 = pos12 * self.local_n2;

        if -self.local_n1.dot(&local_n2) < angle_dot_threshold {
            return false;
        }

        for pt in &mut self.points {
            let local_p2 = pos12 * pt.local_p2;
            let dpt = local_p2 - pt.local_p1;
            let dist = dpt.dot(&self.local_n1);

            if dist * pt.dist < T::zero() {
                // We switched between penetrating/non-penetrating.
                // The may result in other contacts to appear.
                return false;
            }
            let new_local_p1 = local_p2 - self.local_n1 * dist;

            if na::distance_squared(&pt.local_p1, &new_local_p1) > dist_sq_threshold {
                return false;
            }

            pt.dist = dist;
            pt.local_p1 = new_local_p1;
        }

        true
    }

    /// Copy data associated to contacts from `old_contacts` to the new contacts in `self`
    /// based on matching their feature-ids.
    pub fn match_contacts(&mut self, old_contacts: &[TrackedContact<T, ContactData>]) {
        for contact in &mut self.points {
            for old_contact in old_contacts {
                if contact.fid1 == old_contact.fid1 && contact.fid2 == old_contact.fid2 {
                    // Transfer the tracked data.
                    contact.data = old_contact.data;
                }
            }
        }
    }

    /// Copy data associated to contacts from `old_contacts` to the new contacts in `self`
    /// based on matching the contact positions.
    pub fn match_contacts_using_positions(
        &mut self,
        old_contacts: &[TrackedContact<T, ContactData>],
        dist_threshold: T,
    ) {
        let sq_threshold = dist_threshold * dist_threshold;
        for contact in &mut self.points {
            for old_contact in old_contacts {
                if na::distance_squared(&contact.local_p1, &old_contact.local_p1) < sq_threshold
                    && na::distance_squared(&contact.local_p2, &old_contact.local_p2) < sq_threshold
                {
                    // Transfer the tracked data.
                    contact.data = old_contact.data;
                }
            }
        }
    }

    /// Removes all the contacts from `self`.
    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// Returns the contact with the smallest distance (i.e. the largest penetration depth).
    pub fn find_deepest_contact(&self) -> Option<&TrackedContact<T, ContactData>> {
        let mut deepest = self.points.get(0)?;

        for pt in &self.points {
            if pt.dist < deepest.dist {
                deepest = pt;
            }
        }

        Some(deepest)
    }
}
