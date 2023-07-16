use crate::math::{Isometry, Vector};
#[cfg(feature = "std")]
use crate::query::{contact_manifolds::ContactManifoldsWorkspace, ContactManifold};
use crate::query::{ClosestPoints, Contact, NonlinearRigidMotion, Unsupported, TOI};
use crate::shape::Shape;
use ad_trait::AD;

#[cfg(feature = "std")]
/// A query dispatcher for queries relying on spatial coherence, including contact-manifold computation.
pub trait PersistentQueryDispatcher<T: AD, ManifoldData = (), ContactData = ()>: QueryDispatcher {
    /// Compute all the contacts between two shapes.
    ///
    /// The output is written into `manifolds` and `context`. Both can persist
    /// between multiple calls to `contacts` by re-using the result of the previous
    /// call to `contacts`. This persistence can significantly improve collision
    /// detection performances by allowing the underlying algorithms to exploit
    /// spatial and temporal coherence.
    fn contact_manifolds(
        &self,
        pos12: &Isometry<T>,
        g1: &dyn Shape<T>,
        g2: &dyn Shape<T>,
        prediction: T,
        manifolds: &mut Vec<ContactManifold<ManifoldData, ContactData, T>>,
        workspace: &mut Option<ContactManifoldsWorkspace<T>>,
    ) -> Result<(), Unsupported>;

    /// Computes the contact-manifold between two convex shapes.
    fn contact_manifold_convex_convex(
        &self,
        pos12: &Isometry<T>,
        g1: &dyn Shape<T>,
        g2: &dyn Shape<T>,
        prediction: T,
        manifold: &mut ContactManifold<ManifoldData, ContactData, T>,
    ) -> Result<(), Unsupported>;
}

/// Dispatcher for pairwise queries.
///
/// Custom implementations allow crates that support an abstract `QueryDispatcher` to handle custom
/// shapes.
///
/// The `pos12` argument to most queries is the transform from the local space of `g2` to that of
/// `g1`.
pub trait QueryDispatcher<T: AD>: Send + Sync {
    /// Tests whether two shapes are intersecting.
    fn intersection_test(
        &self,
        pos12: &Isometry<T>,
        g1: &dyn Shape<T>,
        g2: &dyn Shape<T>,
    ) -> Result<bool, Unsupported>;

    /// Computes the minimum distance separating two shapes.
    ///
    /// Returns `0.0` if the objects are touching or penetrating.
    fn distance(
        &self,
        pos12: &Isometry<T>,
        g1: &dyn Shape<T>,
        g2: &dyn Shape<T>,
    ) -> Result<T, Unsupported>;

    /// Computes one pair of contact points point between two shapes.
    ///
    /// Returns `None` if the objects are separated by a distance greater than `prediction`.
    fn contact(
        &self,
        pos12: &Isometry<T>,
        g1: &dyn Shape<T>,
        g2: &dyn Shape<T>,
        prediction: T,
    ) -> Result<Option<Contact<T>>, Unsupported>;

    /// Computes the pair of closest points between two shapes.
    ///
    /// Returns `ClosestPoints::Disjoint` if the objects are separated by a distance greater than `max_dist`.
    fn closest_points(
        &self,
        pos12: &Isometry<T>,
        g1: &dyn Shape<T>,
        g2: &dyn Shape<T>,
        max_dist: T,
    ) -> Result<ClosestPoints<T>, Unsupported>;

    /// Computes the smallest time when two shapes under translational movement are separated by a
    /// distance smaller or equal to `distance`.
    ///
    /// Returns `0.0` if the objects are touching or penetrating.
    ///
    /// # Parameters
    /// - `pos12`: the position of the second shape relative to the first shape.
    /// - `local_vel12`: the relative velocity between the two shapes, expressed in the local-space
    ///                  of the first shape. In other world: `pos1.inverse() * (vel2 - vel1)`.
    /// - `g1`: the first shape involved in the TOI computation.
    /// - `g2`: the second shape involved in the TOI computation.
    /// - `max_toi`: the maximum allowed TOI. This method returns `None` if the time-of-impact
    ///              detected is theater than this value.
    fn time_of_impact(
        &self,
        pos12: &Isometry<T>,
        local_vel12: &Vector<T>,
        g1: &dyn Shape<T>,
        g2: &dyn Shape<T>,
        max_toi: T,
        stop_at_penetration: bool,
    ) -> Result<Option<TOI<T>>, Unsupported>;

    /// Construct a `QueryDispatcher` that falls back on `other` for cases not handled by `self`
    fn chain<U: QueryDispatcher<T>>(self, other: U) -> QueryDispatcherChain<Self, U>
    where
        Self: Sized,
    {
        QueryDispatcherChain(self, other)
    }

    /// Computes the smallest time of impact of two shapes under translational and rotational movement.
    ///
    /// # Parameters
    /// * `motion1` - The motion of the first shape.
    /// * `g1` - The first shape involved in the query.
    /// * `motion2` - The motion of the second shape.
    /// * `g2` - The second shape involved in the query.
    /// * `start_time` - The starting time of the interval where the motion takes place.
    /// * `end_time` - The end time of the interval where the motion takes place.
    /// * `stop_at_penetration` - If the casted shape starts in a penetration state with any
    ///    collider, two results are possible. If `stop_at_penetration` is `true` then, the
    ///    result will have a `toi` equal to `start_time`. If `stop_at_penetration` is `false`
    ///    then the nonlinear shape-casting will see if further motion wrt. the penetration normal
    ///    would result in tunnelling. If it does not (i.e. we have a separating velocity along
    ///    that normal) then the nonlinear shape-casting will attempt to find another impact,
    ///    at a time `> start_time` that could result in tunnelling.
    fn nonlinear_time_of_impact(
        &self,
        motion1: &NonlinearRigidMotion<T>,
        g1: &dyn Shape<T>,
        motion2: &NonlinearRigidMotion<T>,
        g2: &dyn Shape<T>,
        start_time: T,
        end_time: T,
        stop_at_penetration: bool,
    ) -> Result<Option<TOI<T>>, Unsupported>;
}

/// The composition of two dispatchers
pub struct QueryDispatcherChain<T, U>(T, U);

macro_rules! chain_method {
    ($name:ident ( $( $arg:ident : $ty:ty,)*) -> $result:ty) => {
        fn $name(&self, $($arg : $ty,)*
        ) -> Result<$result, Unsupported> {
            (self.0).$name($($arg,)*)
                .or_else(|Unsupported| (self.1).$name($($arg,)*))
        }
    }
}

impl<T, U, A: AD> QueryDispatcher<A> for QueryDispatcherChain<T, U>
where
    T: QueryDispatcher<A>,
    U: QueryDispatcher<A>,
{
    chain_method!(intersection_test(
        pos12: &Isometry<A>,
        g1: &dyn Shape<A>,
        g2: &dyn Shape<A>,
    ) -> bool);

    chain_method!(distance(pos12: &Isometry<A>, g1: &dyn Shape<A>, g2: &dyn Shape<A>,) -> A);

    chain_method!(contact(
        pos12: &Isometry<A>,
        g1: &dyn Shape<A>,
        g2: &dyn Shape<A>,
        prediction: A,
    ) -> Option<Contact>);

    chain_method!(closest_points(
        pos12: &Isometry<A>,
        g1: &dyn Shape<A>,
        g2: &dyn Shape<A>,
        max_dist: A,
    ) -> ClosestPoints);

    chain_method!(time_of_impact(
        pos12: &Isometry<A>,
        vel12: &Vector<A>,
        g1: &dyn Shape<A>,
        g2: &dyn Shape<A>,
        max_toi: A,
        stop_at_penetration: bool,
    ) -> Option<TOI<A>>);

    chain_method!(nonlinear_time_of_impact(
        motion1: &NonlinearRigidMotion<A>,
        g1: &dyn Shape<A>,
        motion2: &NonlinearRigidMotion<A>,
        g2: &dyn Shape<A>,
        start_time: A,
        end_time: A,
        stop_at_penetration: bool,
    ) -> Option<TOI<A>>);
}

#[cfg(feature = "std")]
impl<ManifoldData, ContactData, T, U, A: AD> PersistentQueryDispatcher<ManifoldData, ContactData, A>
    for QueryDispatcherChain<T, U>
where
    T: PersistentQueryDispatcher<A, ManifoldData, ContactData>,
    U: PersistentQueryDispatcher<A, ManifoldData, ContactData>,
{
    chain_method!(contact_manifolds(
        pos12: &Isometry<A>,
        g1: &dyn Shape<A>,
        g2: &dyn Shape<A>,
        prediction: A,
        manifolds: &mut Vec<ContactManifold<ManifoldData, ContactData, A>>,
        workspace: &mut Option<ContactManifoldsWorkspace<A>>,
    ) -> ());

    chain_method!(contact_manifold_convex_convex(
        pos12: &Isometry<A>,
        g1: &dyn Shape<A>,
        g2: &dyn Shape<A>,
        prediction: A,
        manifold: &mut ContactManifold<ManifoldData, ContactData, A>,
    ) -> ());
}
