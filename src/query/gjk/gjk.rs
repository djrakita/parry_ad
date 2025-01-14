//! The Gilbert–Johnson–Keerthi distance algorithm.

use na::{self, ComplexField, Unit};

use crate::query::gjk::{CSOPoint, ConstantOrigin, VoronoiSimplex};
use crate::shape::SupportMap;
// use query::Proximity;
use crate::math::{Isometry, Point, Vector, DIM};
use crate::query::{self, Ray};
use ad_trait::AD;

use num::{Bounded};

/// Results of the GJK algorithm.
#[derive(Clone, Debug, PartialEq)]
pub enum GJKResult<T: AD> {
    /// Result of the GJK algorithm when the origin is inside of the polytope.
    Intersection,
    /// Result of the GJK algorithm when a projection of the origin on the polytope is found.
    ///
    /// Both points and vector are expressed in the local-space of the first geometry involved
    /// in the GJK execution.
    ClosestPoints(Point<T>, Point<T>, Unit<Vector<T>>),
    /// Result of the GJK algorithm when the origin is too close to the polytope but not inside of it.
    ///
    /// The returned vector is expressed in the local-space of the first geometry involved in the
    /// GJK execution.
    Proximity(Unit<Vector<T>>),
    /// Result of the GJK algorithm when the origin is too far away from the polytope.
    ///
    /// The returned vector is expressed in the local-space of the first geomety involved in the
    /// GJK execution.
    NoIntersection(Unit<Vector<T>>),
}

/// The absolute tolerence used by the GJK algorithm.
pub fn eps_tol<T: AD>() -> T {
    let _eps = T::constant(crate::math::DEFAULT_EPSILON);
    _eps * T::constant(10.0)
}

/// Projects the origin on the boundary of the given shape.
///
/// The origin is assumed to be outside of the shape. If it is inside,
/// use the EPA algorithm instead.
/// Return `None` if the origin is not inside of the shape or if
/// the EPA algorithm failed to compute the projection.
///
/// Return the projected point in the local-space of `g`.
pub fn project_origin<G: ?Sized, T: AD>(
    m: &Isometry<T>,
    g: &G,
    simplex: &mut VoronoiSimplex<T>,
) -> Option<Point<T>>
where
    G: SupportMap<T>,
{
    match closest_points(
        &m.inverse(),
        g,
        &ConstantOrigin,
        T::constant(f64::max_value()),
        true,
        simplex,
    ) {
        GJKResult::Intersection => None,
        GJKResult::ClosestPoints(p, _, _) => Some(p),
        _ => unreachable!(),
    }
}

/*
 * Separating Axis GJK
 */
/// Projects the origin on a shape using the Separating Axis GJK algorithm.
/// The algorithm will stop as soon as the polytope can be proven to be at least `max_dist` away
/// from the origin.
///
/// # Arguments:
/// * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///             with at least one point on the shape boundary.
/// * exact_dist - if `false`, the gjk will stop as soon as it can prove that the origin is at
/// a distance smaller than `max_dist` but not inside of `shape`. In that case, it returns a
/// `GJKResult::Proximity(sep_axis)` where `sep_axis` is a separating axis. If `false` the gjk will
/// compute the exact distance and return `GJKResult::Projection(point)` if the origin is closer
/// than `max_dist` but not inside `shape`.
pub fn closest_points<G1: ?Sized, G2: ?Sized, T: AD>(
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &G2,
    max_dist: T,
    exact_dist: bool,
    simplex: &mut VoronoiSimplex<T>,
) -> GJKResult<T>
where
    G1: SupportMap<T>,
    G2: SupportMap<T>,
{
    let _eps = crate::math::DEFAULT_EPSILON;
    let _eps_tol: T = eps_tol();
    let _eps_rel: T = ComplexField::sqrt(_eps_tol);

    // FIXME: reset the simplex if it is empty?
    let mut proj = simplex.project_origin_and_reduce();

    let mut old_dir;

    if let Some(proj_dir) = Unit::try_new(proj.coords, T::zero()) {
        old_dir = -proj_dir;
    } else {
        return GJKResult::Intersection;
    }

    let mut max_bound = T::constant(f64::max_value());
    let mut dir;
    let mut niter = 0;

    loop {
        let old_max_bound = max_bound;

        if let Some((new_dir, dist)) = Unit::try_new_and_get(-proj.coords, _eps_tol) {
            dir = new_dir;
            max_bound = dist;
        } else {
            // The origin is on the simplex.
            return GJKResult::Intersection;
        }

        if max_bound >= old_max_bound {
            if exact_dist {
                let (p1, p2) = result(simplex, true);
                return GJKResult::ClosestPoints(p1, p2, old_dir); // upper bounds inconsistencies
            } else {
                return GJKResult::Proximity(old_dir);
            }
        }

        let cso_point = CSOPoint::from_shapes(pos12, g1, g2, &dir);
        let min_bound = -dir.dot(&cso_point.point.coords);

        assert!(min_bound == min_bound);

        if min_bound > max_dist {
            return GJKResult::NoIntersection(dir);
        } else if !exact_dist && min_bound > T::zero() && max_bound <= max_dist {
            return GJKResult::Proximity(old_dir);
        } else if max_bound - min_bound <= _eps_rel * max_bound {
            if exact_dist {
                let (p1, p2) = result(simplex, false);
                return GJKResult::ClosestPoints(p1, p2, dir); // the distance found has a good enough precision
            } else {
                return GJKResult::Proximity(dir);
            }
        }

        if !simplex.add_point(cso_point) {
            if exact_dist {
                let (p1, p2) = result(simplex, false);
                return GJKResult::ClosestPoints(p1, p2, dir);
            } else {
                return GJKResult::Proximity(dir);
            }
        }

        old_dir = dir;
        proj = simplex.project_origin_and_reduce();

        if simplex.dimension() == DIM {
            if min_bound >= _eps_tol {
                if exact_dist {
                    let (p1, p2) = result(simplex, true);
                    return GJKResult::ClosestPoints(p1, p2, old_dir);
                } else {
                    // NOTE: previous implementation used old_proj here.
                    return GJKResult::Proximity(old_dir);
                }
            } else {
                return GJKResult::Intersection; // Point inside of the cso.
            }
        }
        niter += 1;
        if niter == 10000 {
            return GJKResult::NoIntersection(Vector::x_axis());
        }
    }
}

/// Casts a ray on a support map using the GJK algorithm.
pub fn cast_local_ray<G: ?Sized, T: AD>(
    shape: &G,
    simplex: &mut VoronoiSimplex<T>,
    ray: &Ray<T>,
    max_toi: T,
) -> Option<(T, Vector<T>)>
where
    G: SupportMap<T>,
{
    let g2 = ConstantOrigin;
    minkowski_ray_cast(&Isometry::identity(), shape, &g2, ray, max_toi, simplex)
}

/// Compute the normal and the distance that can travel `g1` along the direction
/// `dir` so that `g1` and `g2` just touch.
///
/// The `dir` vector must be expressed in the local-space of the first shape.
pub fn directional_distance<G1: ?Sized, G2: ?Sized, T: AD>(
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &G2,
    dir: &Vector<T>,
    simplex: &mut VoronoiSimplex<T>,
) -> Option<(T, Vector<T>, Point<T>, Point<T>)>
where
    G1: SupportMap<T>,
    G2: SupportMap<T>,
{
    let ray = Ray::new(Point::origin(), *dir);
    minkowski_ray_cast(pos12, g1, g2, &ray, T::constant(f64::max_value()), simplex).map(|(toi, normal)| {
        let witnesses = if !toi.is_zero() {
            result(simplex, simplex.dimension() == DIM)
        } else {
            // If there is penetration, the witness points
            // are undefined.
            (Point::origin(), Point::origin())
        };

        (toi, normal, witnesses.0, witnesses.1)
    })
}

// Ray-cast on the Minkowski Difference `g1 - pos12 * g2`.
fn minkowski_ray_cast<G1: ?Sized, G2: ?Sized, T: AD>(
    pos12: &Isometry<T>,
    g1: &G1,
    g2: &G2,
    ray: &Ray<T>,
    max_toi: T,
    simplex: &mut VoronoiSimplex<T>,
) -> Option<(T, Vector<T>)>
where
    G1: SupportMap<T>,
    G2: SupportMap<T>,
{
    let _eps = crate::math::DEFAULT_EPSILON;
    let _eps_tol: T = eps_tol();
    let _eps_rel: T = ComplexField::sqrt(_eps_tol);

    let ray_length = ray.dir.norm();

    if relative_eq!(ray_length, T::zero()) {
        return None;
    }

    let mut ltoi = T::zero();
    let mut curr_ray = Ray::new(ray.origin, ray.dir / ray_length);
    let dir = -curr_ray.dir;
    let mut ldir = dir;

    // Initialize the simplex.
    let support_point = CSOPoint::from_shapes(pos12, g1, g2, &dir);
    simplex.reset(support_point.translate(&-curr_ray.origin.coords));

    // FIXME: reset the simplex if it is empty?
    let mut proj = simplex.project_origin_and_reduce();
    let mut max_bound = T::constant(f64::max_value());
    let mut dir;
    let mut niter = 0;
    let mut last_chance = false;

    loop {
        let old_max_bound = max_bound;

        if let Some((new_dir, dist)) = Unit::try_new_and_get(-proj.coords, _eps_tol) {
            dir = new_dir;
            max_bound = dist;
        } else {
            return Some((ltoi / ray_length, ldir));
        }

        let support_point = if max_bound >= old_max_bound {
            // Upper bounds inconsistencies. Consider the projection as a valid support point.
            last_chance = true;
            CSOPoint::single_point(proj + curr_ray.origin.coords)
        } else {
            CSOPoint::from_shapes(pos12, g1, g2, &dir)
        };

        if last_chance && ltoi > T::zero() {
            // last_chance && ltoi > T::zero() && (support_point.point - curr_ray.origin).dot(&ldir) >= T::zero() {
            return Some((ltoi / ray_length, ldir));
        }

        // Clip the ray on the support halfspace (None <=> t < 0)
        // The configurations are:
        //   dir.dot(curr_ray.dir)  |   t   |               Action
        // −−−−−−−−−−−−−−−−−−−−-----+−−−−−−−+−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
        //          < 0             |  < 0  | Continue.
        //          < 0             |  > 0  | New lower bound, move the origin.
        //          > 0             |  < 0  | Miss. No intersection.
        //          > 0             |  > 0  | New higher bound.
        match query::details::ray_toi_with_halfspace(&support_point.point, &dir, &curr_ray) {
            Some(t) => {
                if dir.dot(&curr_ray.dir) < T::zero() && t > T::zero() {
                    // new lower bound
                    ldir = *dir;
                    ltoi += t;

                    // NOTE: we divide by ray_length instead of doing max_toi * ray_length
                    // because the multiplication may cause an overflow if max_toi is set
                    // to f64::max_value() by users that want to have an infinite ray.
                    if ltoi / ray_length > max_toi {
                        return None;
                    }

                    let shift = curr_ray.dir * t;
                    curr_ray.origin += shift;
                    max_bound = T::constant(f64::max_value());
                    simplex.modify_pnts(&|pt| pt.translate_mut(&-shift));
                    last_chance = false;
                }
            }
            None => {
                if dir.dot(&curr_ray.dir) > _eps_tol {
                    // miss
                    return None;
                }
            }
        }

        if last_chance {
            return None;
        }

        let min_bound = -dir.dot(&(support_point.point.coords - curr_ray.origin.coords));

        assert!(min_bound == min_bound);

        if max_bound - min_bound <= _eps_rel * max_bound {
            // This is needed when using fixed-points to avoid missing
            // some castes.
            // FIXME: I feel like we should always return `Some` in
            // this case, even with floating-point numbers. Though it
            // has not been sufficinetly tested with floats yet to be sure.
            if cfg!(feature = "improved_fixed_point_support") {
                return Some((ltoi / ray_length, ldir));
            } else {
                return None;
            }
        }

        let _ = simplex.add_point(support_point.translate(&-curr_ray.origin.coords));
        proj = simplex.project_origin_and_reduce();

        if simplex.dimension() == DIM {
            if min_bound >= _eps_tol {
                return None;
            } else {
                return Some((ltoi / ray_length, ldir)); // Point inside of the cso.
            }
        }

        niter += 1;
        if niter == 10000 {
            return None;
        }
    }
}

fn result<T: AD>(simplex: &VoronoiSimplex<T>, prev: bool) -> (Point<T>, Point<T>) {
    let mut res = (Point::origin(), Point::origin());
    if prev {
        for i in 0..simplex.prev_dimension() + 1 {
            let coord = simplex.prev_proj_coord(i);
            let point = simplex.prev_point(i);
            res.0 += point.orig1.coords * coord;
            res.1 += point.orig2.coords * coord;
        }

        res
    } else {
        for i in 0..simplex.dimension() + 1 {
            let coord = simplex.proj_coord(i);
            let point = simplex.point(i);
            res.0 += point.orig1.coords * coord;
            res.1 += point.orig2.coords * coord;
        }

        res
    }
}
