use ad_trait::AD;
use crate::math::{Point, Vector, DIM};
use crate::query::{PointProjection, PointQuery, PointQueryWithLocation};
use crate::shape::{FeatureId, Triangle, TrianglePointLocation};

#[inline]
fn compute_result<T: AD>(pt: &Point<T>, proj: Point<T>) -> PointProjection<T> {
    #[cfg(feature = "dim2")]
    {
        PointProjection::new(*pt == proj, proj)
    }

    #[cfg(feature = "dim3")]
    {
        // TODO: is this acceptable to assume the point is inside of the
        // triangle if it is close enough?
        PointProjection::new(relative_eq!(proj, *pt), proj)
    }
}

impl<T: AD> PointQuery<T> for Triangle<T> {
    #[inline]
    fn project_local_point(&self, pt: &Point<T>, solid: bool) -> PointProjection<T> {
        self.project_local_point_and_get_location(pt, solid).0
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        pt: &Point<T>,
    ) -> (PointProjection<T>, FeatureId) {
        let (proj, loc) = if DIM == 2 {
            self.project_local_point_and_get_location(pt, false)
        } else {
            self.project_local_point_and_get_location(pt, true)
        };

        let feature = match loc {
            TrianglePointLocation::OnVertex(i) => FeatureId::Vertex(i),
            #[cfg(feature = "dim3")]
            TrianglePointLocation::OnEdge(i, _) => FeatureId::Edge(i),
            #[cfg(feature = "dim2")]
            TrianglePointLocation::OnEdge(i, _) => FeatureId::Face(i),
            TrianglePointLocation::OnFace(i, _) => FeatureId::Face(i),
            TrianglePointLocation::OnSolid => FeatureId::Face(0),
        };

        (proj, feature)
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}

impl<T: AD> PointQueryWithLocation<T> for Triangle<T> {
    type Location = TrianglePointLocation<T>;

    #[inline]
    fn project_local_point_and_get_location(
        &self,
        pt: &Point<T>,
        solid: bool,
    ) -> (PointProjection<T>, Self::Location) {
        let a = self.a;
        let b = self.b;
        let c = self.c;

        let _1 = T::one();

        let ab = b - a;
        let ac = c - a;
        let ap = pt - a;

        let ab_ap = ab.dot(&ap);
        let ac_ap = ac.dot(&ap);

        if ab_ap <= T::zero() && ac_ap <= T::zero() {
            // Voronoï region of `a`.
            return (compute_result(pt, a), TrianglePointLocation::OnVertex(0));
        }

        let bp = pt - b;
        let ab_bp = ab.dot(&bp);
        let ac_bp = ac.dot(&bp);

        if ab_bp >= T::zero() && ac_bp <= ab_bp {
            // Voronoï region of `b`.
            return (compute_result(pt, b), TrianglePointLocation::OnVertex(1));
        }

        let cp = pt - c;
        let ab_cp = ab.dot(&cp);
        let ac_cp = ac.dot(&cp);

        if ac_cp >= T::zero() && ab_cp <= ac_cp {
            // Voronoï region of `c`.
            return (compute_result(pt, c), TrianglePointLocation::OnVertex(2));
        }

        enum ProjectionInfo<A: AD> {
            OnAB,
            OnAC,
            OnBC,
            // The usize indicates if we are on the CW side (0) or CCW side (1) of the face.
            OnFace(usize, A, A, A),
        }

        // Checks on which edge voronoï region the point is.
        // For 2D and 3D, it uses explicit cross/perp products that are
        // more numerically stable.
        fn stable_check_edges_voronoi<A: AD>(
            ab: &Vector<A>,
            ac: &Vector<A>,
            bc: &Vector<A>,
            ap: &Vector<A>,
            bp: &Vector<A>,
            cp: &Vector<A>,
            ab_ap: A,
            ab_bp: A,
            ac_ap: A,
            ac_cp: A,
            ac_bp: A,
            ab_cp: A,
        ) -> ProjectionInfo<A> {
            #[cfg(feature = "dim2")]
            {
                let n = ab.perp(&ac);
                let vc = n * ab.perp(&ap);
                if vc < A::zero() && ab_ap >= A::zero() && ab_bp <= A::zero() {
                    return ProjectionInfo::OnAB;
                }

                let vb = -n * ac.perp(&cp);
                if vb < A::zero() && ac_ap >= A::zero() && ac_cp <= A::zero() {
                    return ProjectionInfo::OnAC;
                }

                let va = n * bc.perp(&bp);
                if va < A::zero() && ac_bp - ab_bp >= A::zero() && ab_cp - ac_cp >= A::zero() {
                    return ProjectionInfo::OnBC;
                }

                return ProjectionInfo::OnFace(0, va, vb, vc);
            }
            #[cfg(feature = "dim3")]
            {
                let n;

                #[cfg(feature = "improved_fixed_point_support")]
                {
                    let scaled_n = ab.cross(&ac);
                    n = scaled_n.try_normalize(A::zero()).unwrap_or(scaled_n);
                }

                #[cfg(not(feature = "improved_fixed_point_support"))]
                {
                    n = ab.cross(&ac);
                }

                let vc = n.dot(&ab.cross(&ap));
                if vc < A::zero() && ab_ap >= A::zero() && ab_bp <= A::zero() {
                    return ProjectionInfo::OnAB;
                }

                let vb = -n.dot(&ac.cross(&cp));
                if vb < A::zero() && ac_ap >= A::zero() && ac_cp <= A::zero() {
                    return ProjectionInfo::OnAC;
                }

                let va = n.dot(&bc.cross(&bp));
                if va < A::zero() && ac_bp - ab_bp >= A::zero() && ab_cp - ac_cp >= A::zero() {
                    return ProjectionInfo::OnBC;
                }

                let clockwise = if n.dot(&ap) >= A::zero() { 0 } else { 1 };

                return ProjectionInfo::OnFace(clockwise, va, vb, vc);
            }
        }

        let bc = c - b;
        match stable_check_edges_voronoi(
            &ab, &ac, &bc, &ap, &bp, &cp, ab_ap, ab_bp, ac_ap, ac_cp, ac_bp, ab_cp,
        ) {
            ProjectionInfo::OnAB => {
                // Voronoï region of `ab`.
                let v = ab_ap / ab.norm_squared();
                let bcoords = [_1 - v, v];

                let res = a + ab * v;
                return (
                    compute_result(pt, res),
                    TrianglePointLocation::OnEdge(0, bcoords),
                );
            }
            ProjectionInfo::OnAC => {
                // Voronoï region of `ac`.
                let w = ac_ap / ac.norm_squared();
                let bcoords = [_1 - w, w];

                let res = a + ac * w;
                return (
                    compute_result(pt, res),
                    TrianglePointLocation::OnEdge(2, bcoords),
                );
            }
            ProjectionInfo::OnBC => {
                // Voronoï region of `bc`.
                let w = bc.dot(&bp) / bc.norm_squared();
                let bcoords = [_1 - w, w];

                let res = b + bc * w;
                return (
                    compute_result(pt, res),
                    TrianglePointLocation::OnEdge(1, bcoords),
                );
            }
            ProjectionInfo::OnFace(face_side, va, vb, vc) => {
                // Voronoï region of the face.
                if DIM != 2 {
                    // NOTE: in some cases, numerical instability
                    // may result in the denominator being zero
                    // when the triangle is nearly degenerate.
                    if va + vb + vc != T::zero() {
                        let denom = _1 / (va + vb + vc);
                        let v = vb * denom;
                        let w = vc * denom;
                        let bcoords = [_1 - v - w, v, w];
                        let res = a + ab * v + ac * w;

                        return (
                            compute_result(pt, res),
                            TrianglePointLocation::OnFace(face_side as u32, bcoords),
                        );
                    }
                }
            }
        }

        // Special treatment if we work in 2d because in this case we really are inside of the
        // object.
        if solid {
            (
                PointProjection::new(true, *pt),
                TrianglePointLocation::OnSolid,
            )
        } else {
            // We have to project on the closest edge.

            // TODO: this might be optimizable.
            // TODO: be careful with numerical errors.
            let v = ab_ap / (ab_ap - ab_bp); // proj on ab = a + ab * v
            let w = ac_ap / (ac_ap - ac_cp); // proj on ac = a + ac * w
            let u = (ac_bp - ab_bp) / (ac_bp - ab_bp + ab_cp - ac_cp); // proj on bc = b + bc * u

            let bc = c - b;
            let d_ab = ap.norm_squared() - (ab.norm_squared() * v * v);
            let d_ac = ap.norm_squared() - (ac.norm_squared() * u * u);
            let d_bc = bp.norm_squared() - (bc.norm_squared() * w * w);

            let proj;
            let loc;

            if d_ab < d_ac {
                if d_ab < d_bc {
                    // ab
                    let bcoords = [_1 - v, v];
                    proj = a + ab * v;
                    loc = TrianglePointLocation::OnEdge(0, bcoords);
                } else {
                    // bc
                    let bcoords = [_1 - u, u];
                    proj = b + bc * u;
                    loc = TrianglePointLocation::OnEdge(1, bcoords);
                }
            } else {
                if d_ac < d_bc {
                    // ac
                    let bcoords = [_1 - w, w];
                    proj = a + ac * w;
                    loc = TrianglePointLocation::OnEdge(2, bcoords);
                } else {
                    // bc
                    let bcoords = [_1 - u, u];
                    proj = b + bc * u;
                    loc = TrianglePointLocation::OnEdge(1, bcoords);
                }
            }

            (PointProjection::new(true, proj), loc)
        }
    }
}
