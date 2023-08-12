use std::time::Instant;
use ad_trait::AD;
use ad_trait::forward_ad::adf::{adf_f32x16, adf_f32x2, adf_f32x4, adf_f32x8};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::reverse_ad::adr::{adr, GlobalComputationGraph};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use simba::simd::{f32x16, f32x2, f32x4, f32x8};
use parry3d_f64::query::{contact, distance, intersection_test};
use parry3d_f64::shape::Ball;

fn main() {
    let s1 = Ball::new(adfn::new(1.0, [1.0]));
    let s2 = Ball::new(adfn::new(1.0, [0.0]));

    let iso1 = Isometry3::identity();
    let iso2 = Isometry3::from_parts(Translation3::new(adfn::<1>::constant(200.0), adfn::<1>::constant(1.0), adfn::<1>::constant(0.0)), UnitQuaternion::from_euler_angles(adfn::<1>::constant(2.0), adfn::<1>::constant(1.0), adfn::<1>::constant(1.0)));

    let start = Instant::now();
    for _ in 0..1000 {
        intersection_test(&iso1, &s1, &iso2, &s2).expect("error");
    }
    println!("adfn intersection test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        distance(&iso1, &s1, &iso2, &s2).expect("error");
    }
    println!("adfn distance test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        contact(&iso1, &s1, &iso2, &s2, adfn::<1>::constant(100000.0));
    }
    println!("adfn contact test: {:?}", start.elapsed());

    println!("/////");

    let s1 = Ball::new(1.0);
    let s2 = Ball::new(1.0);

    let iso1 = Isometry3::identity();
    let iso2 = Isometry3::from_parts(Translation3::new(200.0,1.0,0.0), UnitQuaternion::from_euler_angles(2.0, 1.0, 1.0));

    let start = Instant::now();
    for _ in 0..1000 {
        intersection_test(&iso1, &s1, &iso2, &s2);
    }
    println!("f64 intersection test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        distance(&iso1, &s1, &iso2, &s2);
    }
    println!("f64 distance test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        contact(&iso1, &s1, &iso2, &s2, 100000.0);
    }
    println!("f64 contact test: {:?}", start.elapsed());

    println!("/////");

    let s1 = Ball::new(1.0_f32);
    let s2 = Ball::new(1.0_f32);

    let iso1 = Isometry3::identity();
    let iso2 = Isometry3::from_parts(Translation3::new(200.0,1.0,0.0), UnitQuaternion::from_euler_angles(2.0, 1.0, 1.0));

    let start = Instant::now();
    for _ in 0..1000 {
        intersection_test(&iso1, &s1, &iso2, &s2);
    }
    println!("f32 intersection test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        distance(&iso1, &s1, &iso2, &s2);
    }
    println!("f32 distance test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        contact(&iso1, &s1, &iso2, &s2, 100000.0_f32);
    }
    println!("f32 contact test: {:?}", start.elapsed());

    println!("/////");

    let s1 = Ball::new(adf_f32x2::new(1.0, f32x2::new(1.0, 0.0)));
    let s2 = Ball::new(adf_f32x2::new(1.0, f32x2::new(0.0, 0.0)));

    let iso1 = Isometry3::identity();
    let iso2 = Isometry3::from_parts(Translation3::new(adf_f32x2::constant(200.0), adf_f32x2::constant(1.0), adf_f32x2::constant(0.0)), UnitQuaternion::from_euler_angles(adf_f32x2::constant(2.0), adf_f32x2::constant(1.0), adf_f32x2::constant(1.0)));

    let start = Instant::now();
    for _ in 0..1000 {
        intersection_test(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x2 intersection test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        distance(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x2 distance test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        contact(&iso1, &s1, &iso2, &s2, adf_f32x2::constant(100000.0));
    }
    println!("adf_f32x2 contact test: {:?}", start.elapsed());

    println!("/////");

    let s1 = Ball::new(adf_f32x4::new(1.0, f32x4::new(1.0, 0.0, 0.0, 0.5)));
    let s2 = Ball::new(adf_f32x4::new(1.0, f32x4::new(0.0, 0.0, 0.0, 0.0)));

    let iso1 = Isometry3::identity();
    let iso2 = Isometry3::from_parts(Translation3::new(adf_f32x4::constant(200.0), adf_f32x4::constant(1.0), adf_f32x4::constant(0.0)), UnitQuaternion::from_euler_angles(adf_f32x4::constant(2.0), adf_f32x4::constant(1.0), adf_f32x4::constant(1.0)));

    let start = Instant::now();
    for _ in 0..1000 {
        intersection_test(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x4 intersection test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        distance(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x4 distance test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        contact(&iso1, &s1, &iso2, &s2, adf_f32x4::constant(100000.0));
    }
    println!("adf_f32x4 contact test: {:?}", start.elapsed());

    println!("/////");

    let s1 = Ball::new(adf_f32x8::new(1.0, f32x8::new(1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.5)));
    let s2 = Ball::new(adf_f32x8::new(1.0, f32x8::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)));

    let iso1 = Isometry3::identity();
    let iso2 = Isometry3::from_parts(Translation3::new(adf_f32x8::constant(200.0), adf_f32x8::constant(1.0), adf_f32x8::constant(0.0)), UnitQuaternion::from_euler_angles(adf_f32x8::constant(2.0), adf_f32x8::constant(1.0), adf_f32x8::constant(1.0)));

    let start = Instant::now();
    for _ in 0..1000 {
        intersection_test(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x8 intersection test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        distance(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x8 distance test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        contact(&iso1, &s1, &iso2, &s2, adf_f32x8::constant(100000.0));
    }
    println!("adf_f32x8 contact test: {:?}", start.elapsed());

    println!("/////");

    let s1 = Ball::new(adf_f32x16::new(1.0, f32x16::new(1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.5)));
    let s2 = Ball::new(adf_f32x16::new(1.0, f32x16::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.5)));

    let iso1 = Isometry3::identity();
    let iso2 = Isometry3::from_parts(Translation3::new(adf_f32x16::constant(200.0), adf_f32x16::constant(1.0), adf_f32x16::constant(0.0)), UnitQuaternion::from_euler_angles(adf_f32x16::constant(2.0), adf_f32x16::constant(1.0), adf_f32x16::constant(1.0)));

    let start = Instant::now();
    for _ in 0..1000 {
        intersection_test(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x16 intersection test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        distance(&iso1, &s1, &iso2, &s2);
    }
    println!("adf_f32x16 distance test: {:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        contact(&iso1, &s1, &iso2, &s2, adf_f32x16::constant(100000.0));
    }
    println!("adf_f32x16 contact test: {:?}", start.elapsed());

    println!("/////");

    let start = Instant::now();
    for _ in 0..1000 {
        let v = adr::new_variable(1.0, true);
        let s1 = Ball::new(v);
        let s2 = Ball::new(adr::constant(1.0));

        let iso1 = Isometry3::identity();
        let iso2 = Isometry3::from_parts(Translation3::new(adr::constant(200.0), adr::constant(1.0), adr::constant(0.0)), UnitQuaternion::from_euler_angles(adr::constant(2.0), adr::constant(1.0), adr::constant(1.0)));

        intersection_test(&iso1, &s1, &iso2, &s2);
        v.get_backwards_mode_grad();
    }
    println!("adr intersection test: {:?}", start.elapsed());
    println!("{:?}", GlobalComputationGraph::get().num_nodes());

    let start = Instant::now();
    for _ in 0..1000 {
        let v = adr::new_variable(1.0, true);
        let s1 = Ball::new(v);
        let s2 = Ball::new(adr::constant(1.0));

        let iso1 = Isometry3::identity();
        let iso2 = Isometry3::from_parts(Translation3::new(adr::constant(200.0), adr::constant(1.0), adr::constant(0.0)), UnitQuaternion::from_euler_angles(adr::constant(2.0), adr::constant(1.0), adr::constant(1.0)));

        distance(&iso1, &s1, &iso2, &s2);
        v.get_backwards_mode_grad();
    }
    println!("adr distance test: {:?}", start.elapsed());
    println!("{:?}", GlobalComputationGraph::get().num_nodes());

    let start = Instant::now();
    for _ in 0..1000 {
        let v = adr::new_variable(1.0, true);
        let s1 = Ball::new(v);
        let s2 = Ball::new(adr::new_variable(1.0, false));

        let iso1 = Isometry3::identity();
        let iso2 = Isometry3::from_parts(Translation3::new(adr::new_variable(200.0, false), adr::new_variable(1.0, false), adr::constant(0.0)), UnitQuaternion::from_euler_angles(adr::constant(2.0), adr::new_variable(1.0, false), adr::constant(1.0)));

        contact(&iso1, &s1, &iso2, &s2, adr::constant(100000.0));
        v.get_backwards_mode_grad();
    }
    println!("adr contact test: {:?}", start.elapsed());
    println!("{:?}", GlobalComputationGraph::get().num_nodes());
}