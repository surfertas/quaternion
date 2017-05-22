#![deny(missing_docs)]

//! A type agnostic quaternion math library
// Credit:
// https://github.com/PistonDevelopers/quaternion/blob/master/src/lib.rs
// Note: this implementation tries to avoid the use of the vecmath library.

extern crate vecmath;

use vecmath::traits::Float;

/// Quaternion type alias
pub type Quaternion<T> = (T, [T; 3]);


/// Quaternion identity quaternion
#[inline(always)]
pub fn id<T>() -> Quaternion<T>
    where T: Float
{
    let one = T::one();
    let zero = T::zero();
    (one, [zero, zero, zero])
}

/// Adds two quaternions.
#[inline(always)]
pub fn add<T>(a: Quaternion<T>, b: Quaternion<T>) -> Quaternion<T>
    where T: Float
{
    let mut c: Quaternion<T> = id();
    c.0 = a.0 + b.0;
    c.1[0] = a.1[0] + b.1[0];
    c.1[1] = a.1[1] + b.1[1];
    c.1[2] = a.1[2] + b.1[2];
    c
}

/// Scales a quaternion (element-wise) by a scalar
#[inline(always)]
pub fn scale<T>(q: Quaternion<T>, t: T) -> Quaternion<T>
    where T: Float
{
    (q.0 * t, [q.1[0] * t, q.1[1] * t, q.1[2] * t])
}

/// Dot product of two quaternions
pub fn dot<T>(a: Quaternion<T>, b: Quaternion<T>) -> T
    where T: Float
{
    let v_total = a.1[0] * b.1[0] + a.1[1] * b.1[1] + a.1[2] * b.1[2];
    a.0 * b.0 + v_total
}

/// Tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        let q0: Quaternion<f64> = id();
        let q1: Quaternion<f64> = (1.0, [1.0, 1.0, 1.0]);
        assert_eq!(add(q0, q1), (2.0, [1.0, 1.0, 1.0]));
    }

    #[test]
    fn test_scale() {
        let q: Quaternion<f64> = id();
        let t = 5.0;
        assert_eq!(scale(q, t), (5.0, [0.0, 0.0, 0.0]));
    }

    #[test]
    fn test_dot() {
        let q0: Quaternion<f64> = id();
        let q1: Quaternion<f64> = id();
        assert_eq!(dot(q0, q1), 1.0);
    }
}
