#![deny(missing_docs)]

//! A type agnostic quaternion math library
// Reference:
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
#[inline(always)]
pub fn dot<T>(a: Quaternion<T>, b: Quaternion<T>) -> T
    where T: Float
{
    let v_total = a.1[0] * b.1[0] + a.1[1] * b.1[1] + a.1[2] * b.1[2];
    a.0 * b.0 + v_total
}

/// Multiplies two quaternions
#[inline(always)]
pub fn mul<T>(a: Quaternion<T>, b: Quaternion<T>) -> Quaternion<T>
    where T: Float
{
    let mut c: Quaternion<T> = id();
    c.0 = a.0 * b.0 - a.1[0] * b.1[0] - a.1[1] * b.1[1] - a.1[2] * b.1[2];
    c.1[0] = a.0 * b.1[0] + a.1[0] * b.0 + a.1[1] * b.1[2] - a.1[2] * b.1[1];
    c.1[1] = a.0 * b.1[1] - a.1[0] * b.1[2] + a.1[1] * b.0 + a.1[2] * b.1[0];
    c.1[2] = a.0 * b.1[2] + a.1[0] * b.1[1] - a.1[1] * b.1[0] + a.1[2] * b.0;
    c
}

/// Takes the quaternion conjugate
#[inline(always)]
pub fn conj<T>(a: Quaternion<T>) -> Quaternion<T>
    where T: Float
{
    (a.0, [-a.1[0], -a.1[1], -a.1[2]])
}

/// Computes the square length of a quaternion.
#[inline(always)]
pub fn square_len<T>(q: Quaternion<T>) -> T
    where T: Float
{
    q.0 * q.0 + q.1[0] * q.1[0] + q.1[1] * q.1[1] + q.1[2] * q.1[2]
}

/// Computes the length of a quarternion
#[inline(always)]
pub fn len<T>(q: Quaternion<T>) -> T
    where T: Float
{
    square_len(q).sqrt()
}

/// Rotate the given vector using the given quaternion
pub fn rotate_vector<T>(q: Quaternion<T>, v: [T; 3]) -> [T; 3]
    where T: Float
{
    let zero = T::zero();
    let v_as_q: Quaternion<T> = (zero, v);
    let conj: Quaternion<T> = conj(q);
    mul(mul(q, v_as_q), conj).1
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

    #[test]
    fn test_mul() {
        use vecmath::vec3_cross as cross;
        use vecmath::vec3_add as add;
        use vecmath::vec3_dot as dot;
        use vecmath::vec3_scale as scale;

        let q0: Quaternion<f64> = (2.0, [1.0, 1.0, 1.0]);
        let q1: Quaternion<f64> = (3.0, [1.0, 1.0, 1.0]);

        let q: Quaternion<f64> = (q0.0 * q1.0 - dot(q0.1, q1.1),
                                  add(cross(q0.1, q1.1),
                                      add(scale(q1.1, q0.0), scale(q0.1, q1.0))));
        assert_eq!(q, mul(q0, q1));
    }

    #[test]
    fn test_conj() {
        let q: Quaternion<f64> = (2.0, [1.0, 1.0, 1.0]);
        let q_conj: Quaternion<f64> = (2.0, [-1.0, -1.0, -1.0]);

        assert_eq!(q_conj, conj(q));
    }

    #[test]
    fn test_square_len() {
        use vecmath::vec3_square_len;
        let q: Quaternion<f64> = (2.0, [1.0, 1.0, 1.0]);
        assert_eq!(q.0 * q.0 + vec3_square_len(q.1), square_len(q));
    }
}
