#![deny(missing_docs)]

//! A type agnostic quaternion math library
// Reference:
// https://github.com/PistonDevelopers/quaternion/blob/master/src/lib.rs
// Note: this implementation tries to avoid the use of the vecmath library.

extern crate vecmath;

use vecmath::traits::Float;
use std::fmt::Debug;


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
#[inline(always)]
pub fn rotate_vector<T>(q: Quaternion<T>, v: [T; 3]) -> [T; 3]
    where T: Float
{
    let zero = T::zero();
    let v_as_q: Quaternion<T> = (zero, v);
    let conj: Quaternion<T> = conj(q);
    mul(mul(q, v_as_q), conj).1
}

/// Constructs a quaternion for a given angle
#[inline(always)]
pub fn axis_angle<T>(v: [T; 3], theta: T) -> Quaternion<T>
    where T: Float + Debug
{
    use vecmath::vec3_scale as scale;

    let two = T::one() + T::one();
    let half_theta = theta / two;
    (half_theta.cos(), scale(v, half_theta.sin()))
}


/// Construct a quaternion representing the rotation from a to b
#[inline(always)]
pub fn rotation_from_to<T>(a: [T; 3], b: [T; 3]) -> Quaternion<T>
    where T: Float + Debug
{
    use std::f64::consts::PI;
    use vecmath::{vec3_cross, vec3_dot, vec3_square_len, vec3_normalized};
        
    let one = T::one();
    let zero = T::zero();
    
    let a = vec3_normalized(a);
    let b = vec3_normalized(b);
    let dot = vec3_dot(a,b);
    
    if dot >= one {
        // a and b are parallel
        return id();
    }
    
    if dot < T::from_f64(-0.999999) {
        let mut axis = vec3_cross([one, zero, zero], a);
        if vec3_square_len(axis) == zero {
            axis = vec3_cross([zero, one, zero], a);
        }
        axis = vec3_normalized(axis);
        axis_angle(axis, T::from_f64(PI))
    } else {
        let q = (
            one + dot,
            vec3_cross(a,b)
        );
        scale(q, one / len(q))
    }
}


/// Tests
#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;
    
    static EPSILON: f32 = 0.00001;

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

    #[test]
    fn test_axis_angle() {
        use vecmath::Vector3;
        use vecmath::vec3_normalized as normalized;
        let axis: Vector3<f32> = [1.0, 1.0, 1.0];
        let q: Quaternion<f32> = axis_angle(
            normalized(axis), 
            PI
        );
        assert!((square_len(q) - 1.0).abs() < EPSILON);
    }   
    
    #[test]
    fn test_rotation_from_to_1() {
        use vecmath::Vector3;

        let a: Vector3<f32> = [1.0, 1.0, 1.0];
        let b: Vector3<f32> = [-1.0, -1.0, -1.0];
        
        let q = rotation_from_to(a, b);
        let a_prime = rotate_vector(q, a);
    
        println!("a_prime = {:?}", a_prime);
        
        assert!((a_prime[0] + 1.0).abs() < EPSILON);
        assert!((a_prime[1] + 1.0).abs() < EPSILON);
        assert!((a_prime[2] + 1.0).abs() < EPSILON);
    }
}
