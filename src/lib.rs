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
where 
    T: Float
{
    let one = T::one();
    let zero = T::zero();
    (one, [zero, zero, zero])
}

/// Quaternion addition
#[inline(always)]
pub fn add<T>(
    a: Quaternion<T>,
    b: Quaternion<T>
) -> Quaternion<T>
where 
    T: Float
{
    let mut c: Quaternion<T> = id();
    c.0 = a.0 + b.0;
    c.1[0] = a.1[0] + b.1[0];
    c.1[1] = a.1[1] + b.1[1];
    c.1[2] = a.1[2] + b.1[2];
    c
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
}   
        
    
    

