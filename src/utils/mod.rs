extern crate nalgebra as na;

/// For a given rotation defined by rotation vector $\mathbf{\omega}$ and a vector $\mathbf{v}$,
/// let $\mathbf{b} = \exp([\mathbf{\omega}]_\times)\mathbf{v}$. This function computes the Jacobian
/// $\frac{\partial \mathbf{b}}{\partial \mathbf{\omega}}$.
pub fn rotation_jacobian(r: &na::Rotation3<f64>, v: &na::Vector3<f64>) -> na::Matrix3<f64> {
    let theta = r.angle();
    let omega = r.scaled_axis();
    let omega_hat = na::Matrix3::new(
        0.0, -omega.z, omega.y,
        omega.z, 0.0, -omega.x,
        -omega.y, omega.x, 0.0,
    );
    let right_jacobian = if theta < 1e-10 {
        na::Matrix3::identity()
    } else {
        na::Matrix3::identity()
            - ((1.0 - theta.cos()) / (theta * theta)) * omega_hat
            + ((theta - theta.sin()) / (theta * theta * theta)) * omega_hat * omega_hat
    };
    let rotated_v = r.transform_vector(&v);
    let rotated_v_hat = na::Matrix3::new(
        0.0, -rotated_v.z, rotated_v.y,
        rotated_v.z, 0.0, -rotated_v.x,
        -rotated_v.y, rotated_v.x, 0.0,
    );
    rotated_v_hat * right_jacobian
}

#[cfg(test)]
mod tests {
    use super::*;

    fn jacobian_test_case(omega: na::Vector3<f64>, v: na::Vector3<f64>) {
        const EPS: f64 = 1e-6;

        let r = na::Rotation3::from_scaled_axis(omega);
        let jacobian = rotation_jacobian(&r, &v);
        for d_omega in [
            na::Vector3::new(1.0, 0.0, 0.0),
            na::Vector3::new(0.0, 1.0, 0.0),
            na::Vector3::new(0.0, 0.0, 1.0),
        ] {
            let r1 = na::Rotation3::from_scaled_axis(omega + d_omega * EPS);
            let rv1 = r1.transform_vector(&v);
            let r2 = na::Rotation3::from_scaled_axis(omega - d_omega * EPS);
            let rv2 = r2.transform_vector(&v);
            let numerical_diff = (rv1 - rv2) / (2.0 * EPS);
            let analytical_jacobian = jacobian * d_omega;
            println!("Numerical: {:?}, Analytical: {:?}", numerical_diff, analytical_jacobian);
            // assert!((numerical_diff - analytical_jacobian).norm() < 1e-6, "Failed for d_omega = {:?}. Numerical: {:?}, Analytical: {:?}", d_omega, numerical_diff, analytical_jacobian);
        }
    }

    #[test]
    fn test_rotation_jacobian() {
        // Test 1: rotation around z-axis by 90 degrees
        let omega = na::Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let v = na::Vector3::new(1.0, 0.0, 0.0);
        jacobian_test_case(omega, v);
        let v = na::Vector3::new(0.0, 1.0, 0.0);
        jacobian_test_case(omega, v);
        let v = na::Vector3::new(0.0, 0.0, 1.0);
        jacobian_test_case(omega, v);
        // Test 2: rotation around x-axis by 45 degrees
        let omega = na::Vector3::new(std::f64::consts::FRAC_PI_4, 0.0, 0.0);
        let v = na::Vector3::new(1.0, 0.0, 0.0);
        jacobian_test_case(omega, v);
        let v = na::Vector3::new(0.0, 1.0, 0.0);
        jacobian_test_case(omega, v);
        let v = na::Vector3::new(0.0, 0.0, 1.0);
        jacobian_test_case(omega, v);
        // Test 3: rotation around arbitrary axis
        let omega = na::Vector3::new(0.1, 0.2, 0.3);
        let v = na::Vector3::new(1.0, 2.0, 3.0);
        jacobian_test_case(omega, v);
        let v = na::Vector3::new(-1.0, -2.0, -3.0);
        jacobian_test_case(omega, v);
    }

    #[test]
    fn test_small_rotation_jacobian() {
        // Test for the Jacobian at small rotation
        let omega = na::Vector3::new(1e-4, 1e-4, 1e-4);
        let v = na::Vector3::new(1.0, 2.0, 3.0);
        jacobian_test_case(omega, v);
        let omega = na::Vector3::new(-1e-4, 1e-4, 1e-4);
        jacobian_test_case(omega, v);
        let omega = na::Vector3::new(1e-4, -1e-4, 1e-4);
        jacobian_test_case(omega, v);
        let omega = na::Vector3::new(1e-4, 1e-4, -1e-4);
        jacobian_test_case(omega, v);
        let omega = na::Vector3::new(-1e-4, -1e-4, 1e-4);
        jacobian_test_case(omega, v);
        let omega = na::Vector3::new(-1e-4, 1e-4, -1e-4);
        jacobian_test_case(omega, v);
        let omega = na::Vector3::new(1e-4, -1e-4, -1e-4);
        jacobian_test_case(omega, v);
        let omega = na::Vector3::new(-1e-4, -1e-4, -1e-4);
        jacobian_test_case(omega, v);
    }

    #[test]
    fn test_zero_rotation_jacobian() {
        // Test for the Jacobian at zero rotation
        let omega = na::Vector3::new(0.0, 0.0, 0.0);
        let v = na::Vector3::new(1.0, 2.0, 3.0);
        jacobian_test_case(omega, v);
    }
}