use glam::Vec2;
use std::f32::consts::PI;

const EPS: f32 = 1e-4;

/// For given unit circle chord returns segment area and barycenter offset.
///
/// Chord is defined via distance from circle center.
pub fn circle_segment(radius: f32, dist: f32) -> (f32, f32) {
    let cosine = (dist / radius).clamp(-1.0, 1.0);
    let sine = (1.0 - cosine.powi(2)).sqrt();
    let (area, barycenter) = if cosine.abs() < 1.0 - EPS {
        let area = cosine.acos() - cosine * sine;
        (area, (2.0 / 3.0) * sine.powi(3) / area)
    } else {
        // Approximate circle by parabola
        let y = 1.0 - cosine.abs();
        let a = (4.0 / 3.0) * (2.0 * y).sqrt() * y;
        let b = 1.0 - (3.0 / 10.0) * y;
        if cosine > 0.0 {
            (a, b)
        } else {
            (PI - a, -b * a / (PI - a))
        }
    };
    (area * radius.powi(2), barycenter * radius)
}

pub fn intersect_circle_and_plane(
    center: Vec2,
    radius: f32,
    offset: f32,
    normal: Vec2,
) -> Option<(f32, Vec2)> {
    let dist = center.dot(normal) - offset;
    if dist < radius {
        if dist > -radius {
            let (area, barycenter) = circle_segment(radius, dist);
            Some((area, center - normal * barycenter))
        } else {
            Some((PI * radius.powi(2), center))
        }
    } else {
        None
    }
}

/// Intersect two shapes at given positions.
/// Returns area and barycenter of intersection.
pub fn intersect_circles(ac: Vec2, ar: f32, bc: Vec2, br: f32) -> Option<(f32, Vec2)> {
    let vec = bc - ac;
    let dist = vec.length();
    if dist < ar + br {
        if dist > (ar - br).abs() {
            let dir = vec / dist;
            let ax = 0.5 * (dist + (ar.powi(2) - br.powi(2)) / dist);
            let bx = dist - ax;
            let (aa, ab) = circle_segment(ar, ax);
            let (ba, bb) = circle_segment(br, bx);
            let area = aa + ba;
            Some((area, ((ac + dir * ab) * aa + (bc - dir * bb) * ba) / area))
        } else {
            let (minr, minc) = if ar < br { (ar, ac) } else { (br, bc) };
            Some((PI * minr.powi(2), minc))
        }
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    const R: f32 = 1.234;

    #[test]
    fn empty_segment() {
        assert_eq!(circle_segment(R, R), (0.0, R));
    }

    #[test]
    fn full_segment() {
        assert_eq!(circle_segment(R, -R), (PI * R.powi(2), 0.0));
    }

    #[test]
    fn half_segment() {
        assert_eq!(circle_segment(R, 0.0).0, PI * R.powi(2) / 2.0);
    }

    #[test]
    fn numerical_segment() {
        let f = |x: f64| 2.0 * (1.0 - (1.0 - x).powi(2)).sqrt();

        let mut x: f64 = 0.0;
        let dx: f64 = 1e-6;

        let (mut area, mut moment) = (0.0, 0.0);

        let check_step = 1e-2;
        let mut last_check = 0.0;
        while x < 2.0 {
            let d_area = 0.5 * (f(x) + f(x + dx)) * dx;
            area += d_area;
            moment += d_area * (x + 0.5 * dx);
            if x >= last_check + check_step {
                last_check = x;
                let (ref_area, ref_barycenter) = circle_segment(1.0, (1.0 - x) as f32);
                assert_abs_diff_eq!(ref_area, area as f32, epsilon = 1e-4);
                assert_abs_diff_eq!(ref_barycenter, 1.0 - (moment / area) as f32, epsilon = 1e-4);
            }
            x += dx;
        }
    }
}
