//! Nudging: routes found one by one share the grid's lines, so wires
//! through one channel are drawn on top of each other. Afterwards,
//! segments of different routes that lie on one line and overlap are
//! spread apart across it, `spacing` apart and centred on the line.
//!
//! A segment moves as a whole, across its own direction: its two
//! neighbours are perpendicular to it, so they only grow or shrink and
//! the route stays orthogonal. A route's first and last segments are
//! left in place, since they hold its ports. The spread is at most the
//! margin (half of it on each side of the line), so a nudged segment
//! keeps at least half the margin away from the obstacles.
//!
//! A route's first and last segments are its stubs, the margin long
//! where the route turns right out of its port: the line a stub ends
//! on is the channel beside its node, where wires gather. Centred on
//! that line, half of a group would move toward the node and shorten
//! the stubs that end on it, to half the margin at worst. So a group
//! holding a segment next to a port moves away from that port as a
//! whole, until the stub is as long as it was found, or the margin if
//! it was longer. It still stays within half the margin of its line:
//! on the half that is left, its segments come closer than `spacing`
//! where they must, down to half of what they would have. Where even
//! that does not do, with stubs on both sides of the line, the stubs
//! share what is missing.
//!
//! Within a group, segments are ordered so the spreading adds as few
//! crossings as it can. Take vertical segments, ordered left to right:
//! each has a top end and a bottom end, where it turns left or right.
//! A segment turning left at both ends goes left of one turning right
//! at both; between two turning the same way at the top, the one whose
//! top is lower goes on that side (so its turn does not cut the other
//! one), and between two turning the same way at the bottom, the one
//! whose bottom is higher goes on that side. Horizontal segments are
//! ordered the same way with the axes swapped. Ties keep the
//! connectors' order.

use crate::Route;
use crate::geom::{EPS, Point};

/// A segment eligible for nudging.
#[derive(Clone, Copy, Debug)]
struct Seg {
    /// The route and the index of the segment's first point.
    route: usize,
    k: usize,
    horizontal: bool,
    /// Its line: `y` when horizontal, else `x`.
    at: f32,
    lo: f32,
    hi: f32,
    /// Its place across the line, smallest first.
    key: (f32, f32),
}

/// Spreads overlapping collinear segments of `routes` apart. A group
/// spreads over the smallest margin of its routes.
pub(crate) fn nudge(routes: &mut [Route], spacing: f32) {
    if spacing <= 0.0 || !spacing.is_finite() {
        return;
    }
    let mut segs = segments(routes);
    segs.sort_by(|a, b| {
        a.horizontal
            .cmp(&b.horizontal)
            .then(a.at.total_cmp(&b.at))
            .then(a.lo.total_cmp(&b.lo))
            .then(a.route.cmp(&b.route))
            .then(a.k.cmp(&b.k))
    });
    // Shifts, decided on the routes as found, then applied.
    let mut shifts: Vec<(usize, usize, bool, f32)> = Vec::new();
    let mut start = 0;
    while start < segs.len() {
        // A group: one line, and each segment overlapping the span of
        // those before it.
        let first = segs[start];
        let mut end = start + 1;
        let mut hi = first.hi;
        while end < segs.len() {
            let s = segs[end];
            if s.horizontal != first.horizontal || (s.at - first.at).abs() > EPS || s.lo >= hi - EPS
            {
                break;
            }
            hi = hi.max(s.hi);
            end += 1;
        }
        let group = &mut segs[start..end];
        start = end;
        let routes_in = {
            let mut r: Vec<usize> = group.iter().map(|s| s.route).collect();
            r.dedup();
            r.len()
        };
        if group.len() < 2 || routes_in < 2 {
            continue;
        }
        group.sort_by(|a, b| {
            a.key
                .0
                .total_cmp(&b.key.0)
                .then(a.key.1.total_cmp(&b.key.1))
                .then(a.route.cmp(&b.route))
        });
        let n = group.len() as f32;
        let margin = group
            .iter()
            .map(|s| routes[s.route].margin)
            .fold(f32::INFINITY, f32::min);
        let margin = margin.max(0.0);
        // A segment's place in the group, in steps from its middle.
        let place = |m: usize| m as f32 - (n - 1.0) / 2.0;
        // The stubs that end on the group's segments: the place of the
        // segment, and how far it may move toward its port (negative
        // for a port at a smaller coordinate).
        let mut pins: Vec<(f32, f32, bool)> = Vec::new();
        for (m, s) in group.iter().enumerate() {
            let route = &routes[s.route];
            let p = &route.points;
            let ports = [
                (s.k == 1).then_some(0),
                (s.k + 3 == p.len()).then_some(p.len() - 1),
            ];
            for port in ports.into_iter().flatten() {
                let port = if s.horizontal { p[port].y } else { p[port].x };
                let stub = s.at - port;
                let keep = stub.abs().min(route.margin);
                pins.push((place(m), keep.copysign(stub) - stub, stub > 0.0));
            }
        }
        // How far the group may move as a whole with this step: for the
        // stubs, and to stay within half the margin of its line.
        let stubs = |step: f32| {
            let (mut least, mut most) = (f32::NEG_INFINITY, f32::INFINITY);
            for &(place, room, port_before) in &pins {
                if port_before {
                    least = least.max(room - place * step);
                } else {
                    most = most.min(room - place * step);
                }
            }
            (least, most)
        };
        let reach = |step: f32| (margin - (n - 1.0) * step).max(0.0) / 2.0;
        let fits = |step: f32| {
            let (least, most) = stubs(step);
            least.max(-reach(step)) <= most.min(reach(step))
        };
        let full = spacing.min(margin / (n - 1.0));
        let tight = full / 2.0;
        let step = if fits(full) || !fits(tight) {
            full.min(if fits(full) { full } else { tight })
        } else {
            // The widest step that fits: what fits is an interval.
            let (mut fitting, mut wide) = (tight, full);
            for _ in 0..16 {
                let mid = (fitting + wide) / 2.0;
                if fits(mid) {
                    fitting = mid;
                } else {
                    wide = mid;
                }
            }
            fitting
        };
        let (least, most) = stubs(step);
        let moved = if least <= most {
            0f32.clamp(least, most)
        } else {
            (least + most) / 2.0
        };
        let moved = moved.clamp(-reach(step), reach(step));
        for (m, s) in group.iter().enumerate() {
            let off = place(m) * step + moved;
            if off != 0.0 {
                shifts.push((s.route, s.k, s.horizontal, off));
            }
        }
    }
    for (r, k, horizontal, off) in shifts {
        let points = &mut routes[r].points;
        for p in &mut points[k..k + 2] {
            if horizontal {
                p.y += off;
            } else {
                p.x += off;
            }
        }
    }
}

/// The routes' segments that may move: not a route's first or last,
/// and only of routes found around the obstacles.
fn segments(routes: &[Route]) -> Vec<Seg> {
    let mut out = Vec::new();
    for (r, route) in routes.iter().enumerate() {
        if !route.routed {
            continue;
        }
        let p = &route.points;
        for k in 1..p.len().saturating_sub(2) {
            let (a, b) = (p[k], p[k + 1]);
            let (before, after) = (p[k - 1], p[k + 2]);
            let horizontal = (a.y - b.y).abs() <= EPS;
            // Along the segment `u`, across it `v`.
            let uv = |p: Point| if horizontal { (p.x, p.y) } else { (p.y, p.x) };
            let ((ua, va), (ub, _)) = (uv(a), uv(b));
            // Which way it turns at an end: -1 toward smaller `v`.
            let turn = |end: Point, next: Point| (uv(next).1 - uv(end).1).signum();
            let (ta, tb) = (turn(a, before), turn(b, after));
            // The first end (smaller `u`) and the second.
            let ((u1, t1), (u2, t2)) = if ua <= ub {
                ((ua, ta), (ub, tb))
            } else {
                ((ub, tb), (ua, ta))
            };
            out.push(Seg {
                route: r,
                k,
                horizontal,
                at: va,
                lo: u1,
                hi: u2,
                key: (t1 + t2, t1 * u1 - t2 * u2),
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn route(points: &[[f32; 2]]) -> Route {
        route_with(points, 12.0)
    }

    fn route_with(points: &[[f32; 2]], margin: f32) -> Route {
        Route {
            margin,
            points: points.iter().map(|&p| Point::from(p)).collect(),
            routed: true,
        }
    }

    #[test]
    fn two_wires_down_one_channel_are_spread_apart() {
        // Both go down x = 50 between y = 0 and y = 100.
        let mut routes = vec![
            route(&[[0.0, 0.0], [50.0, 0.0], [50.0, 100.0], [100.0, 100.0]]),
            route(&[[0.0, 20.0], [50.0, 20.0], [50.0, 120.0], [100.0, 120.0]]),
        ];
        nudge(&mut routes, 6.0);
        let x0 = routes[0].points[1].x;
        let x1 = routes[1].points[1].x;
        // The second turns in lower at the top and out lower at the
        // bottom: on the left, it crosses neither turn of the first.
        assert_eq!(x0 - x1, 6.0);
        assert_eq!(x0 + x1, 100.0, "centred on the line");
        for r in &routes {
            assert_eq!(r.points[1].x, r.points[2].x, "still vertical");
            assert_eq!(r.points[0], Point::new(0.0, r.points[0].y));
        }
    }

    #[test]
    fn the_spread_is_at_most_the_margin() {
        let mut routes: Vec<Route> = (0..5)
            .map(|i| {
                let y = i as f32 * 10.0;
                route_with(
                    &[[0.0, y], [50.0, y], [50.0, y + 100.0], [100.0, y + 100.0]],
                    8.0,
                )
            })
            .collect();
        nudge(&mut routes, 6.0);
        let xs: Vec<f32> = routes.iter().map(|r| r.points[1].x).collect();
        let (lo, hi) = xs
            .iter()
            .fold((f32::MAX, f32::MIN), |(l, h), &x| (l.min(x), h.max(x)));
        assert!(hi - lo <= 8.0 + 1e-4, "{xs:?}");
    }

    #[test]
    fn a_group_beside_a_port_spreads_away_from_it() {
        // Both end in the top of a node at y = 112, down stubs from the
        // channel at y = 100.
        let mut routes = vec![
            route(&[[0.0, 0.0], [0.0, 100.0], [200.0, 100.0], [200.0, 112.0]]),
            route(&[[20.0, 0.0], [20.0, 100.0], [212.0, 100.0], [212.0, 112.0]]),
        ];
        nudge(&mut routes, 6.0);
        let ys = [routes[0].points[1].y, routes[1].points[1].y];
        assert_eq!((ys[0] - ys[1]).abs(), 6.0, "still spread: {ys:?}");
        for r in &routes {
            let n = r.points.len();
            let stub = r.points[n - 1].y - r.points[n - 2].y;
            assert!(stub >= 12.0, "a stub of {stub}");
        }
    }

    #[test]
    fn a_short_stub_keeps_its_length() {
        // The first ends 4 below the channel, closer than its margin.
        let mut routes = vec![
            route(&[[0.0, 0.0], [0.0, 100.0], [200.0, 100.0], [200.0, 104.0]]),
            route(&[[20.0, 0.0], [20.0, 100.0], [212.0, 100.0], [212.0, 130.0]]),
        ];
        nudge(&mut routes, 6.0);
        let stub = |r: &Route| r.points[3].y - r.points[2].y;
        assert!(stub(&routes[0]) >= 4.0, "{}", stub(&routes[0]));
        assert!(stub(&routes[1]) >= 12.0, "{}", stub(&routes[1]));
        assert_ne!(routes[0].points[1].y, routes[1].points[1].y);
    }

    #[test]
    fn stubs_on_both_sides_share_what_is_missing() {
        // One ends below the channel at y = 100, the other starts above
        // it, each with a stub of exactly the margin.
        let mut routes = vec![
            route(&[[0.0, 0.0], [0.0, 100.0], [200.0, 100.0], [200.0, 112.0]]),
            route(&[[50.0, 88.0], [50.0, 100.0], [300.0, 100.0], [300.0, 200.0]]),
        ];
        nudge(&mut routes, 6.0);
        let below = routes[0].points[3].y - routes[0].points[2].y;
        let above = routes[1].points[1].y - routes[1].points[0].y;
        assert_eq!(below, above, "the same length on both sides");
        assert_ne!(routes[0].points[1].y, routes[1].points[1].y);
    }

    #[test]
    fn segments_that_do_not_overlap_stay() {
        let mut routes = vec![
            route(&[[0.0, 0.0], [50.0, 0.0], [50.0, 40.0], [100.0, 40.0]]),
            route(&[[0.0, 60.0], [50.0, 60.0], [50.0, 100.0], [100.0, 100.0]]),
        ];
        let before = routes.clone();
        nudge(&mut routes, 6.0);
        assert_eq!(routes, before);
    }

    #[test]
    fn ports_never_move() {
        let mut routes = vec![
            route(&[[0.0, 0.0], [100.0, 0.0]]),
            route(&[[0.0, 0.0], [100.0, 0.0]]),
        ];
        let before = routes.clone();
        nudge(&mut routes, 6.0);
        assert_eq!(routes, before);
    }
}
