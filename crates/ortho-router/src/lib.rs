#![doc = include_str!("../README.md")]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod geom;
mod grid;
mod nudge;

pub use geom::{Point, Rect, Side};

use geom::EPS;
use grid::{Grid, Search};

/// A connector's end: a point on a node's edge and the side it leaves
/// from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Port {
    /// Where the connector starts or ends, normally on its node's edge.
    pub at: Point,
    /// The side of the node the port is on: the connector leaves (or
    /// comes in) at a right angle to it.
    pub side: Side,
}

impl Port {
    /// A port at `at` on the `side` of its node.
    pub fn new(at: impl Into<Point>, side: Side) -> Port {
        Port {
            at: at.into(),
            side,
        }
    }
}

/// A connector to route, from one port to another.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Connector {
    /// Where it starts.
    pub from: Port,
    /// Where it ends.
    pub to: Port,
}

impl Connector {
    /// A connector from `from` to `to`.
    pub fn new(from: Port, to: Port) -> Connector {
        Connector { from, to }
    }

    /// Whether both ports have finite coordinates.
    fn is_finite(&self) -> bool {
        self.from.at.is_finite() && self.to.at.is_finite()
    }
}

/// How routes are drawn. The distances are in the caller's units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Options {
    /// How far routes keep from obstacles (default 12). It is also the
    /// length of the straight stub a route leaves a port with.
    pub margin: f32,
    /// What a bend costs, as a length (default 40): the higher, the
    /// fewer bends at the price of longer routes.
    pub bend_penalty: f32,
    /// How far apart [`Router::nudge`] spreads wires that share a
    /// channel (default 6; 0 leaves them on top of each other). A group
    /// spreads over the margin at most, so with many wires in one
    /// channel they come closer than this.
    pub spacing: f32,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            margin: 12.0,
            bend_penalty: 40.0,
            spacing: 6.0,
        }
    }
}

/// A routed connector: a polyline of horizontal and vertical segments.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Route {
    /// Its vertices, from the first port to the second, without
    /// repeated or collinear points.
    pub points: Vec<Point>,
    /// `false` when no way around the obstacles was found: `points` is
    /// then the fallback, a stub out of each port joined by a single
    /// elbow (orthogonal, but it may cross obstacles). A connector with
    /// a coordinate that is not finite falls back to the straight
    /// segment between its ports.
    pub routed: bool,
    /// The margin it keeps from the obstacles: the
    /// [`margin`](Options::margin), or a fraction of it when the route
    /// had to pass between obstacles closer than twice the margin.
    /// [`Router::nudge`] moves it by half of this at most.
    pub margin: f32,
}

impl Route {
    /// How many times it turns.
    pub fn bends(&self) -> usize {
        self.points.len().saturating_sub(2)
    }

    /// Its length.
    pub fn length(&self) -> f32 {
        self.points
            .windows(2)
            .map(|w| (w[1].x - w[0].x).abs() + (w[1].y - w[0].y).abs())
            .sum()
    }
}

/// Routes connectors around obstacles.
///
/// Keep one between frames: it keeps the obstacles and the search's
/// buffers. Its output depends only on its obstacles, its options and
/// the connectors, so a caller can cache routes under
/// [`fingerprint`](Router::fingerprint).
#[derive(Clone, Debug, Default)]
pub struct Router {
    obstacles: Vec<Rect>,
    options: Options,
    search: Search,
}

impl Router {
    /// A router with these options and no obstacles.
    pub fn new(options: Options) -> Router {
        Router {
            options,
            ..Router::default()
        }
    }

    /// The router with these obstacles.
    pub fn with_obstacles(mut self, obstacles: impl IntoIterator<Item = Rect>) -> Router {
        self.set_obstacles(obstacles);
        self
    }

    /// Replaces the obstacles, typically the nodes' rectangles. A
    /// rectangle that is not [valid](Rect::is_valid) is ignored.
    pub fn set_obstacles(&mut self, obstacles: impl IntoIterator<Item = Rect>) {
        self.obstacles.clear();
        self.obstacles
            .extend(obstacles.into_iter().filter(Rect::is_valid));
    }

    /// The obstacles.
    pub fn obstacles(&self) -> &[Rect] {
        &self.obstacles
    }

    /// The options.
    pub fn options(&self) -> &Options {
        &self.options
    }

    /// Replaces the options.
    pub fn set_options(&mut self, options: Options) {
        self.options = options;
    }

    /// Routes one connector.
    pub fn route(&mut self, connector: &Connector) -> Route {
        self.route_all(std::slice::from_ref(connector))
            .pop()
            .unwrap_or_default()
    }

    /// Routes connectors, in order: the routes match `connectors`.
    ///
    /// The connectors share one grid, built from the obstacles and every
    /// connector's ports, so routing many at once costs little more than
    /// routing one. Then wires sharing a channel are spread
    /// [`spacing`](Options::spacing) apart: their segments on one line
    /// move across it, the segments at the ports excepted.
    ///
    /// A connector with no route at the [`margin`](Options::margin) is
    /// tried again with half of it, up to three times, so a port between
    /// boxes closer than twice the margin still gets a route, running
    /// closer to them. Only then does it get the fallback.
    pub fn route_all(&mut self, connectors: &[Connector]) -> Vec<Route> {
        let margin = self.options.margin.max(0.0);
        let bend = self.options.bend_penalty.max(0.0);
        let mut routes = vec![Route::default(); connectors.len()];
        let mut pending: Vec<usize> = (0..connectors.len())
            .filter(|&i| connectors[i].is_finite())
            .collect();
        let mut tried = margin;
        for _ in 0..=MARGIN_HALVINGS {
            if pending.is_empty() {
                break;
            }
            let grown: Vec<Rect> = self.obstacles.iter().map(|o| o.expand(tried)).collect();
            let stubs: Vec<Point> = pending
                .iter()
                .flat_map(|&i| {
                    [
                        stub(&connectors[i].from, tried),
                        stub(&connectors[i].to, tried),
                    ]
                })
                .filter(|p| p.is_finite())
                .collect();
            let grid = Grid::build(&grown, &stubs);
            pending.retain(|&i| {
                match route_in(&grid, &mut self.search, &connectors[i], tried, bend) {
                    Some(route) => {
                        routes[i] = route;
                        false
                    }
                    None => true,
                }
            });
            if tried == 0.0 {
                break;
            }
            tried /= 2.0;
        }
        for (route, c) in routes.iter_mut().zip(connectors) {
            if route.points.is_empty() {
                *route = fallback_route(c, margin);
            }
        }
        self.nudge(&mut routes);
        routes
    }

    /// Spreads wires that share a channel [`spacing`](Options::spacing)
    /// apart: segments of different routes that lie on one line and
    /// overlap move across it, the segments at the ports excepted.
    /// [`route_all`](Router::route_all) does this already.
    ///
    /// On its own, it nudges routes found in separate calls, for
    /// example routes found with different obstacles each. A group of
    /// segments spreads over the smallest [`margin`](Route::margin) of
    /// its routes, so a nudged segment keeps at least half of it from
    /// the obstacles its route was found with. Routes that were not
    /// [`routed`](Route::routed) stay as they are.
    pub fn nudge(&self, routes: &mut [Route]) {
        nudge::nudge(routes, self.options.spacing);
    }

    /// A hash of the obstacles, the options and `connectors`: the same
    /// value, the same routes. Stable across runs and platforms (FNV-1a
    /// over the coordinates' bits), so it can key a cache.
    pub fn fingerprint(&self, connectors: &[Connector]) -> u64 {
        let mut h = Fnv::default();
        h.f32(self.options.margin);
        h.f32(self.options.bend_penalty);
        h.f32(self.options.spacing);
        h.u64(self.obstacles.len() as u64);
        for o in &self.obstacles {
            for v in [o.min.x, o.min.y, o.max.x, o.max.y] {
                h.f32(v);
            }
        }
        h.u64(connectors.len() as u64);
        for c in connectors {
            for p in [c.from, c.to] {
                h.f32(p.at.x);
                h.f32(p.at.y);
                h.u64(p.side as u64);
            }
        }
        h.0
    }
}

/// Where a route leaving `port` turns for the first time at the
/// earliest: `margin` out of the port.
fn stub(port: &Port, margin: f32) -> Point {
    port.at.toward(port.side, margin)
}

/// How many times a connector with no route is tried again with half
/// the margin: down to an eighth of it.
const MARGIN_HALVINGS: u32 = 3;

/// One connector over the grid, if it has a route there.
fn route_in(
    grid: &Grid,
    search: &mut Search,
    c: &Connector,
    margin: f32,
    bend: f32,
) -> Option<Route> {
    let (sa, sb) = (stub(&c.from, margin), stub(&c.to, margin));
    let (va, vb) = grid.vertex(sa).zip(grid.vertex(sb))?;
    // The route arrives going into the second port's node.
    let path = search.find(grid, va, c.from.side, vb, c.to.side.opposite(), bend)?;
    Some(Route {
        margin,
        points: simplify(
            std::iter::once(c.from.at)
                .chain(path.into_iter().map(|v| grid.point(v)))
                .chain(std::iter::once(c.to.at)),
        ),
        routed: true,
    })
}

/// What a connector with no route is drawn as.
fn fallback_route(c: &Connector, margin: f32) -> Route {
    let (a, b) = (c.from.at, c.to.at);
    let points = if c.is_finite() {
        simplify(fallback(
            a,
            stub(&c.from, margin),
            c.from.side,
            stub(&c.to, margin),
            b,
        ))
    } else {
        vec![a, b]
    };
    Route {
        points,
        routed: false,
        margin,
    }
}

/// A stub out of each port joined by one elbow, halfway across.
fn fallback(a: Point, sa: Point, side: Side, sb: Point, b: Point) -> [Point; 6] {
    let (p, q) = if side.is_horizontal() {
        let x = (sa.x + sb.x) / 2.0;
        (Point::new(x, sa.y), Point::new(x, sb.y))
    } else {
        let y = (sa.y + sb.y) / 2.0;
        (Point::new(sa.x, y), Point::new(sb.x, y))
    };
    [a, sa, p, q, sb, b]
}

/// FNV-1a, 64 bits.
struct Fnv(u64);

impl Default for Fnv {
    fn default() -> Fnv {
        Fnv(0xcbf2_9ce4_8422_2325)
    }
}

impl Fnv {
    fn u64(&mut self, v: u64) {
        for b in v.to_le_bytes() {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    fn f32(&mut self, v: f32) {
        // -0.0 and 0.0 route the same.
        self.u64(u64::from((v + 0.0).to_bits()));
    }
}

/// The points without repeats and without the middle of three on one
/// line.
fn simplify(points: impl IntoIterator<Item = Point>) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::new();
    for p in points {
        if out.last().is_some_and(|l| l.near(p)) {
            continue;
        }
        if let [.., u, v] = out[..] {
            let vertical = (u.x - v.x).abs() <= EPS && (v.x - p.x).abs() <= EPS;
            let horizontal = (u.y - v.y).abs() <= EPS && (v.y - p.y).abs() <= EPS;
            if vertical || horizontal {
                out.pop();
            }
        }
        out.push(p);
    }
    out
}
