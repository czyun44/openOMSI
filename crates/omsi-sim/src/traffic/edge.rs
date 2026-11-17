//! Ways that run out of the map: a road the map's author stopped at the edge of the tiles
//! it has. OMSI's cars drive on to its end and are taken off there, so the way is an exit,
//! not a dead end for a driver who has a choice (`AiState::choose_after` keeps clear of
//! ways that end within `DEAD_END`, which sent every car to the one other way at a fork
//! whose other branch was such a road).

use super::*;

impl Network {
    /// Whether lane `i`, which has no lane after it, runs out of the map: a street lane
    /// whose end points at a tile the map does not have, within `EDGE_LOOK` of its end. A
    /// way that ends in the middle of the map, where there is a tile to go on in, is a dead
    /// end.
    pub fn leaves_map(&self, i: usize) -> bool {
        let Some((tiles, size)) = &self.map_tiles else {
            return false;
        };
        let l = &self.lanes[i];
        if l.kind != LaneKind::Street || !l.next.is_empty() || l.points.len() < 2 {
            return false;
        }
        let (a, b) = (l.points[l.points.len() - 2], l.points[l.points.len() - 1]);
        let d = DVec3::new(b.x - a.x, b.y - a.y, 0.0);
        if d.length() < 1e-6 {
            return false;
        }
        let p = b + d.normalize() * EDGE_LOOK;
        !tiles.contains(&((p.x / size).floor() as i32, (p.y / size).floor() as i32))
    }

    /// Give the lanes that run out of the map the reach of a way that goes on, and the lanes
    /// before them with it (`reach` counts a lane without a lane after it as a dead end).
    /// Call it after `link` and after every `extend` (which settles the reach of the new
    /// lanes anew). Returns the number of lanes that run out of the map.
    pub fn apply_edge_exits(&mut self) -> usize {
        if self.map_tiles.is_none() || self.reach.len() != self.lanes.len() {
            return 0;
        }
        let mut pending: Vec<usize> = (0..self.lanes.len())
            .filter(|&i| self.leaves_map(i))
            .collect();
        let exits = pending.len();
        for &i in &pending {
            self.reach[i] = REACH_MAX;
        }
        while let Some(j) = pending.pop() {
            for &p in self.prev.get(j).map(|v| v.as_slice()).unwrap_or(&[]) {
                let counts = !self.lanes[j].no_cars || self.lanes[p].no_cars;
                let reach = (self.lanes[p].length() + self.reach[j]).min(REACH_MAX);
                if counts && reach > self.reach[p] {
                    self.reach[p] = reach;
                    pending.push(p);
                }
            }
        }
        exits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn street(from: (f64, f64), to: (f64, f64)) -> Lane {
        Lane::polyline(
            vec![DVec3::new(from.0, from.1, 0.0), DVec3::new(to.0, to.1, 0.0)],
            LaneKind::Street,
            3.0,
        )
    }

    fn net_with(tiles: &[(i32, i32)], lanes: Vec<Lane>) -> Network {
        let mut net = Network {
            lanes,
            ..Default::default()
        };
        net.map_tiles = Some((tiles.iter().copied().collect(), 300.0));
        net.link(1.5);
        net
    }

    #[test]
    fn a_lane_that_runs_out_into_a_missing_tile_leaves_the_map() {
        // (the lane points at tile (0, -1) from tile (0, 0), and the map has only the one)
        let mut net = net_with(&[(0, 0)], vec![street((50.0, 100.0), (50.0, 10.0))]);
        assert!(net.leaves_map(0));
        assert!(net.reach[0] < DEAD_END);
        assert_eq!(net.apply_edge_exits(), 1);
        assert!(net.reach[0] >= DEAD_END);
    }

    #[test]
    fn a_dead_end_in_the_middle_of_the_map_is_still_a_dead_end() {
        let mut net = net_with(
            &[(0, 0), (0, -1)],
            vec![street((50.0, 100.0), (50.0, 10.0))],
        );
        assert!(!net.leaves_map(0));
        assert_eq!(net.apply_edge_exits(), 0);
        assert!(net.reach[0] < DEAD_END);
    }

    #[test]
    fn a_map_without_a_tile_list_has_no_exits() {
        let mut net = net_with(&[(0, 0)], vec![street((50.0, 100.0), (50.0, 10.0))]);
        net.map_tiles = None;
        assert_eq!(net.apply_edge_exits(), 0);
    }

    #[test]
    fn the_lanes_before_an_exit_lead_on_with_it() {
        let mut net = net_with(
            &[(0, 0)],
            vec![
                street((50.0, 280.0), (50.0, 190.0)),
                street((50.0, 190.0), (50.0, 100.0)),
                street((50.0, 100.0), (50.0, 10.0)),
            ],
        );
        assert!(!net.leaves_map(0) && !net.leaves_map(1) && net.leaves_map(2));
        net.apply_edge_exits();
        assert!(net.reach.iter().all(|&r| r >= DEAD_END));
    }
}
