// Alternative Transport A Star - Main

// ============================================================================================== //
// region: Imports
// ============================================================================================== //

use std::{collections::{HashMap, HashSet}, hash::Hash};

// endregion

// ============================================================================================== //
// region: enum Verbosity & macro verbose_print
// ============================================================================================== //

#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
enum Verbosity {
	None,
	Normal,
	Debug
}

macro_rules! verbose_print {
	($level:expr, $self:expr, $($arg:tt)*) => {
		if $self.verbosity >= $level {
			println!($($arg)*);
		}        
	};
}

// endregion

// ============================================================================================== //
// region: struct Coordinates
// ============================================================================================== //

#[derive(Clone, Copy, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Coordinates {
	x: i32,
	y: i32,
}

impl Coordinates {
	fn new(x: i32, y: i32) -> Coordinates {
		Coordinates { x, y }
	}

	fn cardinal_neighbors(&self) -> Vec<Coordinates> {
		vec![
			Coordinates::new(self.x, self.y + 1),
			Coordinates::new(self.x - 1, self.y),
			Coordinates::new(self.x + 1, self.y),
			Coordinates::new(self.x, self.y - 1)
		]
	}

	fn manhatten_distance(&self, other: &Coordinates) -> i32 {
		let horizontal: i32 = (other.x - self.x).abs();
		let vertical: i32 = (other.y - self.y).abs();
		horizontal + vertical
	}
}

// endregion

// ============================================================================================== //
// region: struct Cell
// ============================================================================================== //

#[derive(Clone, Copy)]
struct Cell {
	coordinates: Coordinates,
	g_score: Option<f32>,
	h_score: Option<f32>
}

impl PartialEq for Cell {
	fn eq(&self, other: &Self) -> bool {
		self.coordinates == other.coordinates
	}
}

impl Eq for Cell {}

impl std::hash::Hash for Cell {
	fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
		self.coordinates.hash(state);
	}
}

impl Cell {
	fn new(x: i32, y: i32) -> Cell {
		let coordinates = Coordinates { x, y };

		Cell { coordinates, g_score: None, h_score: None }
	}

	fn f_score(&self) -> Option<f32> {
		let g = self.g_score?;
		let h = self.h_score?;
		Some(g + h)
	}

	fn manhatten_distance(&self, other: &Coordinates) -> i32 {
		self.coordinates.manhatten_distance(other)
	}
}

// endregion

// ============================================================================================== //
// region: struct GridMap
// ============================================================================================== //

struct GridMap {
	size: Option<[i32; 2]>,
	obstacles: HashSet<Coordinates>
}

impl GridMap {
	fn new(width: i32, height: i32) -> GridMap {
		let obstacles = HashSet::new();
		GridMap { size: Some([width, height]), obstacles }
	}

	fn add_obstacles(&mut self, obstacles: Vec<Coordinates>) {
		self.obstacles.extend(obstacles);
	}

	fn within_bounds(&self, coordinates: &Coordinates) -> bool {
		if self.size.is_none() { return true; }
		let size = self.size.unwrap();

		0 <= coordinates.x && coordinates.x < size[0] &&
		0 <= coordinates.y && coordinates.y < size[1]
	}
}

// endregion

// ============================================================================================== //
// region: enum WorldMap
// ============================================================================================== //

enum WorldMap {
	GridMap(GridMap)
}

// endregion

// ============================================================================================== //
// region: struct AStarSearch

struct AStarSearch {
	world_map: WorldMap,
	start: Coordinates,
	targets: HashSet<Coordinates>,

	step_count: u32,
	history: HashMap<Coordinates, Coordinates>, 
	g_scores: HashMap<Coordinates, f32>,       
	frontier: HashSet<Cell>, 

	verbosity: Verbosity
}

impl AStarSearch {
	fn new(
		world_map: WorldMap,
		start: Coordinates,
		targets_vector: Vec<Coordinates>
	) -> AStarSearch {
		let verbosity = Verbosity::None;
		let targets: HashSet<Coordinates> = targets_vector.into_iter().collect();
		let step_count = 0;
		let history = HashMap::new();
		let mut g_scores = HashMap::new();
		
		let mut start_cell = Cell::new(start.x, start.y);
		start_cell.g_score = Some(0.0);
		g_scores.insert(start, 0.0);
		
		let initial_h = targets.iter()
			.map(|t| start.manhatten_distance(t) as f32)
			.min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
			.unwrap_or(0.0);
		start_cell.h_score = Some(initial_h);

		let frontier = HashSet::from([start_cell]);
		
		AStarSearch { world_map, start, targets, step_count, history, g_scores, frontier, verbosity }
	}

	fn is_frontier_empty(&self) -> bool {
		self.frontier.is_empty()
	}

	fn get_best_frontier_cell(&self) -> Option<Cell> {
		self.frontier
			.iter()
			.min_by(
				|a, b|
				{
					a
						.f_score()
						.unwrap_or(f32::MAX)
						.partial_cmp(
							&b
								.f_score()
								.unwrap_or(f32::MAX)
						)
						.unwrap_or(std::cmp::Ordering::Equal)
				}
			)
			.copied()
	}

	fn get_closest_target(&self, cell: Cell) -> (f32, Coordinates) {
		let mut result: Vec<(f32, Coordinates)> = self.targets
			.iter()
			.map(
				|target|
				(cell.manhatten_distance(target) as f32, *target)
			)
			.collect();

		result.sort_by(
			|a, b|
			a
				.0
				.partial_cmp(&b.0)
				.unwrap_or(std::cmp::Ordering::Equal)
			);
		result[0]
	}

	fn get_obstacles(&self) -> &HashSet<Coordinates> {
		match &self.world_map {
			WorldMap::GridMap(grid_map) => &grid_map.obstacles,
		}
	}

	fn get_path(&self, coordinates: Coordinates) -> Option<Vec<Coordinates>> {
		if !self.history.contains_key(&coordinates) && coordinates != self.start { 
			return None; 
		}

		verbose_print!(
			Verbosity::Normal,
			self,
			"Preparing path to Coordinates({}, {}).",
			coordinates.x,
			coordinates.y
		);

		let mut path = vec![coordinates];
		let mut current = coordinates;
		let mut visited = HashSet::from([coordinates]);

		while let Some(&parent) = self.history.get(&current) {
			if !visited.insert(parent) {
				verbose_print!(
					Verbosity::Debug,
					self,
					"Cycle detected at ({}, {})!",
					parent.x,
					parent.y
				);
				break;
			}
			path.push(parent);
			current = parent;
		}

		path.reverse();
		
		Some(path)
	}

	fn set_verbosity(&mut self, verbosity: Verbosity) {
		self.verbosity = verbosity;
	}

	fn is_obstacle(&self, coordinates: &Coordinates) -> bool {
		self.get_obstacles().contains(coordinates)
	}

	fn is_valid_coordinates(&self, coordinates: &Coordinates) -> bool {
		if self.is_obstacle(coordinates) { return false; }
		
		match &self.world_map {
			WorldMap::GridMap(grid_map) => grid_map.within_bounds(coordinates),
		}
	}
	
	fn step(&mut self) -> Option<Vec<Coordinates>> {
		self.step_count += 1;
		verbose_print!(Verbosity::Debug, self, "Beginning step sequence {}.", self.step_count);

		let frontier_cell = match self.get_best_frontier_cell() {
			Some(cell) => cell,
			None => {
				verbose_print!(Verbosity::Normal, self, "No frontier cells left!");
				return None;
			}
		};

		self.frontier.remove(&frontier_cell);

		if self.targets.contains(&frontier_cell.coordinates) {
			verbose_print!(Verbosity::Normal, self, "Target ({}, {}) found!", frontier_cell.coordinates.x, frontier_cell.coordinates.y);
			return self.get_path(frontier_cell.coordinates);
		}

		let neighbor_coordinates = frontier_cell.coordinates.cardinal_neighbors();
		let current_g = frontier_cell.g_score.unwrap_or(0.0);

		for coordinates in neighbor_coordinates {
			if !self.is_valid_coordinates(&coordinates) {
				continue;
			}

			let tentative_g = current_g + 1.0; 
			let current_best_g = self.g_scores
				.get(&coordinates)
				.copied()
				.unwrap_or(f32::MAX);

			if tentative_g < current_best_g {
				self.g_scores.insert(coordinates, tentative_g);
				self.history.insert(coordinates, frontier_cell.coordinates);

				let mut neighbor = Cell::new(coordinates.x, coordinates.y);
				neighbor.g_score = Some(tentative_g);
				neighbor.h_score = Some(self.get_closest_target(neighbor).0);

				self.frontier.remove(&neighbor);
				self.frontier.insert(neighbor);
			}
		}

		None
	}

	fn find_path(&self) -> Option<Vec<Coordinates>> {
		todo!()
	}
}

// endregion

// ============================================================================================== //
// region: ASCII Grid Visualizer

fn print_grid(
	width: i32, 
	height: i32, 
	start: Coordinates, 
	targets: &HashSet<Coordinates>, 
	obstacles: &[Coordinates], 
	path: &[Coordinates]
) {
	let path_set: HashSet<&Coordinates> = path.iter().collect();
	let obstacle_set: HashSet<&Coordinates> = obstacles.iter().collect();

	println!("\nGrid Visualization:");
	println!("  S = Start | T = Target | * = Path | # = Obstacle\n");

	for y in (0..height).rev() {
		print!("{:2} | ", y);
		for x in 0..width {
			let coord = Coordinates::new(x, y);
			
			let symbol = if coord == start {
				"S "
			} else if targets.contains(&coord) {
				"T "
			} else if path_set.contains(&coord) {
				"* "
			} else if obstacle_set.contains(&coord) {
				"# "
			} else {
				"  "
			};
			print!("{}", symbol);
		}
		println!();
	}

	print!("     ");
	for _ in 0..width {
		print!("- ");
	}
	println!();
	print!("    ");
	for x in 0..width {
		print!("{:2}", x);
	}
	println!("\n");
}

// endregion

// ============================================================================================== //
// region: fn main

fn main() {
	let start = Coordinates::new(0, 0);
	let target = Coordinates::new(9, 9);
	let mut grid_map = GridMap::new(10, 10);
	
	let obstacle_list = vec![
		Coordinates::new(1, 1),
		Coordinates::new(3, 6),
		Coordinates::new(9, 2),
		Coordinates::new(8, 8),
		Coordinates::new(4, 6),
		Coordinates::new(4, 5),
		Coordinates::new(5, 5),
	];
	
	grid_map.add_obstacles(obstacle_list.clone());

	let mut a_star_search = AStarSearch::new(
		WorldMap::GridMap(grid_map),
		start,
		vec![target]
	);
	a_star_search.set_verbosity(Verbosity::Normal);

	for _ in 0..1000 {
		match a_star_search.step() {
			Some(path) => {
				println!("Found path in {} steps:", a_star_search.step_count);
				
				print_grid(10, 10, start, &a_star_search.targets, &obstacle_list, &path);

				for coord in &path {
					print!("({},{}) ", coord.x, coord.y);
				}
				println!();
				break;
			}
			None => {
				if a_star_search.is_frontier_empty() {
					println!("No path found after {} steps.", a_star_search.step_count);
					break;
				}
			}
		}
	}
}

// endregion