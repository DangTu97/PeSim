/**
* Name: dynamicenvironment
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model dynamicenvironment
import 'geometry_species.gaml'
import 'pedestrian_species.gaml'

/* Insert your model definition here */
global {
	geometry shape <- envelope(square(100#m));
	geometry free_space <- copy(shape);
	int nb_points <- 400;
	int nb_obstacle <- 3;
	int nb_people <- 500;
	
	graph network;
	list<point> my_points;
	list<geometry> boundaries;
	list<geometry> triangulations;
	list<list<point>> endpoints;
	
	init {
		create obstacle number: nb_obstacle {
			location <- any_location_in(shape);
			shape <- rectangle(10, 30) rotated_by rnd(180);
		}
		
		loop i from: 1 to: nb_points {
			my_points <- my_points + any_location_in(free_space);
		}
		
		// compute Voronoi diagram by given points
		boundaries <- voronoi(my_points, shape);
		
		// correct point to its Voronoi cell
		list<geometry> boundaries_corrected;
		loop p over: my_points {
			loop b over: boundaries {
				if (p overlaps b) {
					boundaries_corrected <- boundaries_corrected + b;
				}
			}
		}
		
		// create Voronoi cell
		loop i from: 0 to: length(my_points) - 1 {
			create voronoi_cell {
				location <- my_points[i];
				voronoi_boundary <- boundaries_corrected[i];
			}
		}
		
		// create Delaunay triangulation
		triangulations <- triangulate(my_points);
		loop t over: triangulations {
			create delaunay_triangulation {
				delaunay_boundary <- t;
			}
		}
		
		// add edges of Delaunay triangulations to network
		loop ag over: delaunay_triangulation {
			loop i from: 0 to: 2 {
				list<point> e <- [ag.delaunay_boundary.points[i], ag.delaunay_boundary.points[mod(i+1, 3)]];
				// remove duplicating edges
				if !(e in endpoints) and !(reverse(e) in endpoints) {
					endpoints <- endpoints + [e];
				}
			}
		}
		
		// remove edges intersecting obstacles
		list<geometry> edges;
		loop endp over: endpoints {
			if !(line(endp) intersects union(obstacle)) {
				edges <- edges + line(endp);
			}
		}
		
		network <- as_edge_graph(edges);
		write "Number of vertices: " + length(network.vertices);
		write "Number of edges: " + length(network.edges);
		
		create graph_network {
			nodes <- network.vertices;
			edges <- network.edges;
		}
		
		create people number: nb_people {
			start <- one_of(network.vertices);
			goal <- one_of(network.vertices - start);
			location <- start + {rnd(3.0), rnd(3.0)};
			
			obstacle_consideration_distance <-P_obstacle_consideration_distance;
			pedestrian_consideration_distance <-P_pedestrian_consideration_distance;
			shoulder_length <- P_shoulder_length;
			avoid_other <- P_avoid_other;
			proba_detour <- P_proba_detour;
			
			use_geometry_waypoint <- P_use_geometry_target;
			tolerance_waypoint<- P_tolerance_target;
			pedestrian_species <- [people];
			obstacle_species<-[obstacle];
			
			pedestrian_model <- "simple";
			
			if (pedestrian_model = "simple") {
				A_pedestrians_SFM <- P_A_pedestrian_SFM_simple;
				relaxion_SFM <- P_relaxion_SFM_simple;
				gama_SFM <- P_gama_SFM_simple;
				lambda_SFM <- P_lambda_SFM_simple;
				n_prime_SFM <- P_n_prime_SFM_simple;
				n_SFM <- P_n_SFM_simple;
			} else {
				A_pedestrians_SFM <- P_A_pedestrian_SFM_advanced;
				A_obstacles_SFM <- P_A_obstacles_SFM_advanced;
				B_pedestrians_SFM <- P_B_pedestrian_SFM_advanced;
				B_obstacles_SFM <- P_B_obstacles_SFM_advanced;
				relaxion_SFM <- P_relaxion_SFM_advanced;
				gama_SFM <- P_gama_SFM_advanced;
				lambda_SFM <- P_lambda_SFM_advanced;
				minimal_distance <- P_minimal_distance_advanced;
			}
		}
	}	
	
	// when environment changes due to moving obstacles, recompute network
	reflex network_recompute {
		list<geometry> edges;
		loop endp over: endpoints {
			if !(line(endp) intersects union(obstacle)) {
				edges <- edges + line(endp);
			}
		}
		network <- as_edge_graph(edges);
		graph_network(0).edges <- network.edges;
	}
}


experiment dynamic_env type: gui {
	float minimum_cycle_duration <- 0.1;
	
	output {
		display exp1 {
			species obstacle;
//			species voronoi_cell;
//			species delaunay_triangulation;
			species graph_network;
			species people;
		}
	}
}


