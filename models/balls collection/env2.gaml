/***
* Name: env2
* Based on the internal empty template. 
* Author: DangTu
* Tags: Delaunay triangulation, navigation graph, ball collection, obstacle.
* Description: two groups are assigned to collect a given set of balls randomly located on the environment with obstacles.
***/

model env2
import 'my_species.gaml'
import 'global_variables.gaml'
/* Insert your model definition here */

global {
	geometry shape <- envelope(square(100#m));
	geometry free_space <- copy(shape);
	int nb_balls <- 2000;
	float r <- 0.22 #m;
	int nb_people <- 200;
	int nb_points <- 300;
	int nb_obstacle <- 3;
	
	graph network;
	
	init {
		create obstacle number: nb_obstacle {
			location <- any_location_in(shape);
			shape <- rectangle(10, 30) rotated_by rnd(180);
			free_space <- free_space - (free_space inter shape);
		}
		
		create ball number: nb_balls {
			location <- any_location_in(free_space);
			radius <- r;
			color <- flip(0.5) ? #blue : #red;
			distance_search <- 10.0 #m;
		}
		create aggregate_agent {
			type <- "blue";
			existing_balls <- (ball where (each.color = #blue));
		}
		
		create aggregate_agent {
			type <- "red";
			existing_balls <- (ball where (each.color = #red));
		}
		
		// create navigation graph based on Delaunay triangulation
		list<point> my_points;
		loop i from: 1 to: nb_points {
			my_points <- my_points + any_location_in(shape);
		}

		list<geometry> triangulations <- triangulate(my_points);
		
		list<list<point>> endpoints;
		// remove duplicating edges
		loop triangle over: triangulations {
			loop i from: 0 to: 2 {
				list<point> e <- [triangle.points[i], triangle.points[mod(i+1, 3)]];
				if !(e in endpoints) and !(reverse(e) in endpoints) {
					endpoints <- endpoints + [e];
				}
			}
		}
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
		
		// create people
		create people number: nb_people {
			selection_strategy <- "shortest_ball";
			nb_shortest_candidates <- 3;
			distance_search <- 10.0 #m;
			greed_factor <- 0.05;
			location <- any_location_in(free_space);
			color <- flip(0.5) ? #blue : #red;
			
			my_ball_set <- (color = #blue) ? first(aggregate_agent where (each.type = "blue")) :
											 first(aggregate_agent where (each.type = "red"));
											 		 
			obstacle_consideration_distance <- P_obstacle_consideration_distance;
			pedestrian_consideration_distance <- P_pedestrian_consideration_distance;
			shoulder_length <- P_shoulder_length;
			avoid_other <- P_avoid_other;
			proba_detour <- P_proba_detour;
			
			use_geometry_waypoint <- P_use_geometry_target;
			tolerance_waypoint <- P_tolerance_target;
			pedestrian_species <- [people];
			obstacle_species<- [ball, obstacle];
			
			pedestrian_model <- P_model_type;
			
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
	
	reflex stop when: length(ball) = 0 {
		do pause;
		float total_travel_cost;
		loop ped over: people {
			total_travel_cost <- total_travel_cost + ped.travel_cost;
		}
		write total_travel_cost / nb_people;
	}
}

experiment env2 {
	float minimum_cycle_duration <- 0.1;
	output {
		display my_display {
			species obstacle;
			species graph_network;
			species ball;
			species people;
		}
	}
}