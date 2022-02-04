/**
* Name: env1
* Based on the internal empty template. 
* Author: DangTu
* Tags: Delaunay triangulation, navigation graph, ball collection.
* Description: two groups are assigned to collect a given set of balls randomly located on the environment without obstacles.
*/

model env1
import 'my_species.gaml'
import 'global_variables.gaml'
/* Insert your model definition here */

global {
	geometry shape <- envelope(square(100 #m));
	int nb_balls <- 2000;
	float r <- 0.22 #m;
	int nb_people <- 100;
	bool is_colabrated <- false;
	list<rgb> groups_colabrated <- [#blue];
	
	init {
		// create balls
		create ball number: nb_balls {
			location <- any_location_in(shape);
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
		
		// create people
		create people number: nb_people {
			selection_strategy <- "minimum_cost";
			nb_shortest_candidates <- 3;
			distance_search <- 10.0 #m;
			greed_factor <- 0.1;
			location <- any_location_in(shape);
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
			obstacle_species<- [ball];
			
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
		
		if (is_colabrated) {
			loop group_color over: groups_colabrated {
				// clusters by index
				list<ball> balls <- ball where (each.color = group_color);
				list<people> group <- people where (each.color = group_color);
				list<list<float>> coordinates;
				loop b over: balls {
					coordinates <- coordinates + [[b.location.x, b.location.y]];
				}
				
				list<list<int>> cluster_indices <- kmeans(coordinates, length(group));
				loop indices over: cluster_indices {
					point centroid;
					loop idx over: indices {
						centroid <- centroid + {balls[idx].location.x, balls[idx].location.y};
					}
					centroid <- centroid / length(indices);
					people ag <- group closest_to(centroid);
					ask ag {
						selection_strategy <- "ball_list";
						loop idx over: indices {
							ball_list <- ball_list + balls[idx];
						}
					}
					remove ag from: group;
				}
			}
		}
	}
	
	reflex stop when: length(ball) = 0 {
		do pause;
		float total_travel_cost;
		loop ped over: people {
			total_travel_cost <- total_travel_cost + ped.travel_cost;
		}
		write "Average traveled distance:";
		write total_travel_cost / nb_people;
		
		float red;
		list<people> red_group <- people where (each.color = #red);
		loop ped over: red_group {
			red <- red + ped.travel_cost;
		}
		write "Average traveled distance of red group:";
		write red / length(red_group);
		
		float blue;
		list<people> blue_group <- people where (each.color = #blue);
		loop ped over: blue_group {
			blue <- blue + ped.travel_cost;
		}
		write "Average traveled distance of blue group:";
		write blue / length(blue_group);
	}
}

experiment env1 {
	float minimum_cycle_duration <- 0.05;
	output {
		display my_display {
			species ball;
			species people;
		}
	}
}