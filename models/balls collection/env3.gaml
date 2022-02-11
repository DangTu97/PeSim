/***
* Name: env3
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
* Description: two rooms exchange without navigation graph
***/


model env3
import 'my_species.gaml'
import 'global_variables.gaml'

/* Insert your model definition here */
global {
	float env_width <- 100.0 #m;
	float env_length <- 50.0 #m;
	float corridor_width <- 5.0 #m;
	float corridor_length <- 10.0 #m;
	
	int nb_people <- 200;
	int nb_balls <- 1000;
	
	bool is_colabrated <- false;
	list<rgb> groups_colabrated <- [#blue];
			
	geometry shape <- envelope(rectangle(env_width, env_length));
	geometry free_space <- copy(shape);
	
	init {
		create obstacle number: 1 {
			shape <- polyline([{0, 0}, {env_width, 0}, {env_width, env_length}, {0, env_length}, {0, 0}]);
			shape <- shape + 1;
			free_space <- free_space - (free_space inter shape);
		}
		
		create obstacle {
			location <- {env_width/2, (env_length - 1) - (env_length - 2 - corridor_width)/4};
			shape <- rectangle(corridor_length, (env_length - 2 - corridor_width)/2);
			free_space <- free_space - (free_space inter shape);
		}
		
		create obstacle {
			location <- {env_width/2, 1 + (env_length - 2 - corridor_width)/4};
			shape <- rectangle(corridor_length, (env_length - 2 - corridor_width)/2);
			free_space <- free_space - (free_space inter shape);
		}

		create target_space {
			color <- #blue;
			shape <- polygon([{3, 3}, {env_width/2 - corridor_length/2 - 2, 3}, {env_width/2 - corridor_length/2 - 2, env_length - 3}, {3, env_length - 3}]);
		}
		
		create target_space {
			color <- #red;
			shape <- polygon([{env_width/2 + corridor_length/2 + 2, 3}, {env_width - 3, 3}, {env_width - 3, env_length - 3}, {env_width/2 + corridor_length/2 + 2, env_length - 3}]);
		}
		
		create ball number: nb_balls {
			radius <- 0.22 #m;
			color <- flip(0.5) ? #blue : #red;
			distance_search <- 10.0 #m;
			location <- (color = #blue) ? any_location_in(target_space(0).shape) : any_location_in(target_space(1).shape);
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
			selection_strategy <- "shortest_ball";
			nb_shortest_candidates <- 10;
			distance_search <- 10.0 #m;
			greed_factor <- 0.1;
			color <- flip(0.5) ? #blue : #red;
			location <- (color = #blue) ? any_location_in(target_space(1).shape) : any_location_in(target_space(0).shape);
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
			obstacle_species<- [obstacle];
			
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
	
}

species target_space {
	rgb color;
	aspect default {
		draw shape color: color;
	}
}

experiment env3 {
	float minimum_cycle_duration <- 0.1;
	output {
		display my_display {
			species obstacle;
			species ball;
			species people;
//			species target_space;
		}
	}
}
