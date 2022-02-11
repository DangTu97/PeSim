/***
* Name: env5
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
* Description: balls collection in 4 rooms without navigation
***/


model env5
import 'my_species.gaml'
import 'global_variables.gaml'

/* Insert your model definition here */
global {
	geometry shape <- envelope(rectangle(200, 100));
	geometry free_space <- copy(shape);
	geometry outdoor <- copy(shape);
	
	int nb_people <- 200;
	int nb_balls <- 1000;
	
	bool is_colabrated <- false;
	list<rgb> groups_colabrated <- [#blue];
	
	init {
		float door_width <- 8.0 #m;
		float room_width <- 60.0 #m;
		float room_length <- 30 #m;
		float thickness <- 1.0 #m;
		float x <- (room_width - door_width)/2;
		loop i from: 0 to: 1 {
			loop j from: 0 to: 1 {
				create obstacle {
					location <- {100*i + 50, 50*j + 25};
					geometry rec <- rectangle(room_width, room_length);
					shape <- polyline(rec.points) - polyline([rec.points[0] + (rec.points[1] - rec.points[0])*x/room_width, 
															  rec.points[0] + (rec.points[1] - rec.points[0])*(x + door_width)/room_width]);
					shape <- shape + thickness;
					outdoor <- outdoor - (outdoor inter (rec + thickness));
					
					create target_space {
						location <- {100*i + 50, 50*j + 25};
						shape <- rectangle(room_width - 2*thickness, room_length - 2*thickness);
					}
				}
			}
		}
		
		free_space <- free_space - union(obstacle);
		
		create ball number: nb_balls {
			radius <- 0.22 #m;
			color <- flip(0.5) ? #blue : #red;
			distance_search <- 10.0 #m;
			location <- any_location_in(union(target_space));
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
			location <- any_location_in(outdoor);
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
			obstacle_species <- [obstacle];
			
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
	geometry geom_free;
	aspect default {
		draw shape color: color;
	}
}

experiment env5 {
	float minimum_cycle_duration <- 0.05;
	output {
		display my_display {
			species obstacle;
			species graph_network;
			species ball;
			species people;
//			species target_space;
		}
	}
}
