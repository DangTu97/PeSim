/***
* Name: env7
* Based on the internal empty template. 
* Author: DangTu
* Tags:
* Description: regenate balls
***/


model env7
import 'my_species.gaml'
import 'global_variables.gaml'

/* Insert your model definition here */
global {
	float env_width <- 200.0 #m;
	float env_length <- 100.0 #m;
	geometry shape <- envelope(rectangle(env_width, env_length));
	geometry free_space <- copy(shape);
	geometry outdoor <- copy(shape);
	
	int nb_people <- 500;
	int nb_balls <- 1000;
	int nb_points <- 1200;
	
	bool is_colabrated <- false;
	list<rgb> groups_colabrated <- [];
	bool is_reproduced <- true;
	
	init {
		float door_width <- 8.0 #m;
		float room_width <- 50.0 #m;
		float room_length <- 30.0 #m;
		float thickness <- 1.0 #m;
		float x <- (room_width - door_width)/2;
		loop i from: 0 to: 1 {
			loop j from: 0 to: 1 {
				create obstacle {
					location <- {(env_width/2)*i + env_width/4, (env_length/2)*j + env_length/4};
					geometry rec <- rectangle(room_width, room_length);
					shape <- polyline(rec.points) - polyline([rec.points[0] + (rec.points[1] - rec.points[0])*x/room_width, 
															  rec.points[0] + (rec.points[1] - rec.points[0])*(x + door_width)/room_width]);
					shape <- shape + thickness;
					outdoor <- outdoor - (outdoor inter (rec + thickness));

					create room_space {
						location <- {(env_width/2)*i + env_width/4, (env_length/2)*j + env_length/4};
						shape <- rectangle(room_width - 2*thickness, room_length - 2*thickness);
						color <- (mod(i + j, 2) = 0) ? #blue : #red;
					}
				}
			}
		}
		
		free_space <- free_space - union(obstacle);
		
		create ball number: nb_balls {
			radius <- 0.22 #m;
			color <- flip(0.5) ? #blue : #red;
			distance_search <- 10.0 #m;
			current_room <- first(room_space where (each.color = color));
			location <- any_location_in(current_room);
		}
		
		create aggregate_agent {
			type <- "blue";
			existing_balls <- (ball where (each.color = #blue));
		}
		
		create aggregate_agent {
			type <- "red";
			existing_balls <- (ball where (each.color = #red));
		}
		
		list<point> my_points;
		loop i from: 1 to: nb_points {
			if (i < nb_points * 3 / 5) {
				my_points <- my_points + any_location_in(union(room_space));
			} else {
				my_points <- my_points + any_location_in(outdoor);
			}
			
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
			nb_shortest_candidates <- 10;
			distance_search <- 10.0 #m;
			greed_factor <- 0.1;
			color <- flip(0.5) ? #blue : #red;
			selection_strategy <- (color = #red) ? "shortest_ball" : "random";
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

experiment env7 {
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