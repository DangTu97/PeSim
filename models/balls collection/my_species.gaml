/***
* Name: myspecies
* Based on the internal empty template. 
* Author: DangTu
* Tags: pedestrian, behavior 
* Description: 
* 3 levels of behaviors have been taken into account.
* Strategic level: select a goal to move to.
* Tactical level: how to find a path (using a navigation graph) towards the selected goal.
* Operational level: local interaction using simple SFM.
***/


model myspecies

/* Insert your model definition here */
global {
	graph network;
	geometry free_space;
	bool is_reproduced;
}

species room_space {
	rgb color;
	geometry geom_free;
	aspect default {
		draw shape color: color;
	}
}

species graph_network {
	list<point> nodes;
	list<geometry> edges;
	
	aspect default {
		loop node over:nodes {
			draw circle(0.5) at:node color: #green;
		}
		
		loop edge over:edges {
			draw edge color: #blue;
		}
	}
}

species obstacle {
	aspect default {
		draw shape color: #grey;
	}
}

species ball {
	float radius;
	rgb color;
	
	float distance_search;
	float mean_distance;
	list<people> neighboring_pedestrians;
	room_space current_room;
	
	reflex update_infor {
		neighboring_pedestrians <- people at_distance distance_search;
	}
	
	aspect default {
		draw circle(radius) color: color;
	}
}

species aggregate_agent {
	string type;
	list<ball> existing_balls;
}

species people skills: [pedestrian] {
	// intrinsic variables
	float shoulder_length;
	rgb color;
	float greed_factor;
	
	// extrinsic variables
	ball target_ball;
	aggregate_agent my_ball_set;
	point my_goal;
	float travel_cost;
	point previous_location;
	path path_plan;
	point next_target;
	string selection_strategy;
	int nb_shortest_candidates;
	float distance_search;
	list<ball> ball_list;
	
	// strategic level: goal selection
	action select_goal {
		path_plan <- nil;
		if (selection_strategy = "shortest_ball") {
			do select_shortest_goal;
		} else if (selection_strategy = "minimum_cost") {
			do select_minimum_cost_goal;
		} else if (selection_strategy = "ball_list") {
			do select_from_ball_list;
		} else {
			do select_random_goal;
		}
	}
	
	// randomly choose a ball to collect
	action select_random_goal {
		target_ball <- one_of(my_ball_set.existing_balls);	
		my_goal <- target_ball.location;
	}
	
	// choose a shortest ball to collect
	action select_shortest_goal {
		target_ball <- my_ball_set.existing_balls closest_to(location);
		my_goal <- target_ball.location;
	}
	
	action select_minimum_cost_goal {
		list<ball> k_shortest_balls <- my_ball_set.existing_balls closest_to(self, nb_shortest_candidates);
		
		float euclidean_distance;
		float density_value;
		float ball_value;
		list<float> cost_values;
		
		loop candidate_ball over: k_shortest_balls {
			// euclidean factor
			euclidean_distance <- distance_to(location, candidate_ball.location);
			
			// density factor
			density_value <- length(candidate_ball.neighboring_pedestrians) / candidate_ball.distance_search; 
			
			// long-term value factor
			ball_value <- distance_search / (length(candidate_ball neighbors_at distance_search) + 1);
			
			float cost <- greed_factor*euclidean_distance + density_value + ball_value;
			cost_values <- cost_values + cost;
		}
		
		// choose minimum-cost-value ball as the new target
		target_ball <- k_shortest_balls[cost_values index_of min(cost_values)];
		my_goal <- target_ball.location;
	}
	
	action select_from_ball_list {
		if ((ball_list != nil) and length(ball_list) > 0) {
			target_ball <- ball_list closest_to(self);
			my_goal <- target_ball.location;
		}
	}
	
	action action_when_arrive {
		// remove ball from shared set of balls
		remove target_ball from: my_ball_set.existing_balls;
		
		// remove ball from ball list
		if ((ball_list != nil) and length(ball_list) > 0) {
			remove target_ball from: ball_list;
		}
		
		if is_reproduced {
			list<room_space> room <- (room_space where (each.color = self.color)) - target_ball.current_room;
			create ball {
				color <- myself.color;
				radius <- 0.22 #m;
				distance_search <- 10.0 #m;
				current_room <- first(room);
				location <- any_location_in(current_room);
				
				ask myself {
					my_ball_set.existing_balls <- my_ball_set.existing_balls + myself;
				}
			}
		}
		
		// collect target ball and remove it from enviroment
		ask target_ball {
			do die;
		}
		
		// choose new target ball if there exists balls to collect
		if length(my_ball_set.existing_balls) > 0 {
			do select_goal;
		} 
	}
	
	// tactical level: path planning
	action plan_path {
		if (union(obstacle) != nil) {
			point start;
			point end;
			list<point> start_candidates <- network.vertices closest_to(location, nb_shortest_candidates);
			list<point> end_candidates <- network.vertices closest_to(my_goal, nb_shortest_candidates);
	
			loop s over: start_candidates {
				if !(line(location, s) intersects union(obstacle)) {
					start <- s;
					break;
				}
			}
	
			loop e over: end_candidates {
				if !(line(e, my_goal) intersects union(obstacle)) {
					end <- e;
					break;
				}
			}
			
			if (start != nil and end != nil and start != end) {
				path_plan <- network path_between(start, end);
			}
			
			if (path_plan != nil and path_plan.shape != nil) {
				next_target <- path_plan.shape.points[0];
			} else {
				next_target <- nil;
			}
		}
	}
	
	action select_new_next_target {
		if (path_plan != nil and path_plan.shape != nil) {
			int idx <- path_plan.shape.points index_of next_target;
			if (idx < length(path_plan.shape.points) - 1) {
				next_target <- path_plan.shape.points[idx + 1];
			} 
			else {
				next_target <- nil;
			}
		} 
	}

	reflex move {
		previous_location <- location;
		
		// strategic level: goal selection
		if ((target_ball = nil or dead(target_ball)) and (length(my_ball_set.existing_balls) > 0)) {
			do select_goal;
		}
		
		// tactical level: path planning
		if ((path_plan = nil) and (network != nil)) {
			do plan_path;
		}
		
		// operational level: SFM
		if (next_target = nil) {
//			do walk_to target: my_goal;
			do walk_to target: my_goal bounds: free_space;
		} else {
			// next target is the current waypoint
//			do walk_to target: next_target;
			do walk_to target: next_target bounds: free_space;
			
			if (distance_to(location, next_target) < 1.0 #m) {
				do select_new_next_target;
			}
		}
			
		// if arrives to target ball  
		if (distance_to(location, my_goal) < 0.5 #m) {
			do action_when_arrive;
		}
	}
	
	reflex compute_travel_cost {
		travel_cost <- travel_cost + distance_to(previous_location, location);
	}
	
	aspect default {
		draw triangle(shoulder_length) color: color rotate: heading + 90.0;
	}
}