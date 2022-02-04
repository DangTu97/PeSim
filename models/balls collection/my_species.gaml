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
	
	reflex update_infor {
		neighboring_pedestrians <- people at_distance distance_search;
		
//		float sum_distance;
//		loop ped over: neighboring_pedestrians {
//			sum_distance <- sum_distance + distance_to(location, ped.location);
//		}
//		mean_distance <- sum_distance/length(neighboring_pedestrians);
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
	
	// strategic level: goal selection
	action select_goal {
		if (selection_strategy = "shortest_ball") {
			do select_shortest_goal;
		} else if (selection_strategy = "minimum_cost") {
			do select_minimum_cost_goal;
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
		target_ball <- my_ball_set.existing_balls closest_to(self);
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
			ball_value <- length(candidate_ball neighbors_at distance_search) / distance_search;
			
			float cost <- greed_factor*euclidean_distance + density_value + ball_value;
			cost_values <- cost_values + cost;
		}
		
		// choose minimum-cost-value ball as the new target
		target_ball <- k_shortest_balls[cost_values index_of min(cost_values)];
		my_goal <- target_ball.location;
	}
	
	action action_when_arrive {
		// collect target ball and remove it from enviroment
		remove target_ball from: my_ball_set.existing_balls;
		ask target_ball {
			do die;
		}
	
		path_plan <- nil;
		
		// choose new target ball if there exists balls to collect
		if length(my_ball_set.existing_balls) > 0 {
			do select_goal;
		} 
	}
	
	// tactical level: path planning
	action plan_path {
		point start <- network.vertices closest_to(location);
		point end <- network.vertices closest_to(my_goal);

		path_plan <- network path_between(start, end);
	
//		if (path_plan != nil and path_plan.shape != nil) {
//			next_target <- path_plan.shape.points[0];
//		}
		
		if (path_plan != nil and path_plan.shape != nil) {
			next_target <- path_plan.shape.points[0];
		} else {
			next_target <- my_goal;
		}
	}
	
	action select_new_next_target {
		if (path_plan != nil and path_plan.shape != nil) {
			int idx <- path_plan.shape.points index_of next_target;
			if (idx < length(path_plan.shape.points) - 1) {
				next_target <- path_plan.shape.points[idx + 1];
			} else {
				next_target <- my_goal;
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
			do walk_to target: my_goal;
		} else {
			// next target is the current waypoint
			do walk_to target: next_target;
			if (distance_to(location, next_target) < 0.5#m) {
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