/**
* Name: pedestrianagent
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model pedestrianspecies

/* Insert your model definition here */
global {
	graph network;
	geometry shape;
	
	float P_shoulder_length <- 0.45 parameter: true;
	float P_proba_detour <- 0.5 parameter: true ;
	bool P_avoid_other <- true parameter: true ;
	float P_obstacle_consideration_distance <- 3.0 parameter: true ;
	float P_pedestrian_consideration_distance <- 3.0 parameter: true ;
	float P_tolerance_target <- 0.1 parameter: true;
	bool P_use_geometry_target <- true parameter: true;
	
	float P_A_pedestrian_SFM_advanced parameter: true <- 0.16 category: "SFM advanced" ;
	float P_A_obstacles_SFM_advanced parameter: true <- 0.1 category: "SFM advanced" ;
	float P_B_pedestrian_SFM_advanced parameter: true <- 3.0 category: "SFM advanced" ;
	float P_B_obstacles_SFM_advanced parameter: true <- 3.0 category: "SFM advanced" ;
	float P_relaxion_SFM_advanced  parameter: true <- 0.5 category: "SFM advanced" ;
	float P_gama_SFM_advanced parameter: true <- 0.35 category: "SFM advanced" ;
	float P_lambda_SFM_advanced <- 0.1 parameter: true category: "SFM advanced" ;
	float P_minimal_distance_advanced <- 0.5 parameter: true category: "SFM advanced" ;
	
	float P_n_prime_SFM_simple parameter: true <- 3.0 category: "SFM simple" ;
	float P_n_SFM_simple parameter: true <- 2.0 category: "SFM simple" ;
	float P_lambda_SFM_simple <- 2.0 parameter: true category: "SFM simple" ;
	float P_gama_SFM_simple parameter: true <- 0.35 category: "SFM simple" ;
	float P_relaxion_SFM_simple parameter: true <- 0.54 category: "SFM simple" ;
	float P_A_pedestrian_SFM_simple parameter: true <- 4.5category: "SFM simple" ;
}

species obstacle skills: [moving] {
	reflex move when: mod(cycle, 10) = 0 {
		do wander speed: 0.5 #m/#s amplitude: 5.0 bounds: shape;
	}
	
	aspect default {
		draw shape color: #grey;
	}
}

species people skills: [pedestrian] {
	point start;
	point goal;
	point next_target;
	path path_plan;
	
	float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	rgb color <- rnd_color(255);
	
	action static_update {
		// assigne new next target
		if (distance_to(location, next_target) < 0.5#m) {
			if (path_plan != nil and path_plan.shape != nil) {
				int idx <- path_plan.shape.points index_of next_target;
				if idx < length(path_plan.shape.points) - 1 {
						next_target <- path_plan.shape.points[idx + 1];
				}
			}
		}
	}
	
	action dynamic_update {
		// replan
		if ( (distance_to(location, next_target) < 0.5#m) or (line([start, next_target]) intersects union(obstacle)) ) {
			start <- next_target;
			path_plan <- network path_between(start, goal); 
			
			if (path_plan != nil and path_plan.shape != nil) {
				next_target <- path_plan.shape.points[1];
			} else {
				next_target <- goal;
			}
		}
	}
	
	reflex move {
		// initialize
		if (path_plan = nil) {
			path_plan <- network path_between(start, goal); 
			
			if (path_plan != nil and path_plan.shape != nil) {
				next_target <- path_plan.shape.points[0];
			} else {
				next_target <- goal;
			}
		}
		
		do walk_to target: next_target;
		
//		do static_update;
		do dynamic_update;
		
		if (distance_to(location, goal) < 0.5#m) {
			start <- goal;
			goal <- one_of(network.vertices - start);
			path_plan <- nil;
		}
	}
	
	aspect default {
		draw circle(P_shoulder_length/2) color: color;
	}
}