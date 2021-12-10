/**
* Name: crossroadver2
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model crossroadver2

/* Insert your model definition here */

global {
	geometry shape <- square(130);
	float thickness <- 2.0 #m;
	geometry free_space <- copy(shape);
	
	float P_shoulder_length <- 0.45 parameter: true;
	float P_proba_detour <- 1.0 parameter: true ;
	bool P_avoid_other <- true parameter: true ;
	float P_obstacle_consideration_distance <- 2.0 parameter: true ;
	float P_pedestrian_consideration_distance <- 5.0 parameter: true ;
	float P_tolerance_waypoint <- 0.5 parameter: true;
	bool P_use_geometry_waypoint <- true parameter: true;
	
	string P_model_type <- "simple" among: ["simple", "advanced"] parameter: true ; 
	
	float P_A_pedestrian_SFM_advanced parameter: true <- 0.16 category: "SFM advanced" ;
	float P_A_obstacles_SFM_advanced parameter: true <- 0.1 category: "SFM advanced" ;
	float P_B_pedestrian_SFM_advanced parameter: true <- 0.1 category: "SFM advanced" ;
	float P_B_obstacles_SFM_advanced parameter: true <- 0.1 category: "SFM advanced" ;
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
	
	map<int, int> target_map <- [1::2, 2::1, 0::3, 3::0];
	map<int, rgb> color_map <- [0::#green, 1:: #red, 2::#blue, 3::#orange];
	
	init {
		create wall {
			shape <- polyline([{10, 50}, {60, 50}, {60, 0}]);
			shape <- shape + thickness/2;
		}
		create wall {
			shape <- polyline([{70, 0}, {70, 50}, {120, 50}]);
			shape <- shape + thickness/2;
		}
		create wall {
			shape <- polyline([{10, 60}, {60, 60}, {60, 110}]);
			shape <- shape + thickness/2;
		}
		create wall {
			shape <- polyline([{70, 110}, {70, 60}, {120, 60}]);
			shape <- shape + thickness/2;
		}
		
		create place {
			location <- {65, 0};
			shape <- circle(2.0);
		}
		create place {
			location <- {10, 55};
			shape <- circle(2.0);
		}
		create place {
			location <- {120, 55};
			shape <- circle(2.0);
		}
		create place {
			location <- {65, 110};
			shape <- circle(2.0);
		}
	}
	
//	when: mod(cycle, 2)=0
	reflex init_pedestrian_flow {
		create people number: 1 {
			obstacle_consideration_distance <-P_obstacle_consideration_distance;
			pedestrian_consideration_distance <-P_pedestrian_consideration_distance;
			shoulder_length <- P_shoulder_length;
			avoid_other <- P_avoid_other;
			proba_detour <- P_proba_detour;
			use_geometry_waypoint <- P_use_geometry_waypoint;
			tolerance_waypoint <- P_tolerance_waypoint;
			
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
				relaxion_SFM <- 1.0;
				gama_SFM <- P_gama_SFM_advanced;
				lambda_SFM <- P_lambda_SFM_advanced;
				minimal_distance <- P_minimal_distance_advanced;
			}
			
			pedestrian_species <- [people];
			obstacle_species<-[wall];
			
			int idx <- one_of([0, 1, 2, 3]);
			location <- any_location_in(place(idx));
			my_target <- any_location_in(place(target_map[idx]));
			color <- color_map[idx];
		}
		
		if (cycle = 500) {
			do pause;
		}
	}
}

species people skills: [pedestrian] {
	float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	point my_target ;
	rgb color;
	
	reflex move when: my_target != nil {
//		do walk_to target: my_target;
		do walk_to target: my_target bounds:free_space ;
		if (self distance_to my_target < 3.0) {
			do die;
		}
	}
	
	aspect base {
//		draw triangle(shoulder_length) color: color rotate: heading + 90.0;
		draw circle(shoulder_length) color:color;
	}
}

species place {
	aspect default {
		draw shape color:#white border:#black;
	}
}

species wall {
	aspect default {
		draw shape color: #yellow;
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.15;
	output {
		display my_display {
			species wall;
//			species place;
			species people aspect:base;
		}
	}
}
