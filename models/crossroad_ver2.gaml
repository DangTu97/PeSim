/**
* Name: crossroad
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model crossroadver2

/* Insert your model definition here */

global {
	geometry shape <- square(70#m);
	float step <- 1.0 #s;	
	
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
						
//	horizontal space			
	geometry free_space1 <- polygon([{10, 31}, {60, 31}, {60, 39}, {10, 39}]);
//	vertical space
	geometry free_space2 <- polygon([{31, 10}, {39, 10}, {39, 60}, {31, 60}]);
									
	map<int, int> target_map <- [1::2, 2::1, 0::3, 3::0];
	map<int, rgb> color_map <- [0::#green, 1:: #red, 2::#blue, 3::#orange];
	int flow parameter: true <- 1 min: 0 max: 12;
	int nb_people -> length(people);
	float thickness <- 2.0 #m;
	float area <- 50.0*8.0*2.0 - 8.0*8.0;
	float density -> (nb_people + 1)/area;
	
	int count;
	int T;
	float average_time;
	
	init {
//		shape of one direction: 50m x 8m
		create wall {
			shape <- polyline([{10, 30}, {30, 30}, {30, 10}]) + thickness/2;
		}
		create wall {
			shape <- polyline([{40, 10}, {40, 30}, {60, 30}]) + thickness/2;
		}
		create wall {
			shape <- polyline([{40, 60}, {40, 40}, {60, 40}]) + thickness/2;
		}
		create wall {
			shape <- polyline([{10, 40}, {30, 40}, {30, 60}]) + thickness/2;
		}
		
		create place {
			location <- {35, 10.5};
			shape <- rectangle(8, 1);
		}
		create place {
			location <- {10.5, 35};
			shape <- rectangle(8, 1) rotated_by 90;
		}
		create place {
			location <- {59.5, 35};
			shape <- rectangle(8, 1) rotated_by 90;
		}
		create place {
			location <- {35, 59.5};
			shape <- rectangle(8, 1);
		}
		
		create free {
			shape <- free_space1;
		}
		
		create free {
			shape <- free_space2;
		}
	}
	
	reflex init_pedestrian_flow when: every(1#s) {
		create people number: flow {
			obstacle_consideration_distance <- P_obstacle_consideration_distance;
			pedestrian_consideration_distance <- P_pedestrian_consideration_distance;
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
			obstacle_species <- [wall];
			
			int idx <- one_of([0, 1, 2, 3]);
			location <- any_location_in(place(idx));
			my_target <- any_location_in(place(target_map[idx]));
			color <- color_map[idx];
			is_horizontal <- mod(idx, 3) = 0 ? false : true;
		}
	}
	
	reflex compute_average_travel_time when: (count != 0) {
		average_time <- T/count;
	}
}

species free {
	aspect default {
		draw shape color:#yellow;
	}
}

species people skills: [pedestrian] {
	float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	point my_target ;
	rgb color;
	int t;
	bool is_horizontal;
	
	reflex move when: my_target != nil {
		if is_horizontal {
			do walk_to target: my_target bounds:free_space1;
		} else {
			do walk_to target: my_target bounds:free_space2;
		}
		
		t <- t + 1;
		if (self distance_to my_target < 0.5) {
			T <- T + t;
			count <- count + 1;
			do die;
		}
	}
	
	aspect base {
//		draw triangle(shoulder_length) color: color rotate: heading + 90.0;
		draw circle(shoulder_length/2) color:color;
	}
}

species place {
	aspect default {
		draw shape color:#white border:#black;
	}
}

species wall {
	aspect default {
		draw shape color: #grey;
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.15;
	output {
		display my_display {
			species wall;
//			species free;
//			species place;
			species people aspect:base;
		}
	
		monitor "No. people" value: nb_people;
		monitor "Density" value: density;
		
		display my_chart refresh: every(20#cycle){
			chart "Average travel time"{
				data "T" value: average_time;
			}
		}
	}
}

