/**
* Name: laneformation
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model laneformation

/* Insert your model definition here */

global {
	geometry shape <- envelope(rectangle(100, 20));
	
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
	
	int flow parameter: true <- 1 min: 0 max: 3;
	geometry free_space <- copy(polygon([{20, 10 - thickness/2}, {70, 10 - thickness/2}, {70, 18 + thickness/2}, {20, 18 + thickness/2}]));
	
	int nb_people -> length(people);
	float thickness <- 2.0 #m;
	float area <- 50.0*6.0;
	float density -> 100*(nb_people + 1)/area;
	float step <- 1.0 #s;
	
	int count;
	int T;
	float average_time;
	
	init {
//		square 50m x 6m
		create wall {
			shape <- polyline([{20, 10}, {70, 10}]) + thickness/2;
			free_space <- free_space - (free_space inter shape);
		}
		
		create wall {
			shape <- polyline([{70, 18}, {20, 18}])  + thickness/2;
			free_space <- free_space - (free_space inter shape);
		}
		
		create place {
			location <- {20.5, 14};
			shape <- rectangle(6, 1) rotated_by 90;
		}
		
		create place {
			location <- {69.5, 14};
			shape <- rectangle(6, 1) rotated_by 90;
		}
	}

	reflex init_pedestrian_flow {
		create people number: flow {
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
			
			if flip(0.5) {
				location <- any_location_in(place(0));
				my_target <- any_location_in(place(1));
				color <- #red;
			} else {
				location <- any_location_in(place(1));
				my_target <- any_location_in(place(0));
				color <- #blue;
			}
		}
	}
	
	reflex compute_average_travel_time when: cycle > 50 {
		average_time <- T/count;
	}
}

species place {
	aspect default {
		draw shape color:#white border:#black;
	}
}

species people skills: [pedestrian] {
	float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	point my_target ;
	rgb color;
	int t;
	
	reflex move when: my_target != nil {
		do walk_to target: my_target bounds:free_space ;
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
//			species place;
			species people aspect:base;
		}
		monitor "Density (%)" value: density;
		monitor "No. people" value: nb_people;
		
		display my_chart refresh: every(20#cycle){
			chart "Average travel time"{
				data "T" value: average_time;
			}
		}
	}
}