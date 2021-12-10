/**
* Name: bottleneck
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model bottleneck

/* Insert your model definition here */

global {
	geometry shape <- square(70);
	
	float P_shoulder_length <- 0.45 parameter: true;
	float P_proba_detour <- 1.0 parameter: true ;
	bool P_avoid_other <- true parameter: true ;
	float P_obstacle_consideration_distance <- 5.0 parameter: true ;
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
	
	bool display_force <- false parameter: true;
	bool display_circle_min_dist <- true parameter: true;
	int nb_people <- 500;
	geometry free_space <- copy(shape);
	
	init {
		create bound {
			shape <- polygon([{0, 0}, {0, 70}, {70, 70}, {70, 0}]);
		}
		
		create wall {
//			shape <- polyline([{29, 60}, {10, 60}, {10, 10}, {60, 10}, {60, 60}, {31, 60}]);
//			shape <- polyline([ {10, 60}, {10, 10}, {60, 10}, {60, 60}, {10, 60}]);
			shape <- polyline([{27, 60}, {10, 60}, {10, 10}, {60, 10}, {60, 60}, {33, 60}]);
			shape <- shape + 1.0;
			free_space <- free_space - shape;
		}
		
		create destination_zone {
			shape <- polygon([{29, 65.5}, {31, 65.5}, {31, 67.5}, {29, 67.5}]);	
		}
		
		create people number: nb_people {
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
			
			location <- any_location_in(polygon([{12, 12}, {58, 12}, {58, 58}, {12, 58}]));
			my_target <- any_location_in(one_of(destination_zone));
		}
	}
}

species people skills: [pedestrian] {
	rgb color <- rnd_color(255);
	float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	point my_target ;
	
	reflex move when: my_target != nil {
//		do walk_to target: my_target;
		do walk_to target: my_target bounds:free_space ;
		if (self distance_to my_target < 0.5) {
			do die;
		}
	}
	
	aspect base {
		draw triangle(shoulder_length) color: color rotate: heading + 90.0;
//		draw circle(shoulder_length) color: color;
//		draw circle(pedestrian_consideration_distance) at:location border:#red wireframe: true;
		if  display_force {
			loop op over: forces.keys {
				if (species(agent(op)) = wall ) {
					draw line([location, location + point(forces[op])]) color: #red end_arrow: 0.1;
				}
				else if ((agent(op)) = self ) {
					draw line([location, location + point(forces[op])]) color: #blue end_arrow: 0.1;
				} 
				else {
					draw line([location, location + point(forces[op])]) color: #green end_arrow: 0.1;
				}
			}
		}
	}
}

species destination_zone {
	aspect default {
		draw shape color: #white border:#black;
	}
}

species wall {
	aspect default {
		draw (shape + 0.2#m) border: #black depth: 0.3#m;
	}
}

species bound {
	aspect default {
		draw shape color: rgb(245, 245, 245);
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.15;
	output {
		display my_display {
			species bound;
			species wall;
			species destination_zone;
			species people aspect: base;
		}			
	}
}