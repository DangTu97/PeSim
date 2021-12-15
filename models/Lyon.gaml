/**
* Name: Lyon
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model Lyon

/* Insert your model definition here */

global {
	string ped_map <- "map1" among: ["map1","map2"];
	
	shape_file building_shape_file <- file('../includes/lyon/polygons.shp');
	shape_file boudary_shape_file <- file('../includes/lyon/boundary.shp');
	
	shape_file free_spaces_shape_file <- shape_file("../includes/"+ped_map+"/free spaces.shp");
	shape_file open_area_shape_file <- shape_file("../includes/"+ped_map+"/open area.shp");
	shape_file pedestrian_paths_shape_file <- shape_file("../includes/"+ped_map+"/pedestrian paths.shp");

	bool display_free_space <- false parameter: true;
	bool display_force <- false parameter: true;
	bool display_target <- false parameter: true;
	bool display_circle_min_dist <- true parameter: true;
	
	float P_shoulder_length <- 0.45 parameter: true;
	float P_proba_detour <- 0.5 parameter: true ;
	bool P_avoid_other <- true parameter: true ;
	float P_obstacle_consideration_distance <- 3.0 parameter: true ;
	float P_pedestrian_consideration_distance <- 3.0 parameter: true ;
	float P_tolerance_target <- 0.1 parameter: true;
	bool P_use_geometry_target <- true parameter: true;
	
	string P_model_type <- "simple" among: ["simple", "advanced"] parameter: true ; 
	
	float P_A_pedestrian_SFM_advanced parameter: true <- 0.16 category: "SFM advanced" ;
	float P_A_obstacles_SFM_advanced parameter: true <- 1.9 category: "SFM advanced" ;
	float P_B_pedestrian_SFM_advanced parameter: true <- 0.1 category: "SFM advanced" ;
	float P_B_obstacles_SFM_advanced parameter: true <- 1.0 category: "SFM advanced" ;
	float P_relaxion_SFM_advanced  parameter: true <- 0.5 category: "SFM advanced" ;
	float P_gama_SFM_advanced parameter: true <- 0.35 category: "SFM advanced" ;
	float P_lambda_SFM_advanced <- 0.1 parameter: true category: "SFM advanced" ;
	float P_minimal_distance_advanced <- 0.25 parameter: true category: "SFM advanced" ;
	
	float P_n_prime_SFM_simple parameter: true <- 3.0 category: "SFM simple" ;
	float P_n_SFM_simple parameter: true <- 2.0 category: "SFM simple" ;
	float P_lambda_SFM_simple <- 2.0 parameter: true category: "SFM simple" ;
	float P_gama_SFM_simple parameter: true <- 0.35 category: "SFM simple" ;
	float P_relaxion_SFM_simple parameter: true <- 0.54 category: "SFM simple" ;
	float P_A_pedestrian_SFM_simple parameter: true <- 4.5category: "SFM simple" ;
	
	geometry open_area;
	graph network;
	int nb_people <- 2000;
	string scenario <- "wandering" among: ["wandering", "attractions"] ;
	
	list<string> attractions <- ['Bellecour', 'Place des Jacobins', 'Grande Poste', 'Cour Saint-Martin'];
	map<string, geometry> attraction_map;
	
	geometry shape <- envelope(building_shape_file);
	
 	init {
 		create building from: building_shape_file {
 			color <- #grey;
 			if name in attractions {
 				attraction_map[name] <- shape;
 				color <- #red;
 			}
 		}
 		
		open_area <- first(open_area_shape_file.contents);
		create pedestrian_path from: pedestrian_paths_shape_file {
			list<geometry> fs <- free_spaces_shape_file overlapping self;
			free_space <- fs first_with (each covers shape); 
		}
		
		network <- as_edge_graph(pedestrian_path);
		
		create people number: nb_people {
			location <- any_location_in(one_of(open_area));
			obstacle_consideration_distance <-P_obstacle_consideration_distance;
			pedestrian_consideration_distance <-P_pedestrian_consideration_distance;
			shoulder_length <- P_shoulder_length;
			avoid_other <- P_avoid_other;
			proba_detour <- P_proba_detour;
			
			use_geometry_waypoint <- P_use_geometry_target;
			tolerance_waypoint<- P_tolerance_target;
			pedestrian_species <- [people];
			obstacle_species<-[building];
			
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
	}
}

species people skills: [pedestrian]{
	rgb color <- rnd_color(255);
	float speed <- gauss(5,1.5) #km/#h min: 2 #km/#h;
	point my_target;
	string current_attraction;
	building attractive_builing;
	
	reflex wander when: (scenario = "wandering") {
		if (final_waypoint = nil) {
			do compute_virtual_path pedestrian_graph:network target: any_location_in(open_area) ;
		}
	
		do walk;
	}	

	reflex move_to_attraction when: (scenario = "attractions") {
		if (final_waypoint = nil) {
			current_attraction <- one_of(attractions - current_attraction);
			attractive_builing <- first(building where (each.name = current_attraction));
			my_target <- any_location_in(attractive_builing);
			do compute_virtual_path pedestrian_graph:network target: my_target;
		}
	
		do walk;
	}	
	
	aspect default {
		if display_circle_min_dist and minimal_distance > 0 {
			draw circle(minimal_distance).contour color: color;
		}
		
//		draw triangle(shoulder_length) color: color rotate: heading + 90.0;
		draw circle(1.0) color: color;
			
		if display_target and current_waypoint != nil {
			draw line([location,current_waypoint]) color: color;
		}
	}
}

species building {
	aspect default {
		draw shape color: (name in attractions ? #red : #grey);
	}
}

species pedestrian_path skills: [pedestrian_road]{
	aspect default { 
		draw shape  color: #gray;
	}
	aspect free_area_aspect {
		if(display_free_space and free_space != nil) {
			draw free_space color: #lightpink border: #black;
		}
		
	}
}

experiment move_to_attractions type: gui {
	float minimum_cycle_duration <- 0.05;
	action _init_ {
		create simulation with: [scenario :: "attractions", nb_people::2000, 
								 free_spaces_shape_file::shape_file("../includes/"+ped_map+"/free spaces.shp"), 
								 pedestrian_paths_shape_file::shape_file("../includes/"+ped_map+"/pedestrian paths.shp")];
	}
	output {
		display my_display {
			species building refresh: false;
			species pedestrian_path aspect:free_area_aspect transparency: 0.5 ;
			species pedestrian_path refresh: false;
			species people;
		}
	}
}

experiment wandering type: gui {
	float minimum_cycle_duration <- 0.05;
	action _init_ {
		create simulation with: [scenario :: "wandering", nb_people::2000, 
								 free_spaces_shape_file::shape_file("../includes/"+ped_map+"/free spaces.shp"), 
								 pedestrian_paths_shape_file::shape_file("../includes/"+ped_map+"/pedestrian paths.shp")];
	}
	output {
		display my_display {
			species building refresh: false;
			species pedestrian_path aspect:free_area_aspect transparency: 0.5 ;
			species pedestrian_path refresh: false;
			species people;
		}
	}
}