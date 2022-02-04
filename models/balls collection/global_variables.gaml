/**
* Name: globalvariables
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model globalvariables

/* Insert your model definition here */

global {
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
	float P_B_pedestrian_SFM_advanced parameter: true <- 3.0 category: "SFM advanced" ;
	float P_B_obstacles_SFM_advanced parameter: true <- 3.0 category: "SFM advanced" ;
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
}