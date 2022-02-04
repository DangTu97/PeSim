/**
* Name: navigation
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model navigation
import 'geometry_species.gaml'
import 'pedestrian_species.gaml'

/* Insert your model definition here */

global {
	geometry shape <- envelope(square(100 #m));
	int nb_points <- 200;
	int nb_people <- 300;
	
	graph network;
	list<point> my_points;
	list<geometry> boundaries;
	list<geometry> triangulations;
	list<list<point>> endpoints;
	
	init {
		loop i from: 1 to: nb_points {
			my_points <- my_points + any_location_in(shape);
		}
		
		// compute Voronoi diagram by given points
		boundaries <- voronoi(my_points, shape);
		
		// correct point to its voronoi cell
		list<geometry> boundaries_corrected;
		loop p over: my_points {
			loop b over: boundaries {
				if (p overlaps b) {
					boundaries_corrected <- boundaries_corrected + b;
				}
			}
		}
		
		// create Voronoi cell
		loop i from: 0 to: length(my_points) - 1 {
			create voronoi_cell {
				location <- my_points[i];
				voronoi_boundary <- boundaries_corrected[i];
			}
		}
		
		// create Delaunay triangulation
		triangulations <- triangulate(my_points);
		loop t over: triangulations {
			create delaunay_triangulation {
				delaunay_boundary <- t;
			}
		}
		
		// add edges of Delaunay triangulations to network
		loop ag over: delaunay_triangulation {
			loop i from: 0 to: 2 {
				list<point> e <- [ag.delaunay_boundary.points[i], ag.delaunay_boundary.points[mod(i+1, 3)]];
				// remove duplicating edges
				if !(e in endpoints) and !(reverse(e) in endpoints) {
					endpoints <- endpoints + [e];
				}
			}
		}
		
		list<geometry> edges;
		loop endp over: endpoints {
			edges <- edges + line(endp);
		}
		network <- as_edge_graph(edges);
		write "Number of vertices: " + length(network.vertices);
		write "Number of edges: " + length(network.edges); 
		
		create people number: nb_people {
			start <- one_of(network.vertices);
			goal <- one_of(network.vertices - start);
			location <- start + {rnd(3.0), rnd(3.0)};
			
			pedestrian_model <- "advanced";
			
			A_pedestrians_SFM <- P_A_pedestrian_SFM_advanced;
			A_obstacles_SFM <- P_A_obstacles_SFM_advanced;
			B_pedestrians_SFM <- P_B_pedestrian_SFM_advanced;
			B_obstacles_SFM <- P_B_obstacles_SFM_advanced;
			relaxion_SFM <- 1.0;
			gama_SFM <- P_gama_SFM_advanced;
			lambda_SFM <- P_lambda_SFM_advanced;
			minimal_distance <- P_minimal_distance_advanced;
		}
	}
}	

experiment nav_exp type: gui {
	float minimum_cycle_duration <- 0.05;
	
	output {
		display exp1 {
//			species voronoi_cell;
//			species delaunay_triangulation;
			species people;
		}
	}
}