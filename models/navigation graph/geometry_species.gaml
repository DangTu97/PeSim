/**
* Name: species
* Based on the internal empty template. 
* Author: DangTu
* Tags: 
*/


model geometryspecies

/* Insert your model definition here */

species graph_network {
	list<point> nodes;
	list<geometry> edges;
	
	aspect default {
//		loop node over:nodes {
//			draw circle(0.5) at:node color: #red;
//		}
		
		loop edge over:edges {
			draw edge color: #blue;
		}
	}
}

species voronoi_cell {
	geometry voronoi_boundary;
	aspect default {
		draw polygon(voronoi_boundary.points) color: #white border: #red wireframe: true;
		draw circle(0.2) at: location color: #red;
	}
}

species delaunay_triangulation {
	geometry delaunay_boundary;
	aspect default {
		draw polygon(delaunay_boundary.points) color: #white border: #blue wireframe: true;
	}
}