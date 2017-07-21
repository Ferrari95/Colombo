#include "p1/graph.h"

typedef struct vertex_s {
	int id;
	vertex_type state;
	
	std::vector<edge *> * adjacency_list;
	
	double source_distance;
	double destination_distance;
	vertex * next;
	
	double x;
	double y;
	double theta;	
} vertex_;

typedef struct edge_s {
	double weight;
	vertex * vertex1;
	vertex * vertex2;
} edge_;

typedef struct graph_s {
	int progr;
	vertex * destination;
	int border_vertex_number;
	std::vector<vertex *> * vertex_list;
	std::vector<edge *> * edge_list;
} graph_;

graph * new_graph(double destination_x, double destination_y, double destination_theta) {
	graph_ * g = (graph_ *) malloc(sizeof(graph_));
	
	g->vertex_list = new std::vector<vertex *>;
	g->edge_list = new std::vector<edge *>;
	
	vertex_ * v = (vertex_ *) malloc(sizeof(vertex_));
	v->id = 0;
	v->next = NULL;
	v->destination_distance = 0.0;
	
	v->x = destination_x;
	v->y = destination_y;
	v->theta = destination_theta;
	v->adjacency_list = new std::vector<void *>;
	v->state = border;
	
	g->vertex_list->push_back(v);
	
	g->destination = v;
	g->progr = 1;
	g->border_vertex_number = 1;
	
	return (graph *) g;
}

vertex * add_vertex(graph * gg, double x, double y, double theta, vertex_type s) {
	graph_ * g = (graph_ *) gg;
	
	vertex_ * v = (vertex_ *) malloc(sizeof(vertex_));
	v->id = g->progr++;
	v->next = NULL;
	
	vertex_ * d = (vertex_ *) g->destination;
	v->destination_distance = sqrt(pow(x - d->x, 2) + pow(y - d->y, 2));
	
	v->x = x;
	v->y = y;
	v->theta = theta;
	v->adjacency_list = new std::vector<void *>;
	v->state = s;
	
	g->vertex_list->push_back(v);
	if(s == border)
		g->border_vertex_number++;
	
	return (vertex *) v;
}

bool equals_vertex(vertex * vv1, vertex * vv2) {
	vertex_ * v1 = (vertex_ *) vv1;
	vertex_ * v2 = (vertex_ *) vv2;
	
	if(v1->id == v2->id)	
		return true;
	return false;
}

bool equals_edge(edge * ee1, edge * ee2) {
	edge_ * e1 = (edge_ *) ee1;
	edge_ * e2 = (edge_ *) ee2;
	
	if(equals_vertex(e1->vertex1, e2->vertex1) && equals_vertex(e1->vertex2, e2->vertex2)
		|| equals_vertex(e1->vertex1, e2->vertex2) && equals_vertex(e1->vertex2, e2->vertex1))
		return true;
	return false;
}

edge * add_edge(graph * gg, vertex * vv1, vertex * vv2) {
	graph_ * g = (graph_ *) gg;
	vertex_ * v1 = (vertex_ *) vv1;
	vertex_ * v2 = (vertex_ *) vv2;
	
	edge_ * e = (edge_ *) malloc(sizeof(edge_));
	e->weight = sqrt(pow(v1->x - v2->x, 2) + pow(v1->y - v2->y, 2));
	e->vertex1 = (vertex *) v1;
	e->vertex2 = (vertex *) v2;
	
	for(edge * edge_iter : *(g->edge_list))
		if(equals_edge(e, edge_iter)) {
			free(e);
			return NULL;
		}
	
	v1->adjacency_list->push_back(e);
	v2->adjacency_list->push_back(e);
	
	g->edge_list->push_back(e);
	
	return e;
}

vertex * opposite(vertex * vv, edge * ee) {
	vertex_ * v = (vertex_ *) vv;
	edge_ * e = (edge_ *) ee;
	
	if(v == e->vertex1)
		return e->vertex2;
	
	return e->vertex1;
}

void print_vertex(vertex * vv) {
	vertex_ * v = (vertex_ *) vv;
	/*
	std::cout << "id: " << v->id << ", x: " << v->x << ", y: " << v->y << ", theta: " << v->theta
			  << ", edges: " << v->adjacency_list->size() << ", state: " << v->state
			  << ", source distance: " << v->source_distance << ", destination distance: " << v->destination_distance << "\n";
	*/
	std::cout << "id: " << v->id << ", x: " << v->x << ", y: " << v->y << ", state: " << v->state << ", edges: " << v->adjacency_list->size() << "\n";
}

void print_edge(edge * ee) {
	edge_ * e = (edge_ *) ee;
	std::cout << "id: " << ((vertex_ *) e->vertex1)->id << " <---> id: " << ((vertex_ *) e->vertex2)->id;
	std::cout << " ----------- weight: " << e->weight << "\n";
}

void print_graph(graph * gg) {
	graph_ * g = (graph_ *) gg;
	
	std::cout << "VERTEX LIST\n";
	for(vertex * vertex_iter : *(g->vertex_list))
		print_vertex(vertex_iter);
	
	std::cout << "\n\nEDGE LIST\n";
	for(edge * edge_iter : *(g->edge_list))
		print_edge(edge_iter);
	
	std::cout << "##########################################\n\n";
}

bool vertex_pq_comparator(vertex * vv1, vertex * vv2) {
	vertex_ * v1 = (vertex_ *) vv1;
	vertex_ * v2 = (vertex_ *) vv2;
	
	return (v1->source_distance > v2->source_distance);
}

void graph_delete(graph * gg) {
	graph_ * g = (graph_ *) gg;
	
	for(vertex * vertex_iter : *(g->vertex_list)) {
		free(vertex_iter);
	
	for(edge * edge_iter : *(g->edge_list))
		free(edge_iter);
}
