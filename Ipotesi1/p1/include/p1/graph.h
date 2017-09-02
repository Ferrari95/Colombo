typedef void graph;
typedef void vertex;
typedef void edge;

typedef enum {border, inner} vertex_type;

graph * new_graph(double destination_x, double destination_y, double destination_theta);

vertex * add_vertex(graph * gg, double x, double y, double theta, vertex_type s);

bool equals_vertex(vertex * vv1, vertex * vv2);

bool equals_edge(edge * ee1, edge * ee2);

edge * add_edge(graph * gg, vertex * vv1, vertex * vv2);

vertex * opposite(vertex * vv, edge * ee);

void print_vertex(vertex * vv);

void print_edge(edge * ee);

void print_graph(graph * gg);

bool vertex_pq_comparator(vertex * vv1, vertex * vv2);

void graph_delete(graph * g);
