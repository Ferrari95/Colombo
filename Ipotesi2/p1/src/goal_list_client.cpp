#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <p1/goalistAction.h>
#include <p1/laserAction.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <functional>
#include <queue>
#include <vector>
#include <iostream>

#include <math.h>

#include <p1/graph.cpp>

#include <move_base_msgs/MoveBaseAction.h>


void shortest_path(graph * gg, vertex * source) {
	graph_ * g = (graph_ *) gg;
	
	std::priority_queue<vertex *, std::vector<vertex *>, std::function<bool(vertex *, vertex *)>> q(vertex_pq_comparator);
	q.push(source);
	
	for(vertex * vertex_iter : *(g->vertex_list)) {
		vertex_ * v = (vertex_ *) vertex_iter;
		
		if(v == source)
			v->source_distance = 0.0;
		else
			v->source_distance = 1000000.0;
		
		v->next = NULL;
	}
	
	while(!q.empty()) {		
        vertex_ * u = (vertex_ *) q.top();
        q.pop();
        
        for(edge * edge_iter : *(u->adjacency_list)) {
			edge_ * e = (edge_ *) edge_iter;
			
			vertex_ * z = (vertex_ *) opposite(u, e);
			double r = u->source_distance + e->weight;
			
			if(r < z->source_distance) {
				z->source_distance = r;
				z->next = u;
				q.push(z);
			}
		}
    }
    
    std::cout << "Shortest path computed\n\n";
}


bool move_to(graph * gg, vertex * vv) {
	actionlib::SimpleActionClient<p1::goalistAction> ac("goalist_server", true);

	std::cout << "Waiting for goals list server to start\n";
	ac.waitForServer();
	
	p1::goalistGoal goal;
	goal.goals_number = 0;
	
	std::vector<geometry_msgs::Point> * goals_aux;
	goals_aux = new std::vector<geometry_msgs::Point>;
	
	vertex_ * vertex_iter = (vertex_*) vv;
	
	goal.final_theta = vertex_iter->theta;
	
	while(vertex_iter != NULL) {
		geometry_msgs::Point p;
		p.x = vertex_iter->x;
		p.y = vertex_iter->y;
		goals_aux->push_back(p);
		goal.goals_number++;
		vertex_iter = (vertex_ *) vertex_iter->next;
	}
	
	goal.goals_number--;
	if(!goals_aux->empty())
		goals_aux->pop_back();
	
	std::reverse(goals_aux->begin(), goals_aux->end());
	
	goal.goals_list = *goals_aux;
	
	std::cout << "Goals list server started, sending goal\n";
	ac.sendGoal(goal);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(100.0));
	free(goals_aux);
	
	if(finished_before_timeout) {
		graph_ * g = (graph_ *) gg;
		vertex_iter = (vertex_*) vv;
		if(vertex_iter->state == border) {
			vertex_iter->state = inner;
			g->border_vertex_number--;
		}
		actionlib::SimpleClientGoalState state = ac.getState();
		std::cout << "Action finished: " << state.toString().c_str() << "\n\n";
		return true;
	}
	else
		std::cout << "Action did not finish before the time out\n\n";
	
	return false;
}

/*
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool move_to(graph * gg, vertex * vv) {
	std::vector<geometry_msgs::Point> goals_aux;
	
	vertex_ * vertex_iter = (vertex_*) vv;
	
	double final_theta = vertex_iter->theta;
	
	while(vertex_iter != NULL) {
		geometry_msgs::Point p;
		p.x = vertex_iter->x;
		p.y = vertex_iter->y;
		goals_aux.push_back(p);
		vertex_iter = (vertex_ *) vertex_iter->next;
	}
	
	if(!goals_aux.empty())
		goals_aux.pop_back();
	
	std::reverse(goals_aux.begin(), goals_aux.end());
	
	//////
	std::cout << "HEREEEEEEEEEEEEEEEEEEe\n";
	MoveBaseClient ac("move_base", true);
	
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	bool finished_before_timeout = true;
	
	for(int i = 0; i < goals_aux.size(); i++) {
		move_base_msgs::MoveBaseGoal goal;
		
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		
		goal.target_pose.pose.position.x = goals_aux[i].x;
		goal.target_pose.pose.position.y = goals_aux[i].y;
		
		tf::Quaternion q;
		q.setRPY(0.0, 0.0, final_theta);
		tf::quaternionTFToMsg(q, goal.target_pose.pose.orientation);
		
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		finished_before_timeout &= ac.waitForResult(ros::Duration(100.0));
	}
	std::cout << "HEREEEEEEEEEEEEEEEEEEMMMMMMMMM\n";
	
	if(finished_before_timeout) {
		graph_ * g = (graph_ *) gg;
		vertex_iter = (vertex_*) vv;
		if(vertex_iter->state == border) {
			vertex_iter->state = inner;
			g->border_vertex_number--;
		}
		actionlib::SimpleClientGoalState state = ac.getState();
		std::cout << "Action finished: " << state.toString().c_str() << "\n\n";
		return true;
	}
	else
		std::cout << "Action did not finish before the time out\n\n";
	
	return false;
}
*/

void graph_update(graph * gg, vertex * source, actionlib::SimpleActionClient<p1::laserAction> &ac) {
	graph_ * g = (graph_ *) gg;
	vertex_ * s = (vertex_ *) source;
	
	p1::laserResult result = *ac.getResult();
	
	for(int i = 0; i < result.new_edge_number; i++) {					// aggiunta nuovi archi
		int vertex_id = result.new_edge_list[i];
		vertex_ * v = (vertex_ *) (*(g->vertex_list))[vertex_id];
		add_edge(g, s, v);
	}
	
	for(int i = 0; i < result.new_vertex_number; i++) {					// aggiunta nuovi vertici e i relativi archi
		double x = result.new_vertex_list[i].x;
		double y = result.new_vertex_list[i].y;
		double theta = result.new_theta_list[i];
		vertex_ * v = (vertex_ *) add_vertex(g, x, y, theta, border);
		
		add_edge(g, s, v);
	}
}

bool print_vertexes_rviz(graph * gg) {
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	
	graph_ * g = (graph_ *) gg;

	uint32_t shape = visualization_msgs::Marker::CUBE;
	
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.resize(g->vertex_list->size());
	
	for(int i = 0; i < g->vertex_list->size(); i++) {
		marker_array.markers[i].header.frame_id = "/map";
		marker_array.markers[i].header.stamp = ros::Time::now();

		marker_array.markers[i].ns = "goalist_client";
		marker_array.markers[i].id = i;
		
		marker_array.markers[i].type = shape;
		
		marker_array.markers[i].action = visualization_msgs::Marker::ADD;
		
		vertex_ * vertex_iter = (vertex_ *) ((*(g->vertex_list))[i]);
		marker_array.markers[i].pose.position.x = vertex_iter->x;
		marker_array.markers[i].pose.position.y = vertex_iter->y;
		marker_array.markers[i].pose.position.z = 0;
		marker_array.markers[i].pose.orientation.x = 0.0;
		marker_array.markers[i].pose.orientation.y = 0.0;
		marker_array.markers[i].pose.orientation.z = 0.0;
		marker_array.markers[i].pose.orientation.w = 1.0;

		marker_array.markers[i].scale.x = 0.2;
		marker_array.markers[i].scale.y = 0.2;
		marker_array.markers[i].scale.z = 0.2;

		if(vertex_iter->state == border) {
			marker_array.markers[i].color.r = 0.0f;
			marker_array.markers[i].color.g = 1.0f;
			marker_array.markers[i].color.b = 0.0f;
			marker_array.markers[i].color.a = 1.0;
		}
		else {
			marker_array.markers[i].color.r = 1.0f;
			marker_array.markers[i].color.g = 0.0f;
			marker_array.markers[i].color.b = 0.0f;
			marker_array.markers[i].color.a = 1.0;
		}

		marker_array.markers[i].lifetime = ros::Duration();
	}
	
	while (marker_pub.getNumSubscribers() < 1) {
		if (!ros::ok())
			return false;
		
		ROS_WARN_ONCE("Please create a subscriber to the marker");
		sleep(1);
	}
	
	marker_pub.publish(marker_array);
	sleep(1);
	
	std::cout << "Visualization msg sent to Rviz\n";
	
	return true;
}


bool observe(graph * gg, vertex * source) {
	actionlib::SimpleActionClient<p1::laserAction> ac("laser_server", true);
	
	std::cout << "Waiting for laser server to start\n";
	ac.waitForServer();
	
	graph_ * g = (graph_ *) gg;
	vertex_ * s = (vertex_ *) source;
	
	std::cout << "Source: " << s->id << "\n";
	
	p1::laserGoal goal;
	
	goal.source = s->id;
	
	goal.vertex_number = g->vertex_list->size();
	
	std::vector<geometry_msgs::Point> * vertex_list_aux;
	vertex_list_aux = new std::vector<geometry_msgs::Point>;
	
	for(vertex * vertex_iter : *(g->vertex_list)) {
		vertex_ * v = (vertex_ *) vertex_iter;
		geometry_msgs::Point p;
		p.x = v->x;
		p.y = v->y;
		vertex_list_aux->push_back(p);
	}
	goal.vertex_list = *vertex_list_aux;
	
	std::cout << "Laser server started, sending goals\n";
	ac.sendGoal(goal);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(100.0));
	free(vertex_list_aux);
	
	if(finished_before_timeout) {
		graph_update(g, s, ac);
		std::cout << "Graph updated, " << g->vertex_list->size() << " vertexes, " << g->border_vertex_number << " border vertexes\n";
		
		actionlib::SimpleClientGoalState state = ac.getState();
		std::cout << "Action finished: " << state.toString().c_str() << "\n\n";
		return true;
	}
	else
		std::cout << "Action did not finish before the time out\n\n";
	
	return false;
}

vertex * select_next_border_vertex(graph * gg, vertex * source) {
	// DISTANZA DALLA DESTINAZIONE
	/*
	graph_ * g = (graph_ *) gg;
	vertex_ * next_vertex = NULL;
	double min_distance = 1000000.0;
	for(vertex * vertex_iter : *(g->vertex_list)) {
		vertex_ * v = (vertex_ *) vertex_iter;
		if(v->state == border && v->source_distance < 1000000.0)
			if(v->destination_distance < min_distance) {
				next_vertex = v;
				min_distance = v->destination_distance;
			}
	}
	return next_vertex;
	*/
	
	// DISTANZA DALL'ORIGINE
	graph_ * g = (graph_ *) gg;
	
	vertex_ * dest = (vertex_ *) g->destination;
	if(dest->source_distance < 1000000)
		return dest;
	
	vertex_ * next_vertex = NULL;
	double min_rate = 1000000.0;
	for(vertex * vertex_iter : *(g->vertex_list)) {
		if(vertex_iter != source && vertex_iter != dest) {
			vertex_ * v = (vertex_ *) vertex_iter;
			
			if(v->state == border && v->source_distance < min_rate) {
				next_vertex = v;
				min_rate = v->source_distance;
			}
		}
	}
	
	// MISTO
	/*
	graph_ * g = (graph_ *) gg;
	
	vertex_ * dest = (vertex_ *) g->destination;
	if(dest->source_distance < 1000000)
		return dest;
	
	vertex_ * next_vertex = NULL;
	double min_rate = 1000000.0;
	for(vertex * vertex_iter : *(g->vertex_list)) {
		if(vertex_iter != source && vertex_iter != dest) {
			vertex_ * v = (vertex_ *) vertex_iter;
			double rate = v->source_distance * v->destination_distance;
			
			if(v->state == border && rate < min_rate) {
				next_vertex = v;
				min_rate = rate;
			}
		}
	}
	*/
	
	if(next_vertex != NULL)
		std::cout << "Next border vertex: " << next_vertex->id << "\n";
	else
		std::cout << "There is no reachable border vertex\n";
	
	return next_vertex;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "goalist_client");
	ros::NodeHandle n;
	
	bool success = true;
	
	graph_ * g = (graph_ *) new_graph(20.0, 30.0, 0.0);
	vertex_ * source = (vertex_ *) add_vertex(g, 0.0, 0.0, 3.14, border);
    
	success &= observe(g, source);
	success &= print_vertexes_rviz(g);
	//print_graph(g);
	
	success &= move_to(g, source);
	success &= observe(g, source);
	success &= print_vertexes_rviz(g);
	
	shortest_path(g, source);
	
	vertex_ * next_vertex = (vertex_ *) select_next_border_vertex(g, source);
	
	std::cout << "__________________________________\n\n";
	
	while(source != g->destination && next_vertex != NULL && success) {
		success &= move_to(g, next_vertex);
		source = next_vertex;
		
		success &= observe(g, source);
		success &= print_vertexes_rviz(g);
		//print_graph(g);
		
		shortest_path(g, source);
		next_vertex = (vertex_ *) select_next_border_vertex(g, source);
		
		std::cout << "__________________________________\n\n";
	}
	
	if(success) {
		if(source == g->destination)
			std::cout << "Destination reached!\n";
		else
			std::cout << "Destination unreacheable\n";
	}
	else
		std::cout << "Server error\n";
	
	graph_delete(g);
	
	return 0;
}
