#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <p1/laserAction.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#define pi 3.141592653589


double fov = pi;														// 180 gradi di campo visivo
double robot_size = 0.65;												// grandezza robot (m)
double mdbo = 1.2;														// distanza minima fra oggetti (min dist between objects)
double mdbv = 1.5;														// distanza minima fra vertici (min dist between vertexes)

// fixed distances (per posizionamento nuovi vertici)
double fd1 = 0.2;														// radiale
double fd2 = 0.6;														// perpendicolare al raggio


typedef struct section_s {
	int sx;
	int dx;
	int sx_index;
	int dx_index;
} section;

section * new_section() {
	section * s = (section *) malloc(sizeof(section));
	s->sx = 0;
	s->dx = 0;
}


class LaserAction {
	
	enum get_base_scan_state {on, off};
	
	protected:
	
		ros::NodeHandle nh_;
		
		laser_geometry::LaserProjection projector_;						// attributi acquisizione trasformate
		tf::TransformListener listener_;
			
		actionlib::SimpleActionServer<p1::laserAction> as_;				// attributi di server
		std::string action_name_;
		p1::laserResult result_;
		
		geometry_msgs::Point robot_position_;							// posizione robot
		tf::StampedTransform map_T_base_footprint;						// trasformata di base_footprint wrt map
		
		sensor_msgs::LaserScan laser_scan_;								// laser scan
	    sensor_msgs::PointCloud cloud_;									// laser scan in coordinate wrt map
	    
	    ros::Subscriber base_scan_sub_;									// subscriber per acquisire il laser
	    get_base_scan_state base_scan_state_;							// stato di acquisizione laser
	
	public:
	
		LaserAction(std::string name) :
			as_(nh_, name, boost::bind(&LaserAction::execute, this, _1), false),
			action_name_(name) {
			
			base_scan_state_ = off;
			base_scan_sub_ = nh_.subscribe("base_scan", 10, &LaserAction::get_base_scan, this);
			
			as_.start();
		}

		~LaserAction(void) {
		}
		
		static double eulerianDist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
			return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
		}
		
		static void to360(double& angle) {
			while(angle < 0)
				angle += 2.0 * pi;
			while(angle > 2.0 * pi)
				angle -= 2.0 * pi;
		}
		
		static void to180(double& angle) {
			while(angle < -pi)
				angle += 2.0 * pi;
			while(angle > pi)
				angle -= 2.0 * pi;
		}
		
		static void _360tomap(double& angle) {
			angle -= pi * (5.0 / 2.0);
		}
		
		double point2laser_angle(geometry_msgs::Point p) {
			// trasformata punto rispetto alla mappa
			tf::Vector3 destination;
			destination.setX(p.x);
			destination.setY(p.y);
			destination.setZ(0.0);
			tf::StampedTransform map_T_point;
			map_T_point.setOrigin(destination);
			
			// trasformata punto rispetto a base_footprint
			tf::Transform base_footprint_T_point = map_T_base_footprint.inverse().operator*(map_T_point);
			
			double s3n = base_footprint_T_point.getOrigin().x();
			double c0s = - base_footprint_T_point.getOrigin().y();
			if(c0s == 0) c0s += 0.01;
			double theta = atan(s3n / c0s);
			to360(theta);
			
			if(c0s < 0)													// il codominio dell'arcotangente è [-90, 90] gradi
				if(s3n < 0)
					theta += pi;
				else
					theta -= pi;
			
			double laser_angle = theta + (fov / 2) - (pi / 2.0);
			to360(laser_angle);
			
			if(laser_angle > fov)
				laser_angle = -1;
			
			return laser_angle;
		}
		
		void get_robot_position() {
			// "posizione del robot": trasformata di base_footprint wrt map
			
			// calcolo trasformata
			try {
				listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), map_T_base_footprint);
			}
			catch(tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}
			
			// salviamo la posizione sottoforma di Point
			robot_position_.x = map_T_base_footprint.getOrigin().x();
			robot_position_.y = map_T_base_footprint.getOrigin().y();
			robot_position_.z = 0.0;
		}
		
		void get_base_scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
			if(base_scan_state_ == off)
				return;
			
			// salvataggio del messaggio come membro della classe
			// correzione del range_max del messaggio
			laser_scan_ = *msg;
			for(int i = 0; i < laser_scan_.ranges.size(); i++)
				if(laser_scan_.ranges[i] == laser_scan_.range_max)
					laser_scan_.ranges[i] = laser_scan_.range_max - 0.1;
			
			
			if(!listener_.waitForTransform(laser_scan_.header.frame_id, "/map",
				laser_scan_.header.stamp + ros::Duration().fromSec(laser_scan_.ranges.size()*laser_scan_.time_increment),
				ros::Duration(1.0))) {
				return;
			}
						
			try {
				projector_.transformLaserScanToPointCloud("/map", laser_scan_, cloud_, listener_);
			}
			catch(tf::TransformException& e) {
				std::cout << e.what();
				return;
			}
			
			
			base_scan_state_ = off;
		}
		
		bool print_vertexes_rviz(std::vector<geometry_msgs::Point> to_print) {
			ros::NodeHandle n;
			ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

			uint32_t shape = visualization_msgs::Marker::CUBE;
			
			visualization_msgs::MarkerArray marker_array;
			marker_array.markers.resize(to_print.size());
			
			for(int i = 0; i < to_print.size(); i++) {
				marker_array.markers[i].header.frame_id = "/map";
				marker_array.markers[i].header.stamp = ros::Time::now();

				marker_array.markers[i].ns = "goalist_client";
				marker_array.markers[i].id = i;
				
				marker_array.markers[i].type = shape;
				
				marker_array.markers[i].action = visualization_msgs::Marker::ADD;
				
				marker_array.markers[i].pose.position.x = to_print[i].x;
				marker_array.markers[i].pose.position.y = to_print[i].y;
				marker_array.markers[i].pose.position.z = 0;
				marker_array.markers[i].pose.orientation.x = 0.0;
				marker_array.markers[i].pose.orientation.y = 0.0;
				marker_array.markers[i].pose.orientation.z = 0.0;
				marker_array.markers[i].pose.orientation.w = 1.0;

				marker_array.markers[i].scale.x = 0.2;
				marker_array.markers[i].scale.y = 0.2;
				marker_array.markers[i].scale.z = 0.2;
				
				marker_array.markers[i].color.r = 1.0f;
				marker_array.markers[i].color.g = 0.0f;
				marker_array.markers[i].color.b = 0.0f;
				marker_array.markers[i].color.a = 1.0;

				marker_array.markers[i].lifetime = ros::Duration();
			}
			
			while(marker_pub.getNumSubscribers() < 1) {
				if(!ros::ok())
					return false;
				
				ROS_WARN_ONCE("Please create a subscriber to the marker");
				sleep(1);
			}
			
			marker_pub.publish(marker_array);
			sleep(1);
			
			std::cout << "Visualization msg sent to Rviz\n";
			
			return true;
		}
		
		void execute(const p1::laserGoalConstPtr& goal) {				// è la funzione di callback di questo server
			std::cout << action_name_.c_str() << ": Started\n\n";
			
			bool success = true;
			
			
			// acquisizione odometria e laser_scan
			get_robot_position();
			
			base_scan_state_ = on;
			while(base_scan_state_ != off)
				ros::spinOnce();
			
			
			// inizializzazione result
			result_.new_edge_number = 0;
			std::vector<int> new_edge_list_aux;
			
			result_.new_vertex_number = 0;
			std::vector<geometry_msgs::Point> new_vertex_list_aux;
			
			result_.new_theta_number = 0;
			std::vector<double> new_theta_list_aux;
			
			
			// vertici nel campo visivo (da connettere a source)
			std::cout << "Source: " << goal->source << "\n";
			
			std::cout << "Vertexes in fov: ";
			for(int i = 0; i < goal->vertex_number; i++)
				if(i != goal->source) {
					double laser_angle = point2laser_angle(goal->vertex_list[i]);
					
					if(laser_angle != -1) {
						int laser_index = laser_angle / laser_scan_.angle_increment;
						
						if(laser_scan_.ranges[laser_index] > eulerianDist(robot_position_, goal->vertex_list[i])) {
							new_edge_list_aux.push_back(i);
							result_.new_edge_number++;					// creazione di un nuovo arco
							
							std::cout << i << " ";
						}
					}
				}
			std::cout << "\n";
			
			
			// sezionamento campo visivo (in range)
			std::vector<section *> sections;
			//std::vector<geometry_msgs::Point> to_print;
			
			section * actual_section = new_section();
			actual_section->dx = 0;
			actual_section->dx_index = 0;
			
			for(int i = 1; i < laser_scan_.ranges.size() - 2; i++) {
				double displacement = laser_scan_.ranges[i] - laser_scan_.ranges[i+1];
				
				if(std::abs(displacement) > mdbo) {
					std::cout << "DISPLACEMENT " << displacement << "\n";
					
					section * next_section = new_section();
					
					if(displacement < 0) {								// muro a destra
						actual_section->sx = 0;
						actual_section->sx_index = i - 1;
						
						next_section->dx = 1;
						next_section->dx_index = i;
						
						/*
						geometry_msgs::Point p;
						p.x = cloud_.points[i].x;
						p.y = cloud_.points[i].y;
						to_print.push_back(p);
						*/
					}
					else {												// muro a sinistra
						actual_section->sx = 1;
						actual_section->sx_index = i + 1;
						
						next_section->dx = 0;
						next_section->dx_index = i + 2;
						
						/*
						geometry_msgs::Point p;
						p.x = cloud_.points[i+1].x;
						p.y = cloud_.points[i+1].y;
						to_print.push_back(p);
						*/
						
						i++;
					}
					
					sections.push_back(actual_section);
					actual_section = next_section;
				}
			}
			
			actual_section->sx = 0;
			actual_section->sx_index = laser_scan_.ranges.size() - 1;
			sections.push_back(actual_section);
			
			//print_vertexes_rviz(to_print);
			
			std::cout << "Sections (in range): " << sections.size() << "\n";
			
			
			// calcolo nuovi vertici e theta (in range)
			sensor_msgs::LaserScan laser_scan = laser_scan_;
			sensor_msgs::PointCloud cloud;
			
			std::vector<int> new_vertex_index;
			
			for(int i = 0; i < sections.size(); i++) {
				if(sections[i]->dx == 1) {
					int index = sections[i]->dx_index;
					double r = laser_scan_.ranges[index];
					double t = r + fd1;
					double c = sqrt(pow(t, 2.0) - (pow(fd2, 2.0) / 4.0));
					double alpha = 2 * acos(c/t);
					int to_sum_index = alpha / laser_scan_.angle_increment;
					
					double sum_index = index + to_sum_index;
					if(sum_index > 1080)
						sum_index = 1080;
						
					laser_scan.ranges[sum_index] = t;
					new_vertex_index.push_back(sum_index);
				}
				if(sections[i]->sx == 1) {
					int index = sections[i]->sx_index;
					double r = laser_scan_.ranges[index];
					double t = r + fd1;
					double c = sqrt(pow(t, 2.0) - (pow(fd2, 2.0) / 4.0));
					double alpha = 2 * acos(c/t);
					int to_sub_index = alpha / laser_scan_.angle_increment;
					
					double sub_index = index - to_sub_index;
					if(sub_index < 0)
						sub_index = 0;
					
					laser_scan.ranges[sub_index] = t;
					new_vertex_index.push_back(sub_index);
				}
			}
			std::cout << "Possible new border vertexes (in range): " << new_vertex_index.size() << "\n";
			
			
			// traduzione nuovi vertici wrt map (in range)
			try {
				projector_.transformLaserScanToPointCloud("/map", laser_scan, cloud, listener_);
			}
			catch(tf::TransformException& e) {
				std::cout << e.what();
				return;
			}
			
			geometry_msgs::Point32 p_aux;
			geometry_msgs::Point p;
			
			int k = 0;
			for(int i = 0; i < sections.size(); i++) {
				if(sections[i]->dx == 1) {
					int index = new_vertex_index[k++];
					p_aux = cloud.points[index];
					p.x = p_aux.x; p.y = p_aux.y;
					
					bool aborted = false;
					for(int j = 0; j < goal->vertex_number && !aborted; j++)
						if(j != goal->source && eulerianDist(p, goal->vertex_list[j]) < mdbv)
							aborted = true;								// un nuovo "vertice candidato" viene cancellato
																		// se sta "vicino" ad un qualunque altro vertice
					
					if(!aborted) {
						new_vertex_list_aux.push_back(p);
						result_.new_vertex_number++;
						
						double s3n = p.x - robot_position_.x;
						double c0s = - p.y + robot_position_.y;
						if(c0s == 0) c0s += 0.01;
						
						double theta = atan(s3n / c0s);
						to360(theta);
						
						if(c0s < 0)										// il codominio dell'arcotangente è [-90, 90] gradi
							if(s3n < 0)
								theta += pi;
							else
								theta -= pi;
						
						_360tomap(theta);
						to180(theta);
						
						theta -= pi / 2.0;
						
						new_theta_list_aux.push_back(theta);
						result_.new_theta_number++;
					}
				}
				if(sections[i]->sx == 1) {
					int index = new_vertex_index[k++];
					p_aux = cloud.points[index];
					p.x = p_aux.x; p.y = p_aux.y;
					
					bool aborted = false;
					for(int j = 0; j < goal->vertex_number && !aborted; j++)
						if(j != goal->source && eulerianDist(p, goal->vertex_list[j]) < mdbv)
							aborted = true;								// un nuovo "vertice candidato" viene cancellato
																		// se sta "vicino" ad un qualunque altro vertice
					
					if(!aborted) {
						new_vertex_list_aux.push_back(p);
						result_.new_vertex_number++;
						
						double s3n = p.x - robot_position_.x;
						double c0s = - p.y + robot_position_.y;
						if(c0s == 0) c0s += 0.01;
						
						double theta = atan(s3n / c0s);
						to360(theta);
						
						if(c0s < 0)										// il codominio dell'arcotangente è [-90, 90] gradi
							if(s3n < 0)
								theta += pi;
							else
								theta -= pi;
						
						_360tomap(theta);
						to180(theta);
						
						theta += pi / 2.0;
						
						new_theta_list_aux.push_back(theta);
						result_.new_theta_number++;
					}
				}
			}
			
			
			// sezionamento campo visivo (out range)
			std::vector<section *> out_range_sections;
			//std::vector<geometry_msgs::Point> to_print;
			
			for(int i = 0; i < laser_scan_.ranges.size() - 1; i++) {
			
				while(i < laser_scan_.ranges.size() - 1 && laser_scan_.ranges[i] < laser_scan_.range_max - 0.1)
					i++;
				
				if(i < laser_scan_.ranges.size() - 1) {
					section * next_section = new_section();
					next_section->dx_index = i;
					
					while(i < laser_scan_.ranges.size() - 1 && !(laser_scan_.ranges[i] < laser_scan_.range_max - 0.1))
						i++;
						
					next_section->sx_index = i - 1;
					out_range_sections.push_back(next_section);
					
					/*
					geometry_msgs::Point p;
					p.x = cloud_.points[i].x;
					p.y = cloud_.points[i].y;
					to_print.push_back(p);
					*/
				}
			}
			
			//print_vertexes_rviz(to_print);
			
			std::cout << "Sections (out range): " << out_range_sections.size() << "\n";
			
			
			// calcolo nuovi vertici e theta (out range)
			laser_scan = laser_scan_;
			new_vertex_index.clear();
			
			for(int i = 0; i < out_range_sections.size(); i++) {
				int index = (out_range_sections[i]->dx_index + out_range_sections[i]->sx_index) / 2;
					
				laser_scan.ranges[index] = 3.0;
				new_vertex_index.push_back(index);
			}
			std::cout << "Possible new border vertexes (out range): " << new_vertex_index.size() << "\n";
			
			
			// traduzione nuovi vertici wrt map (out range)
			try {
				projector_.transformLaserScanToPointCloud("/map", laser_scan, cloud, listener_);
			}
			catch(tf::TransformException& e) {
				std::cout << e.what();
				return;
			}
			
			for(int i = 0; i < out_range_sections.size(); i++) {
				if(out_range_sections[i]->sx_index - out_range_sections[i]->dx_index > 50) {
					int index = new_vertex_index[i];
					p_aux = cloud.points[index];
					p.x = p_aux.x; p.y = p_aux.y;
					
					bool aborted = false;
					for(int j = 0; j < goal->vertex_number && !aborted; j++)
						if(j != goal->source && eulerianDist(p, goal->vertex_list[j]) < mdbv)
							aborted = true;									// un nuovo "vertice candidato" viene cancellato
																			// se sta "vicino" ad un qualunque altro vertice
					
					if(!aborted) {
						new_vertex_list_aux.push_back(p);
						result_.new_vertex_number++;
						
						double s3n = p.x - robot_position_.x;
						double c0s = - p.y + robot_position_.y;
						if(c0s == 0) c0s += 0.01;
						
						double theta = atan(s3n / c0s);
						to360(theta);
						
						if(c0s < 0)											// il codominio dell'arcotangente è [-90, 90] gradi
							if(s3n < 0)
								theta += pi;
							else
								theta -= pi;
						
						_360tomap(theta);
						to180(theta);
						
						new_theta_list_aux.push_back(theta);
						result_.new_theta_number++;
					}
				}
			}
			
			
			// set result
			std::cout << "New vertexes added to graph: " << result_.new_vertex_number << "\n";
			std::cout << "Old vertexes connected to source: " << result_.new_edge_number << "\n\n";
			
			result_.new_edge_list = new_edge_list_aux;
			result_.new_vertex_list = new_vertex_list_aux;
			result_.new_theta_list = new_theta_list_aux;
			
			
			// rimozione sezioni
			for(int i = 0; i < sections.size(); i++)
				free(sections[i]);
			for(int i = 0; i < out_range_sections.size(); i++)
				free(out_range_sections[i]);
			
			
			// stampa successo/fallimento
			if(success) {
				std::cout << action_name_.c_str() << ": Succeeded\n";
				as_.setSucceeded(result_);
			}
			else
				std::cout << action_name_.c_str() << ": Failed\n";
			
			std::cout << "__________________________________\n\n";
		}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "laser_server");
	
	LaserAction ls("laser_server");
	ros::spin();

	return 0;
}
