#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <p1/goalistAction.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>

#define pi 3.141592653589793238462643383279502884


double max_angular_speed = 1.0;											// parametri velocità
double max_linear_speed = 1.0;

double coll_distance = 0.50;
double avoid_distance = 0.60;

double robot_size = 0.65;												// grandezza robot (m)

class GoalistAction {
	enum state_1 {activated, moving, arrived};
	enum state_2 {rotating, finished};
	enum base_scan_state {on, collision, avoidance, undetected};
	
	protected:
		ros::NodeHandle nh_;
		tf::TransformListener listener_;
		actionlib::SimpleActionServer<p1::goalistAction> as_;
		std::string action_name_;
		p1::goalistResult result_;
		
		ros::Publisher cmd_vel_pub_;									// publisher per comandi di velocità
		ros::Subscriber base_scan_sub_;									// subscriber per base scan
		
		state_1 state_1_;												// stato di esecuzione della fase di movimento
		state_2 state_2_;												// stato di esecuzione della fase di rotazione finale
		base_scan_state base_scan_state_;								// stato di esecuzione della funzione di callback di base_scan
		int obstacle_direction_;
		double nearest_point_;
		
		// I seguenti due parametri servono per iterare la lista dei goal da visitare
		geometry_msgs::Point current_start_point_;						// partenza attuale
		geometry_msgs::Point current_destination_point_;				// destinazione attuale
		
		double arrived_displacement_;									// distanza euclidea fra posizione di partenza e destinazione
		
		double final_theta_;
		

	public:
		GoalistAction(std::string name) :
			as_(nh_, name, boost::bind(&GoalistAction::execute, this, _1), false),
			action_name_(name) {
			state_1_ = arrived;
			state_2_ = finished;
			base_scan_state_ = undetected;
			
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
			base_scan_sub_ = nh_.subscribe("base_scan", 10, &GoalistAction::obstacle_avoid, this);
			as_.start();
		}

		~GoalistAction(void) {
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
		
		static void mapto360(double& angle) {
			angle += pi * (5.0 / 2.0);
		}
		
		void obstacle_avoid(const sensor_msgs::LaserScan::ConstPtr& msg) {
			if(base_scan_state_ != on)
				return;
			
			obstacle_direction_ = 0;
			bool avoid = false;
			
			int size = msg->ranges.size();
			double angle_increment = msg->angle_increment;
			
			double beta = acos(coll_distance / avoid_distance);
			int beta_index = beta / angle_increment;
			
			for(int i = 0; i < beta_index; i++) {
				double alpha = i * angle_increment;
				double t = tan(alpha) * coll_distance;
				double r = sqrt(pow(t, 2.0) + pow(coll_distance, 2.0));
				
				if(msg->ranges[i] < r)
					obstacle_direction_++;
				else if(msg->ranges[i] < avoid_distance)
					avoid = true;
				
				if(msg->ranges[size - i - 1] < r)
					obstacle_direction_--;
				else if(msg->ranges[size - i - 1] < avoid_distance)
					avoid = true;
			}
			
			for(int i = beta_index; i < size / 2; i++) {
				if(msg->ranges[i] < avoid_distance)
					obstacle_direction_++;
				if(msg->ranges[size - i - 1] < avoid_distance)
					obstacle_direction_--;
			}
			
			if(obstacle_direction_ != 0)
				base_scan_state_ = collision;
			else {
				if(avoid)
					base_scan_state_ = avoidance;
				else
					base_scan_state_ = undetected;
			}
			
			
			nearest_point_ = coll_distance - (robot_size / 2.0);
			
			double alpha = 0.0;
			for(int i = 0; i < size / 4; i++) {
				double t = tan(alpha) * robot_size / 2.0;
				double r = sqrt(pow(t, 2.0) + pow(robot_size / 2.0, 2.0));
				
				if(msg->ranges[i] - r < nearest_point_)
					nearest_point_ = msg->ranges[i] - r;
				
				if(msg->ranges[size - i - 1] - r < nearest_point_)
					nearest_point_ = msg->ranges[size - i - 1] - r;
					
				alpha += angle_increment;
			}
			
			alpha = 0.0;
			for(int i = size / 2; i > size / 4; i--) {
				double t = tan(alpha) * robot_size / 2.0;
				double r = sqrt(pow(t, 2.0) + pow(robot_size / 2.0, 2.0));
				
				if(msg->ranges[i] - r < nearest_point_)
					nearest_point_ = msg->ranges[i] - r;
				
				if(msg->ranges[size - i - 1] - r < nearest_point_)
					nearest_point_ = msg->ranges[size - i - 1] - r;
					
				alpha += angle_increment;
			}
			
			/*
			double fov_ = 180.0;										// parametri per evitare ostacoli
			double fov_considered = 180.0;
			double avoid_activation_dist = 0.50;
			
			int size = msg->ranges.size();
			int half = size / 2;
			int diff = (half * fov_considered) / fov_;
			int dx = half - diff;
			int sx = half + diff;
			
			for(int i = sx; i < half; i++)
				if(msg->ranges[i] < avoid_activation_dist)
					obstacle_direction_++;
			
			for(int i = half; i < dx; i++)
				if(msg->ranges[i] < avoid_activation_dist)
					obstacle_direction_--;
			
					
			if(obstacle_direction_ == 0) {
				base_scan_state_ = undetected;
			}
			else {
				obstacle_direction_ = std::abs(obstacle_direction_) / obstacle_direction_;
				base_scan_state_ = detected;
			}
			*/
		}
		
		void actuator_1() {
			if(state_1_ == arrived)
				return;
			
			tf::StampedTransform robot_position;
			try {
				listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), robot_position);
			}
			catch(tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}
				
			/* ------- ANGOLO DESIDERATO ------- */
			double s3n = current_destination_point_.x - robot_position.getOrigin().x();
			double c0s = - current_destination_point_.y + robot_position.getOrigin().y();
			if(c0s == 0) c0s += 0.01;
			
			double desired_angle = atan(s3n / c0s);
			to360(desired_angle);
			
			if(c0s < 0)													// il codominio dell'arcotangente è [-90, 90] gradi
				if(s3n < 0)
					desired_angle += pi;
				else
					desired_angle -= pi;
			
			/* ------- ANGOLO ATTUALE ------- */
			double current_angle = tf::getYaw(robot_position.getRotation());
			mapto360(current_angle);
			to360(current_angle);
			
			/* ------- SFASAMENTO ANGOLARE ------- */
			double angular_displacement_1 = std::abs(desired_angle - current_angle);
			double angular_displacement_2 = std::abs(angular_displacement_1 - (pi * 2.0));
			double angular_displacement = !(angular_displacement_1 < angular_displacement_2)?angular_displacement_2 : angular_displacement_1;
			
			/* ------- VELOCITA' ANGOLARE ------- */
			geometry_msgs::Twist motion_msg;
			
			double angular_speed = (max_angular_speed / pi) * angular_displacement;
			
			double opp = current_angle + pi;							// calcolo angolo opposto a quello attuale
			to360(opp);
			
			if(current_angle <= pi)										// faccio prima a girarmi a sinistra o a destra?
				if(desired_angle > current_angle && desired_angle < opp)
					angular_speed = angular_speed; // a sx
				else
					angular_speed = -angular_speed; // a dx
			else
				if(desired_angle < current_angle && desired_angle > opp)
					angular_speed = -angular_speed; // a dx
				else 
					angular_speed = angular_speed; // a sx
			
			/* ------- VELOCITA' LINEARE ------- */
			double linear_speed = 0.0;
			if(angular_displacement < (pi / 6.0))
				linear_speed = (- max_angular_speed / (pi / 6.0)) * angular_displacement + max_angular_speed;
			
			/* ------- EVITARE OSTACOLI ------- */
			if(angular_displacement < (pi / 6.0)) {
				base_scan_state_ = on;
				while(base_scan_state_ == on)
					ros::spinOnce();
				
				switch(base_scan_state_) {
					case collision: {
						double m = max_angular_speed / ((robot_size / 2.0) - coll_distance);
						angular_speed = ((double) obstacle_direction_)* (m * nearest_point_ + max_angular_speed);
						
						m = max_linear_speed / (coll_distance - (robot_size / 2.0));
						linear_speed = m * nearest_point_;
						
						//std::cout << "COLLISION " << nearest_point_ << "\n";
						break;
					}
					case avoidance: {
						linear_speed = max_linear_speed / 6.0;
						//std::cout << "AVOIDANCE\n";
						break;
					}
					case undetected: {
						//std::cout << "UNDETECTED\n";
						break;
					}
					default : {}
				}
				/*
				if(base_scan_state_ == detected) {
					std::cout << "DETECTEXXXXXXXXx\n";
					linear_speed = max_angular_speed / 7.0;
					angular_speed = ((double) obstacle_direction_) * max_angular_speed;
				}
				*/
			}
			
			motion_msg.linear.x = linear_speed;
			motion_msg.angular.z = angular_speed;
			
			/* ------- CONTROLLO DELLO STATO ------- */
			switch(state_1_) {
				case activated: {
					current_start_point_.x = robot_position.getOrigin().x();
					current_start_point_.y = robot_position.getOrigin().y();
					
					arrived_displacement_ = eulerianDist(current_start_point_, current_destination_point_);
					state_1_ = moving;
					
					break;
				}
				case moving: {
					geometry_msgs::Point pose;
					pose.x = robot_position.getOrigin().x();
					pose.y = robot_position.getOrigin().y();
					
					double linear_displacement = eulerianDist(pose, current_start_point_);
					
					if(linear_displacement > arrived_displacement_) {
						std::cout << "Real x: " << robot_position.getOrigin().x() << " y: " << robot_position.getOrigin().y() << "\n";
						state_1_ = arrived;
					}
					else
						cmd_vel_pub_.publish(motion_msg);
					
					break;
				}
				case arrived: {
					break;
				}
				default: {}
			}
			
		}
		
		void actuator_2() {
			if(state_2_ == finished)
				return;
				
			tf::StampedTransform robot_position;
			try {
				listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), robot_position);
			}
			catch(tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}			
			
			/* ------- ANGOLO DESIDERATO ------- */
			double desired_angle = final_theta_;
			mapto360(desired_angle);
			to360(desired_angle);
			
			/* ------- ANGOLO ATTUALE ------- */
			double current_angle = tf::getYaw(robot_position.getRotation());
			mapto360(current_angle);
			to360(current_angle);
			
			/* ------- SFASAMENTO ANGOLARE ------- */
			double angular_displacement_1 = std::abs(desired_angle - current_angle);
			double angular_displacement_2 = std::abs(angular_displacement_1 - (pi * 2.0));
			double angular_displacement = !(angular_displacement_1 < angular_displacement_2)?angular_displacement_2 : angular_displacement_1;
			
			/* ------- VELOCITA' ANGOLARE ------- */
			geometry_msgs::Twist motion_msg;
			
			double angular_speed = (max_angular_speed / pi) * angular_displacement;
			
			double opp = current_angle + pi;							// calcolo angolo opposto a quello attuale
			to360(opp);
			
			if(current_angle <= pi)										// faccio prima a girarmi a sinistra o a destra?
				if(desired_angle > current_angle && desired_angle < opp)
					angular_speed = angular_speed; // a sx
				else
					angular_speed = -angular_speed; // a dx
			else
				if(desired_angle < current_angle && desired_angle > opp)
					angular_speed = -angular_speed; // a dx
				else 
					angular_speed = angular_speed; // a sx
			
			motion_msg.angular.z = angular_speed;
			
			/* ------- CONTROLLO DELLO STATO ------- */
			switch(state_2_) {
				case rotating: {						
					if(angular_displacement < (pi * 0.05)) {
						std::cout << "Real theta: " << getYaw(robot_position.getRotation()) << "\n";
						state_2_ = finished;
					}
					else
						cmd_vel_pub_.publish(motion_msg);
					
					break;
				}
				case finished: {
					break;
				}
				default: {}
			}
			
		}
		
		void execute(const p1::goalistGoalConstPtr& goal) {
			bool success = true;
			
			std::cout << "VISITING " << goal->goals_number << " VERTEXES\n";
			for(int i = 0; i < goal->goals_number; i++) {
				current_destination_point_ = goal->goals_list[i];
				
				std::cout << "Destination x: " << current_destination_point_.x << " y: " << current_destination_point_.y << "\n";
				
				state_1_ = activated;
				while(state_1_ != arrived)
					actuator_1();
				
				if(as_.isPreemptRequested() || !ros::ok()) {
					std::cout << action_name_.c_str() << ": Preempted\n";
					as_.setPreempted();
					success = false;
					break;
				}
			}
			
			std::cout << "\nFINAL ROTATION\n";
			final_theta_ = goal->final_theta;
			std::cout << "Final theta: " << final_theta_ << "\n";
			state_2_ = rotating;
			while(state_2_ != finished)
				actuator_2();
			
			if(as_.isPreemptRequested() || !ros::ok()) {
				std::cout << action_name_.c_str() << ": Preempted\n";
				as_.setPreempted();
				success = false;
			}
			
			if(success) {
				std::cout << action_name_.c_str() << ": Succeeded\n";
				as_.setSucceeded(result_);
			}
			
			std::cout << "__________________________________\n\n";
		}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "goalist_server");
	
	GoalistAction nav("goalist_server");
	ros::spin();

	return 0;
}
