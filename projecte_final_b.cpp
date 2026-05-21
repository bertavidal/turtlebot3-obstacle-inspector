#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>



// Variables globals.
int estat = 0; // estats: moviment = 0, detecció = 1, paret = 2, conegut = 3, nou = 4

bool aprop = false; // flag per identificar si hi ha un obstacle davant del robot

double dist = 0.0; // distància mesurada al scanCallback
double angle = 0.0; // angle mesurat al scanCallback

int index_obstacle = -1; // índex del rang del laser on s'ha detectat un obstacle al scanCallback

float error = 0.3; // rang d'error per la identificació d'obstacles 

bool scan_classificat = false;  // flag que indica que hi ha una classificació nova
bool scan_paret = false; // flag, resultat de la classificació del làser


struct Obstacle { double x,y; };
std::vector<Obstacle> obstacles_coneguts; // vector que guarda la posició dels obstacles inpeccionats



geometry_msgs::Pose2D current_pose;
ros::Publisher pub_pose2d;

geometry_msgs::Twist velocitat;


void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // angular position
    current_pose.theta = yaw;
    pub_pose2d.publish(current_pose);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    aprop = false;
    index_obstacle = -1;
    
    for (int j = -10; j <= 10; j++) {  // mirem angles frontals del 350-359 i del 0-10
	int i = j;
	if (i < 0) i += 360;
    	
	if (!aprop) {
            if ((msg->ranges[i]) < 0.65 && (msg->ranges[i]) != 0) {
                aprop = true;
                index_obstacle = i;
                dist = (msg->ranges[i]);
                angle = (msg->angle_min) + i * (msg->angle_increment);
            }
        }
    }

    // classificació de parets vs obstacles
    if (estat == 1 && aprop) {
    	int comptador = 0;
    	bool paret = false;
    	
        // mirem angle de 50 graus centrat en el punt on s'ha detectat l'obstacle/paret
        for (int j = index_obstacle - 25; j <= index_obstacle + 25; ++j) {
            int i = j;
            if (i < 0) i += 360;
            else if (i >= 360) i -= 360;
      
            if (msg->ranges[i] < 1.5 && (msg->ranges[i]) != 0) {
            	comptador += 1;
            	}
	}
	
	if (comptador >= 30) paret = true;

        scan_paret = paret;
        scan_classificat = true;
    }
}

int main(int argc, char **argv) {

    const double PI = 3.14159265358979323846;

    ros::init(argc, argv, "node_berta");
    ros::NodeHandle nh;
    
    ros::Subscriber sub_scan = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::Subscriber sub_odom = nh.subscribe("odom", 1000, odomCallback);
    
    pub_pose2d = nh.advertise<geometry_msgs::Pose2D>("turtlebot_pose2d", 1000);

    // velocitats inicials
    ros::Rate loop_rate(10);
     
    double obs_x = 0.0, obs_y = 0.0; 

    while (ros::ok())
    {
        if (estat == 0) { // robot en moviment. explora fins a detectar un nou obstacle
            if (aprop) {
                velocitat.linear.x = 0;
                estat = 1;
                scan_classificat = false;
            } else {
            velocitat.angular.z = 0;
            velocitat.linear.x = 0.3;
            }
        }
	    
        if (estat == 1) { // robot aturat, actua segons el tipus d'obstacle detectat
            
            if (scan_classificat) {
                
                if (scan_paret){ // paret
                    estat = 2;
                    
                }
                else { // obstacle
                
                    // càlcul de la posició global de l'obstacle 
                    obs_x = current_pose.x + dist * cos(current_pose.theta + angle);
                    obs_y = current_pose.y + dist * sin(current_pose.theta + angle);
                    
                    // classificació obstacle conegut vs nou
                    bool conegut = false;
                    for (int i = 0; i < obstacles_coneguts.size(); i++) {
                        double dx = obs_x - obstacles_coneguts[i].x;
                        double dy = obs_y - obstacles_coneguts[i].y;
                        double dist = sqrt(dx*dx + dy*dy);
                    
                        if (dist < error) {
                            conegut = true;
                            break;
                            }
                        }

                    if (conegut) { // conegut
                        estat = 3;
                        ROS_INFO("Evitant obstacle conegut");
                    }
                    else { // nou 
                        estat = 4;
                    }
                    scan_classificat = false;
		}
	    }
        }
        
        if (estat == 2 || estat == 3) { // robot evitant paret o obstacle conegut
        
            velocitat.angular.z = 1; // gir a l'esquerra fins a poder avançar
            
            if (!aprop) {
                velocitat.angular.z = 0;
                velocitat.linear.x = 0.3;
                estat = 0;
            }
        }
        
        if (estat == 4) { // robot inspecciona un obstacle nou
        
            // guardar posició de l'obstacle nou al vector
            Obstacle ob;
            ob.x = obs_x;
            ob.y = obs_y;
            obstacles_coneguts.push_back(ob);
            ROS_INFO("Nou obstacle detectat a x:%.3f, y=%.3f", ob.x, ob.y);

            // inspecció (volta de 360 sobre si mateix)
            double angle_anterior = current_pose.theta;
            double angle_actual = angle_anterior;
            double angle_girat = 0;

            velocitat.angular.z = 1;

            while (ros::ok() && angle_girat < (2*PI - PI/6)) {
                pub_vel.publish(velocitat);
                ros::spinOnce();
                loop_rate.sleep();

                angle_actual = current_pose.theta;
                double gir = angle_actual - angle_anterior;
                if (gir > PI) gir -= 2*PI;
                else if (gir < -PI) gir += 2*PI;
                
                angle_girat += fabs(gir);
                angle_anterior = angle_actual;
                
	    }
	    
            velocitat.angular.z = 0;
            velocitat.linear.x = 0.3;
            estat = 0;
        }
        
        pub_vel.publish(velocitat);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}