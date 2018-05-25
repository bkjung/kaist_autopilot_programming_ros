
// state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <kaist_autopilot_programming_ros/purePursuit.h>
#include <kaist_autopilot_programming_ros/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>



///////////////// added
#include <stdio.h>


//map spec
cv::Mat map;
cv::Mat dynamic_map; //use this variable at dynamic mapping
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

double min_distance=0.8;			//threshold distance to detect obstacle

double obstacle_threshold_angle = 90.0;


//way points
std::vector<point> waypoints;

//path
std::vector<point> path_RRT;

//robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

//point cloud data from kinect
pcl::PointCloud<pcl::PointXYZ> point_cloud;

//FSM state
int state;

//function definition
bool isCollision();
void dynamic_mapping();
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",1,callback_state);
    ros::Subscriber gazebo_kinect_sub = n.subscribe("/camera/depth/points",1,callback_points);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map
    char* user = getlogin();
    map = cv::imread((std::string("/home/")+
                      std::string(user)+
                      std::string("/catkin_ws/src/kaist_autopilot_programming_ros/src/23106.jpg")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;
    res = 0.05;
    printf("Load map\n");

    dynamic_map = map.clone();			//copy map to dynamic_map

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    control ctrl_pp;
    purePursuit pure_pursuit;
    int look_ahead_idx;
    point look_ahead_point;
    double look_ahead_distance = 0.8;		//pure pursuit _ look ahead distance
    
    bool obstacle_avoid = false;
    bool rotation_clockwise = false;
    int stop_wait = 0;



    ros::Rate control_rate(10);

    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;

            //visualize path
            for(int i = 0; i < path_RRT.size(); i++){
                gazebo_msgs::SpawnModel model;
                model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
                        std::string("<link name=\"ball\">") +
                        std::string("<inertial>") +
                        std::string("<mass value=\"1.0\" />") +
                        std::string("<origin xyz=\"0 0 0\" />") +
                        std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
                        std::string("</inertial>") +
                        std::string("<visual>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</visual>") +
                        std::string("<collision>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0\"/>") +
                        std::string("</geometry>") +
                        std::string("</collision>") +
                        std::string("</link>") +
                        std::string("<gazebo reference=\"ball\">") +
                        std::string("<mu1>10</mu1>") +
                        std::string("<mu2>10</mu2>") +
                        std::string("<material>Gazebo/Blue</material>") +
                        std::string("<turnGravityOff>true</turnGravityOff>") +
                        std::string("</gazebo>") +
                        std::string("</robot>");

                std::ostringstream ball_name;
                ball_name << i;
                model.request.model_name = ball_name.str();
                model.request.reference_frame = "world";
                model.request.initial_pose.position.x = path_RRT[i].x;
                model.request.initial_pose.position.y = path_RRT[i].y;
                model.request.initial_pose.position.z = 0.7;
                model.request.initial_pose.orientation.w = 0.0;
                model.request.initial_pose.orientation.x = 0.0;
                model.request.initial_pose.orientation.y = 0.0;
                model.request.initial_pose.orientation.z = 0.0;

                gazebo_spawn.call(model);

                ros::spinOnce();
                ros::Rate(10).sleep();
            }
            printf("Spawn path\n");

            //initialize robot position
            geometry_msgs::Pose model_pose;
            model_pose.position.x = waypoints[0].x;
            model_pose.position.y = waypoints[0].y;
            model_pose.position.z = 0.3;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 0.0;

            geometry_msgs::Twist model_twist;
            model_twist.linear.x = 0.0;
            model_twist.linear.y = 0.0;
            model_twist.linear.z = 0.0;
            model_twist.angular.x = 0.0;
            model_twist.angular.y = 0.0;
            model_twist.angular.z = 0.0;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = "RosAria";
            modelstate.reference_frame = "world";
            modelstate.pose = model_pose;
            modelstate.twist = model_twist;

            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;

            gazebo_set.call(setmodelstate);
            ros::spinOnce();
            ros::Rate(0.33).sleep();
            printf("Initialize ROBOT\n");

/*
	printf("point cloud size = %d\n", point_cloud.size());

	FILE *out;


  	out = fopen("test.txt", "w");

	for(int i = 0; i < point_cloud.size(); i++){
		
		if(point_cloud[i].x==point_cloud[i].x && point_cloud[i].y==point_cloud[i].y && point_cloud[i].z==point_cloud[i].z) {
			//printf("x: %f, y: %f, z: %f\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z);
			fprintf(out, "x: %f, y: %f, z: %f\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z); 
		}
	}


	fclose(out);

*/
            state = RUNNING;
	//state = FINISH;
        } break;

        case RUNNING: {
            //TODO
            /*
             * copy your code from previous project2
             */
		if(stop_wait<20 && stop_wait>0) {		//totally-stop wating
			setcmdvel(0,0);
			cmd_vel_pub.publish(cmd_vel);
			stop_wait++;
		}

		else if(obstacle_avoid == true) {
			double dx=path_RRT[look_ahead_idx].x-robot_pose.x;
			double dy=path_RRT[look_ahead_idx].y-robot_pose.y;
			double l = sqrt(dx*dx+dy*dy);
			double path_th = atan2(path_RRT[look_ahead_idx].y-robot_pose.y,path_RRT[look_ahead_idx].x-robot_pose.x);
			double dth = path_th - robot_pose.th;
			if (dth > M_PI)
				dth -= M_PI*2;
			else if(dth <= -M_PI) 
				dth += M_PI*2;
			
			if(std::abs(dth)<10*M_PI/180) {
				obstacle_avoid = false;
			}
			else {
				setcmdvel(0, (rotation_clockwise == true? 20*M_PI/180:-20*M_PI/180));
				cmd_vel_pub.publish(cmd_vel);
			}
		}

		else if(isCollision()==true) {			
			if(stop_wait == 20) {		//waiting finished
				setcmdvel(0,0);
				cmd_vel_pub.publish(cmd_vel);
				stop_wait = 0;
				rotation_clockwise = (rotation_clockwise == true? false:true);
				obstacle_avoid = true;
				state = PATH_PLANNING;	//go to dynamic mapping
			}
			
			else{				//obstacle just detected
				setcmdvel(0,0);
				cmd_vel_pub.publish(cmd_vel);
				stop_wait = 1;
			}
		}

		else {		//normal running

			//obstacle_avoid = false;

			if ( ( pow(robot_pose.x-path_RRT[look_ahead_idx].x,2) + pow(robot_pose.y-path_RRT[look_ahead_idx].y,2) ) <= pow(0.2,2) ) {
				if( look_ahead_idx == path_RRT.size() - 1 ) {
					state = FINISH;
				}
				else {
					look_ahead_idx ++;
			
					for(int i = 0; i<30; i++){
						look_ahead_point.x = path_RRT[look_ahead_idx].x + (double)i/10*(path_RRT[look_ahead_idx-1].x - path_RRT[look_ahead_idx].x);
						look_ahead_point.y = path_RRT[look_ahead_idx].y + (double)i/10*(path_RRT[look_ahead_idx-1].y - path_RRT[look_ahead_idx].y);
						if( pow(robot_pose.x-look_ahead_point.x,2) + pow(robot_pose.y-look_ahead_point.y,2)  <= pow(look_ahead_distance,2) ) {
							break;
						}
					}

					printf("current point: %f, %f   next point: %f, %f\n", robot_pose.x, robot_pose.y, path_RRT[look_ahead_idx].x, path_RRT[look_ahead_idx].y);	
					ctrl_pp = pure_pursuit.get_control(robot_pose, look_ahead_point);
					setcmdvel(ctrl_pp.v,ctrl_pp.w);
					cmd_vel_pub.publish(cmd_vel);
				}
			}

			else{

				for(int i = 0; i<30; i++){
					look_ahead_point.x = path_RRT[look_ahead_idx].x + (double)i/10*(path_RRT[look_ahead_idx-1].x - path_RRT[look_ahead_idx].x);
					look_ahead_point.y = path_RRT[look_ahead_idx].y + (double)i/10*(path_RRT[look_ahead_idx-1].y - path_RRT[look_ahead_idx].y);
					if( pow(robot_pose.x-look_ahead_point.x,2) + pow(robot_pose.y-look_ahead_point.y,2) <= pow(look_ahead_distance,2) ) {
						break;
					}
				}
					 
				ctrl_pp = pure_pursuit.get_control(robot_pose, look_ahead_point);
				setcmdvel(ctrl_pp.v, ctrl_pp.w);
				cmd_vel_pub.publish(cmd_vel);
			}
		}



            /*
             * add transition part from RUNNING to PATH_PLANNING
             * when meeting obstacle
             */

		ros::spinOnce();
		control_rate.sleep();
		
        } break;

        case PATH_PLANNING: {

		printf("path planning begin\n");

		dynamic_mapping();

		printf("dynamic mapping done\n");



//////////////////////////////IT SHOULD BE CHECKED AGAIN !!!!!!!!!!!!!!!!



		if( look_ahead_idx == path_RRT.size() - 1 ) {		//when path_RRT[look_ahead_idx] is the goal point
			//rrtTree tree = rrtTree(robot_pose, path_RRT[look_ahead_idx], dynamic_map, map_origin_x, map_origin_y, res, 5);		//make a new path from current position to goal point
			rrtTree tree = rrtTree(path_RRT[look_ahead_idx-2], path_RRT[look_ahead_idx], dynamic_map, map_origin_x, map_origin_y, res, 10);	////margin 5
	
			int iteration = tree.generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 5000, 0.5);
		
			std::vector<point> tmp = tree.backtracking();

			tree.visualizeTree(tmp);		//////public function call


			int size = tmp.size();

			std::vector<point> tmp_reverse;

			for(int j=0; j<size; j++) {
				tmp_reverse.push_back(tmp[size-j-1]);
			}

			std::vector<point>::iterator it = path_RRT.begin();

			path_RRT.erase (it+look_ahead_idx);	//erase path_RRT[look_ahead_idx] from the path
			path_RRT.insert(it+look_ahead_idx, tmp_reverse.begin(), tmp_reverse.end()-1);		//from the point near path_RRT[look_ahead] to the very first current position
		
		}
		
		else {
			//rrtTree tree = rrtTree(robot_pose, path_RRT[look_ahead_idx+1], dynamic_map, map_origin_x, map_origin_y, res, 5);		//make a new path from current position to next lookahead point

			printf("path from (%f, %f) to (%f, %f)\n", path_RRT[look_ahead_idx-1].x,path_RRT[look_ahead_idx-1].y, path_RRT[look_ahead_idx+1].x,path_RRT[look_ahead_idx+1].y);
			rrtTree tree = rrtTree(path_RRT[look_ahead_idx-1], path_RRT[look_ahead_idx+1], dynamic_map, map_origin_x, map_origin_y, res, 10);	//margin 5
	
			int iteration = tree.generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 5000, 0.5);

			std::vector<point> tmp = tree.backtracking();
			int size = tmp.size();

			tree.visualizeTree(tmp);
			
			std::vector<point> tmp_reverse;

			for(int j=0; j<size; j++) {
				tmp_reverse.push_back(tmp[size-j-1]);
			}

////////////////////////////
			printf("path size = %d\n\n", size);
			for(int j=0; j<size; j++) {
				printf("path %d : (%f, %f)\n", j, tmp_reverse[j].x, tmp_reverse[j].y);
			}
///////////////////////////
			

			std::vector<point>::iterator it = path_RRT.begin();

			path_RRT.erase (it+look_ahead_idx);	//erase path_RRT[look_ahead_idx] from the path
			path_RRT.insert(it+look_ahead_idx, tmp_reverse.begin(), tmp_reverse.end()-1); //from the point near path_RRT[look_ahead] to the very first current position


		}	

		printf("path remake done\n");

		//look_ahead_idx++;				//from look ahead idx -1 to look ahead idx +1

		state = RUNNING;

		printf("path planning done\n");


            //TODO
            /*
	     * call void dynamic_mapping();
             * do dynamic mapping from kinect data
             * pop up the opencv window
             * after drawing the dynamic map, transite the state to RUNNING state
             */

            ros::spinOnce();
            control_rate.sleep();
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd_vel);
            running = false;

            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }

        printf("curr state : %d\ncurr robot pos : %.2f,%.2f\ncurr robot vel : %.2f,%.2f\n",state,robot_pose.x,robot_pose.y,cmd_vel.linear.x,cmd_vel.angular.z);
    }

    return 0;
}

void generate_path_RRT()
{
	int iteration = 0;
	int size = 0;

	std::vector<point> tmp;

	for(int i=0; i<3; i++){
		rrtTree tree = rrtTree(waypoints[i], waypoints[i+1], map, map_origin_x, map_origin_y, res, 5);

		iteration = tree.generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 5000, 1);	

		tmp = tree.backtracking();



		size = tmp.size();
		printf("waypoints[%d] to waypoints[%d], path_size = %d\n", i, i+1, size);
		
		for(int j=0; j<size; j++) {
			
			path_RRT.push_back(tmp[size-j-1]);
		}
	}

	int a = path_RRT.size();
	printf("path total size = %d\n", a);

	for(int k=0; k<a; k++) {
		printf("%d path (%f, %f)\n", k, path_RRT[k].x, path_RRT[k].y);
	}
}

void set_waypoints()
{
    point waypoint_candid[4];
    waypoint_candid[0].x = 5.0;
    waypoint_candid[0].y = -8.0;
    waypoint_candid[1].x = -6.0;
    waypoint_candid[1].y = -7.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 8.0;
    waypoint_candid[3].x = 3.0;
    waypoint_candid[3].y = 7.0;

    int order[] = {3,1,2,3};
    int order_size = 4;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"RosAria") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void callback_points(sensor_msgs::PointCloud2ConstPtr msgs){
    pcl::fromROSMsg(*msgs,point_cloud);
}

bool isCollision()
{

  for(int i = 0; i < point_cloud.size(); i++){
    //if(atan2(point_cloud[i].z, point_cloud[i].y)*(180/M_PI)>=70 && point_cloud[i].z<=0.57){
	if(point_cloud[i].y<-0.1 && point_cloud[i].y>-0.3){
      
       //if(hypot(point_cloud[i].z, point_cloud[i].x)<min_distance){
	if(hypot(point_cloud[i].z, point_cloud[i].x)<min_distance){
		return true;
        }

        else{
          continue;
        }
      }
  }

    return false;
}


void dynamic_mapping()
{

	double x_, y_;

	double r = 7*res;
	double cx_, cy_;	//center of obstacle
	double nx_, ny_;
	double x__, y__;
	double length;

	int i_map, j_map;


	for(int i = 0; i < point_cloud.size(); i++){
		//printf("%dth point cloud\n", i);

		//if(atan2(point_cloud[i].z, point_cloud[i].y)*(180/M_PI)>=70 && point_cloud[i].z<=0.6){
		if(point_cloud[i].y<-0.2 && point_cloud[i].y>-0.25){

			//if(hypot(point_cloud[i].z, point_cloud[i].x)<min_distance){
			if(std::abs(point_cloud[i].z)<min_distance){
				//printf("%dth point cloud -> dynamic mapping \n", i);
				x_ = -point_cloud[i].z;
				y_ = -point_cloud[i].x;
				length = hypot(x_,y_);
				cx_ = x_*(length+r)/length;
				cy_ = y_*(length+r)/length;

						
				int idx_x=0;
				int idx_y=0;
/*
				for(int idx_x = -4; idx_x < 5; idx_x++) {
					for(int idx_y = -4; idx_y < 5; idx_y++) {
						nx_ = cx_ + idx_x*res;
						ny_ = cy_ + idx_y*res;
						x__ = robot_pose.x - ( nx_*cos(robot_pose.th) - ny_*sin(robot_pose.th) );
						y__ = robot_pose.y - ( nx_*sin(robot_pose.th) + ny_*cos(robot_pose.th) );

						i_map = (int) (x__/res + map_origin_x);
						j_map = (int) (y__/res + map_origin_y);

						if(pow(cx_-nx_,2)+pow(cy_-ny_,2)<=pow(r,2))
							dynamic_map.at<uchar>(i_map, j_map) = 0;	
					}
				}
*/

				x__ = robot_pose.x - ( x_*cos(robot_pose.th) - y_*sin(robot_pose.th) );
				y__ = robot_pose.y - ( x_*sin(robot_pose.th) + y_*cos(robot_pose.th) );

				i_map = (int) (x__/res + map_origin_x);
				j_map = (int) (y__/res + map_origin_y);

				dynamic_map.at<uchar>(i_map, j_map) = 0;

			}
        	}
	}

	//TODO
	/*
	* draw dynamic map using variable dynamic_map and variable point_cloud
	*/
}

void setcmdvel(double v, double w){
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}
