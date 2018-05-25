#include "rrtTree.h"

rrtTree::rrtTree(){
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

rrtTree::~rrtTree(){
    for (int i = 0; i < count; i++) {	//revised
        delete ptrTable[i];
    }
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(60);

}

void rrtTree::visualizeTree(std::vector<point> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
        x1 = cv::Point((int)(Res*(path[i-1].y/res + map_origin_y)), (int)(Res*(path[i-1].x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(60);

}



// TODO
// 1. Copy your implementation of member functions in Project Assignment #2
void rrtTree::addVertex(point x_new, point x_rand, int idx_near) {
    // TODO

	root = new node;
	ptrTable[count] = root;
	root->idx = count;
	root->idx_parent = idx_near;
	root->location = x_new;
	root->rand = x_rand;

	count = count + 1;
}



point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
	double rand1 = x_min + rand()*(x_max - x_min)/(double)RAND_MAX;
	double rand2 = y_min + rand()*(y_max - y_min)/(double)RAND_MAX;


	point temp;
	temp.x = rand1; temp.y = rand2;

	return temp;

}

point rrtTree::newState(int idx_near, point x_rand, double MaxStep) {
    // TODO
	point temp;

	point x_near = ptrTable[idx_near]->location;

	double distance_rand_near = sqrt(pow( ( x_near.x - x_rand.x) , 2) + pow( ( x_near.y - x_rand.y) , 2) );

	temp.x = x_near.x + (x_rand.x - x_near.x) * MaxStep / distance_rand_near;
	temp.y = x_near.y + (x_rand.y - x_near.y) * MaxStep / distance_rand_near;
	
	return temp;	

}


int rrtTree::nearestNeighbor(point x_rand) {
    // TODO
	double min_distance = 10000000;
	double current_distance;

	
	int idx = -1;
	for(int i = 0; i<count; i++) {
		current_distance = pow( ( (ptrTable[i]->location).x - x_rand.x) , 2) + pow( ( (ptrTable[i]->location).y - x_rand.y) , 2) ;
		if (current_distance < min_distance) { 
			min_distance = current_distance; 
			idx = i;
		}
	}

	return idx;
}

bool rrtTree::near(point x_test, point x_new, double gamma, double vertex_number, double etta) {
	double test_distance = sqrt(pow(x_test.x - x_new.x , 2) + pow( x_test.y - x_new.y , 2));
	//double radius = std::min(gamma*sqrt(log(vertex_number)/vertex_number),etta);
	double radius = std::min(gamma*sqrt(log(30)/30),etta);
	//double radius = 10;
	if(test_distance<=radius)
		return true;
	else
		return false;
}


double rrtTree::cost(int idx) {	//recursive formula

	double distance;	
	int parent_index;

	if(idx == 0) {
		return 0;
	}

	else {
		parent_index = ptrTable[idx]->idx_parent;
		point parent_location = ptrTable[parent_index]->location;

		distance = cost(parent_index) + sqrt(pow(parent_location.x - (ptrTable[idx]->location).x, 2) + pow(parent_location.y - (ptrTable[idx]->location).y, 2));
	}

	return distance;	
}


bool rrtTree::isCollision(point x1, point x2) {		//x1 : x_near, x2 : x_new	//return true if there is collision
    // TODO
	int i_near = (int) (x1.x/res + map_origin_x);
	int j_near = (int) (x1.y/res + map_origin_y);
	
	int i_new = (int) (x2.x/res + map_origin_x);
	int j_new = (int) (x2.y/res + map_origin_y);

	//bool flag = false;

	if(i_near>=i_new && j_near>=j_new) {
		for(int i=i_new; i<=i_near; i++) {
			for(int j=j_new; j<=j_near; j++) {
				if(map.at<uchar>(i, j) < 125) {
					return true;
				}
			}	
		}	
	}
	if(i_near>=i_new && j_near<j_new) {
		for(int i=i_new; i<=i_near; i++) {
			for(int j=j_new; j>=j_near; j--) {
				if(map.at<uchar>(i, j) < 125) {
					return true;
				}
			}	
		}
	}

	if(i_near<i_new && j_near>=j_new) {
		for(int i=i_new; i>=i_near; i--) {
			for(int j=j_new; j<=j_near; j++) {
				if(map.at<uchar>(i, j) < 125) {
					return true;
				}
			}	
		}
	}

	if(i_near<i_new && j_near<j_new) {
		for(int i=i_new; i>=i_near; i--) {
			for(int j=j_new; j>=j_near; j--) {
				if(map.at<uchar>(i, j) < 125) {
					return true;
				}
			}	
		}
		
	}
	
	return false;

}


std::vector<point> rrtTree::backtracking(){
    // TODO
	int next_idx = nearestNeighbor(x_goal); //initially nearest from goal

	std::vector<point> goal_to_init;

	bool flag = true;

	while(flag == true){
		goal_to_init.push_back(ptrTable[next_idx]->location);
		
		if(ptrTable[next_idx]->idx == 0) {
			break;
		}
		else {
			next_idx = ptrTable[next_idx]->idx_parent;
		}

	}

	return goal_to_init;
	
}

// 2. Implement generateRRTst
int rrtTree::generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep)
{
    // TODO 2.
	point x_rand;
	int idx_nearest;
	int idx_min;

	point x_new;
	

	point x_init = ptrTable[0]->location;
	
	int i = 0;
	while(i<K) {
									////with biasing
		if(i%5 != 4){
			x_rand = randomState(x_max, x_min, y_max, y_min);
		}

		else {
			x_rand = x_goal;
		}
		

		x_rand = randomState(x_max, x_min, y_max, y_min);	

		idx_nearest = nearestNeighbor(x_rand);

		x_new = newState(idx_nearest, x_rand, MaxStep);
		
		point x_nearest = ptrTable[idx_nearest]->location;

		//if x_new is near from goal point
		if(pow( x_new.x - x_goal.x , 2) + pow( x_new.y - x_goal.y , 2) < pow(0.2,2) ) {
			//printf("%f\n", pow( x_new.x - x_goal.x , 2) + pow( x_new.y - x_goal.y , 2));
			break;
		}

		else {
			if(isCollision( x_nearest , x_new )  == true) {
				i ++;
				continue;
			}
			else{
				std::vector<int> X_near_ind;

				for(int j=0; j<count; j++) {	//X_near
					if(near(ptrTable[j]->location, x_new, 10, count, 5) == true)	//gamma = 10, etta = 5
						X_near_ind.push_back(j);
				}
				
				int a = X_near_ind.size();
				printf("x_near : number of points = %d\n", a);

				double idx_min = idx_nearest;
				
				
				double c_min = cost(idx_nearest) + sqrt(pow(x_nearest.x - x_new.x, 2) + pow(x_nearest.y - x_new.y, 2));

				for(int k=0; k<X_near_ind.size(); k++) {
					point x_near = ptrTable[X_near_ind[k]]->location;
					bool b1 = isCollision(x_near, x_new);
					double c_check = cost(X_near_ind[k]) + sqrt(pow(x_near.x - x_new.x, 2) + pow(x_near.y - x_new.y, 2));
					if(b1 == false && c_check < c_min) {
						printf("##################### x_near change ##########\n");
						idx_min = X_near_ind[k];
						c_min = c_check;
					}
				}

				addVertex(x_new, x_rand, idx_min);		


				for(int k=0; k<X_near_ind.size(); k++) {
					point x_near = ptrTable[X_near_ind[k]]->location;
					bool b1 = isCollision(x_new, x_near);
					double c_check = cost(count - 1) + sqrt(pow(x_near.x - x_new.x, 2) + pow(x_near.y - x_new.y, 2));
					if(b1 == false && c_check < cost(X_near_ind[k]) ) {
						printf("##################### parent change ##########\n");
						ptrTable[X_near_ind[k]]->idx_parent = count - 1;	//index of x_new
					}
				}

				i ++;
			}
		}
	}

/////////////////////////////////////////

/*
	std::vector<point> tmp;

	tmp = backtracking();
	visualizeTree(tmp);
*/
	
	
	return i; 		//return number of iteration

}
