#include "simulation.h"
#include "slam.hpp"
#include "levmarq.h"
#include <QThread>
#include <QMessageBox>
#include <QDebug>

//Error function to optimize using Levenberg Marquardt algorithm.
void error(const aruco::LevMarq<double>::eVector &sol, aruco::LevMarq<double>::eVector &err, const slam::Map &map, const double *dist, const double *theta, const int count)
{
    err.resize(1);
    slam::Position P(sol(0), sol(1), sol(2));
    err(0) = map.fit(P, dist, theta, count);
}


void Simulation::init()
{
    running_ = false;
    started_ = false;
    finished_ = false;
    loaded_ = false;
    live_ = true;
    readings_ready_ = false;
    start_ = 0;
    ignore_human_ = true;
    navigating_ = false;
    moving_ = false;
    safety_distance_ = 20;
    perc_occupancy_ = 1;
    started_new_path_ = false;
    heuristic_ = 'm';
}

void Simulation::create_cell_map()
{
    cells_m_ = (cell**)malloc(height_/(safety_distance_) * sizeof(cell*));
    for (int i = 0; i < width_/(safety_distance_); i++)
        cells_m_[i] = (cell*)malloc(height_/(safety_distance_) * sizeof(cell));

    for (int i = 0; i < height_/(safety_distance_) ; i++)
        for (int j = 0; j < width_/(safety_distance_) ; j++)
        {
            int x = (safety_distance_)/2.0 + (i * (safety_distance_));
            int y = (safety_distance_)/2.0 + (j * (safety_distance_));
            cells_m_[i][j].free = true;
            cells_m_[i][j].unknown = false;
            cells_m_[i][j].center = cv::Point(x, y);

            if (x - (safety_distance_)/2.0 >= 0)
                cells_m_[i][j].x_min = x - (safety_distance_)/2.0;
            else
                cells_m_[i][j].x_min = 0;

            if (x + (safety_distance_)/2.0 <= width_)
                cells_m_[i][j].x_max = x + (safety_distance_)/2.0;
            else
                cells_m_[i][j].x_max = width_;

            if (y - (safety_distance_)/2.0 >= 0)
                 cells_m_[i][j].y_min = y - (safety_distance_)/2.0;
            else
                cells_m_[i][j].y_min = 0;

            if (y + (safety_distance_)/2.0 <= height_)
                cells_m_[i][j].y_max = y + (safety_distance_)/2.0;
            else
                cells_m_[i][j].y_max = height_;
        }
}

void Simulation::compute_cell_occupancy()
{
    cv::Mat oc_map = map_.getOc();
    //slam::Map show_map(map_.getMap());

    for (int i = 0; i < height_/(safety_distance_) ; i++)
        for (int j = 0; j < width_/(safety_distance_) ; j++)
        {
            //qDebug() << "i: " << i << " j: " << j;
            int count_occupied = 0;
            int x_min = cells_m_[i][j].x_min;
            int x_max = cells_m_[i][j].x_max;
            int y_min = cells_m_[i][j].y_min;
            int y_max = cells_m_[i][j].y_max;
//            qDebug() << "i: " << i << " j: " << j;
//            qDebug() << "x_min: " << x_min << " x_max: " << x_max;
//            qDebug() << "y_min: " << y_min << " y_max: " << y_max;
            for (int x = x_min; x < x_max ; x++)
                for (int y = y_min; y < y_max ; y++)
                {
                    //qDebug() << oc_map.at<uchar>(x,y);
                    if (oc_map.at<uchar>(y,x) > 160)
                        count_occupied++;
                }
//            qDebug() << count_occupied;
//            qDebug() << (1.0*count_occupied/((x_max-x_min)*(y_max-y_min)));
//            qDebug() << (perc_occupancy_/100.0);
//            qDebug() << ((1.0*count_occupied/((x_max-x_min)*(y_max-y_min))) > (perc_occupancy_/100.0));
//            qDebug() << (0.0752 > 0.05);
            if ((1.0*count_occupied/((x_max-x_min)*(y_max-y_min))) > (perc_occupancy_/100.0))
                cells_m_[i][j].free = false;
            else
                cells_m_[i][j].free = true;

//            if (!cells_m_[i][j].free)
//            {
//                show_map.line(x_min, y_min, x_max, y_min, 0, 255, 0);
//                show_map.line(x_min, y_min, x_min, y_max, 0, 255, 0);
//                show_map.line(x_min, y_max, x_max, y_max, 0, 255, 0);
//                show_map.line(x_max, y_min, x_max, y_max, 0, 255, 0);
//                show_map.line(x_min+1, y_min+1, x_max-1, y_min+1, 0, 255, 0);
//                show_map.line(x_min+1, y_min+1, x_min+1, y_max-1, 0, 255, 0);
//                show_map.line(x_min+1, y_max-1, x_max-1, y_max-1, 0, 255, 0);
//                show_map.line(x_max-1, y_min+1, x_max-1, y_max-1, 0, 255, 0);
//            }
//            else
//            {
//                show_map.line(x_min, y_min, x_max, y_min, 0, 0, 255);
//                show_map.line(x_min, y_min, x_min, y_max, 0, 0, 255);
//                show_map.line(x_min, y_max, x_max, y_max, 0, 0, 255);
//                show_map.line(x_max, y_min, x_max, y_max, 0, 0, 255);
//                show_map.line(x_min+1, y_min+1, x_max-1, y_min+1, 0, 0, 255);
//                show_map.line(x_min+1, y_min+1, x_min+1, y_max-1, 0, 0, 255);
//                show_map.line(x_min+1, y_max-1, x_max-1, y_max-1, 0, 0, 255);
//                show_map.line(x_max-1, y_min+1, x_max-1, y_max-1, 0, 0, 255);
//            }
//            qDebug() << "-----------------------";

        }

    //cv::namedWindow("cells",cv::WINDOW_AUTOSIZE);
    //cv::imshow("cells",show_map.getMap());
}


int Simulation::canTravel(int x, int y)
{
    //check for out of bounds
    if(x < 0 or x >=(width_/(safety_distance_)) or y < 0 or y >= (height_/(safety_distance_)))
        return 0;
    else if(!cells_m_[x][y].free) //wall reached, cannot travel
        return 0;
    else
        return 1;
}

int Simulation::get_neighbors(int x, int y, node * neighbors)
{
    node nodeToAdd;
    nodeToAdd.parent_pos = cv::Point(x, y);
    int numNeighbors = 0;
    //qDebug() << "1";
    if(canTravel(x-1, y) == 1)  //can travel up
    {
        nodeToAdd.pos = cv::Point(x-1, y);
        neighbors[numNeighbors] = nodeToAdd;
        numNeighbors++;
    }
    //qDebug() << "2";
    if(canTravel(x, y-1) == 1)  //can travel left
    {
        nodeToAdd.pos = cv::Point(x, y-1);
        neighbors[numNeighbors] = nodeToAdd;
        numNeighbors++;
    }
    //qDebug() << "3";
    if(canTravel(x,y+1) == 1)   //can travel right
    {
        nodeToAdd.pos = cv::Point(x, y+1);
        neighbors[numNeighbors] = nodeToAdd;
        numNeighbors++;
    }
    //qDebug() << "4";
    if(canTravel(x+1, y) == 1)  //can travel down
    {
        nodeToAdd.pos = cv::Point(x+1, y);
        neighbors[numNeighbors] = nodeToAdd;
        numNeighbors++;
    }
    //qDebug() << "5";
    return numNeighbors;
}

void Simulation::reconstruct_path(node next, int system_x, int system_y, node **node_track)
{
    node node_aux = next;
    navigation_points_.clear();
    //navigation_points_.insert(navigation_points_.begin(), next.pos);
    int path_size = 0;
    while (node_aux.pos.x != system_x or node_aux.pos.y != system_y)
    {
        cv::Point child_raw  = cells_m_[node_aux.pos.x][node_aux.pos.y].center;
        cv::Point parent_raw = cells_m_[node_aux.parent_pos.x][node_aux.parent_pos.y].center;
        //qDebug() << "x : " << node_aux.pos.x << " y : " << node_aux.pos.y;
        //qDebug() << "x parent : " << node_aux.parent_pos.x << " y parent raw: " << node_aux.parent_pos.y;
        //qDebug() << "x raw: " << child_raw.x << " y : " << child_raw.y;
        //qDebug() << "x parent raw: " << parent_raw.x << " y parent raw: " << parent_raw.y;
        navigation_points_.insert(navigation_points_.begin(), child_raw);
        cv::Point aux_point(child_raw);
        int n_points = 4;
        for (int n_p = n_points; n_p > 1; n_p--)
        {
            aux_point.x = aux_point.x - (aux_point.x - parent_raw.x)/n_p;
            aux_point.y = aux_point.y - (aux_point.y - parent_raw.y)/n_p;
            //qDebug() << "aux point x: " << aux_point.x << " y: " << aux_point.y;
            navigation_points_.insert(navigation_points_.begin(), aux_point);
        }
        node_aux = node_track[node_aux.parent_pos.x][node_aux.parent_pos.y];
        //qDebug() << "x raw : " << cells_m_[node_aux.pos.x][node_aux.pos.y].center.x << " y raw : " << cells_m_[node_aux.pos.x][node_aux.pos.y].center.y;
        path_size++;
    }
    navigation_points_.insert(navigation_points_.begin(), cells_m_[system_x][system_y].center);
    cv::Point aux_point(P_.getX(), P_.getY());

    aux_point.x = aux_point.x - (aux_point.x - cells_m_[system_x][system_y].center.x)/2;
    aux_point.y = aux_point.y - (aux_point.y - cells_m_[system_x][system_y].center.y)/2;
    navigation_points_.insert(navigation_points_.begin(), aux_point);

    //qDebug() << "aux point center x: " << aux_point.x << " y: " << aux_point.y;
    //navigation_points_.insert(navigation_points_.begin(), aux_point);
    navigation_points_.push_back(target_p_);

    //qDebug() << " ----- Path -----";
    //for (int i = 0; uint(i) < navigation_points_.size(); i++)
        //qDebug() << "(" << navigation_points_[i].x << "," << navigation_points_[i].y << ")";
    //qDebug() << " ----------------";
    //qDebug() << "Path size: " << path_size;
}

double heuristic(cv::Point pos, cv::Point target_pos, char heuristic)
{
    if (heuristic == 'm')
        return abs(pos.x - target_pos.x) + abs(pos.y - target_pos.y);
    else if (heuristic == 'e')
        return sqrt(pow(pos.x - target_pos.x, 2) + pow(pos.y - target_pos.y, 2));
    return 0;
}


bool prev_taken_x = false;
double astar_runtime;
int num_astar_executions;
int Simulation::find_path()
{
    //qDebug() << "===================";
//    QElapsedTimer timer;
//    timer.start();
    bool found_path = false;
    int curr_dist = 0;
    int target_x = (target_p_.x/(safety_distance_));
    int target_y = (target_p_.y/(safety_distance_));

    //qDebug () << "Target x raw: " << target_p_.x << "Target y raw: " << target_p_.y;
    //qDebug () << "Target x: " << target_x << "Target y: " << target_y;

    node curr_node;
    int system_x = P_.getX()/(safety_distance_);
    int system_y = P_.getY()/(safety_distance_);

    //qDebug () << "System x: " << system_x << " System y: " << system_y;

    curr_node.pos = cv::Point(system_x, system_y);
    curr_node.heap = 'o';
    curr_node.distance_walked = curr_dist;
    curr_node.heuristic = heuristic(curr_node.pos, cv::Point(target_x, target_y), heuristic_);
    curr_node.total_dist = curr_node.distance_walked + curr_node.heuristic;

    std::vector<node> hold_nodes;
    hold_nodes.push_back(curr_node);

    node **node_track;
    node_track = (node**)malloc(height_/(safety_distance_) * sizeof(node*));
    for (int i = 0; i < width_/(safety_distance_); i++)
        node_track[i] = (node*)malloc(height_/(safety_distance_) * sizeof(node));
    node_track[curr_node.pos.x][curr_node.pos.y] = curr_node;
    //qDebug() << "x pos : " << node_track[16][16].pos.x << " y pos: " << node_track[16][16].pos.y;
    node neighbors[4];

    //qDebug("Start");

    while (!found_path && !hold_nodes.empty())
    {
        //qDebug() << "---------------------";
        //qDebug() << "while";
        // Find best node on hold
        int aux_dist = (width_/(safety_distance_) + height_/(safety_distance_));
        //qDebug() << "aux dist: " << aux_dist;
        std::vector<node>::iterator aux_it = hold_nodes.begin();
        //qDebug() << "looking for best node";
        node aux_node;
        for (std::vector<node>::iterator it = hold_nodes.begin(); it < hold_nodes.end(); it++)
        {
            //qDebug() << "pos x: " << (*it).pos.x << " y: " << (*it).pos.y << " dist: " << (*it).total_dist;
            //qDebug() << "x pos : " << (*it).pos.x << " y pos: " << (*it).pos.y;
            if ((*it).total_dist < aux_dist)
            {
                aux_node = *it;
                aux_dist = aux_node.total_dist;
                aux_it = it;
                //qDebug() << "found";
                //qDebug() << "x best : " << (*it).pos.x << " y best: " << (*it).pos.y << " total dist best: " << (*it).total_dist;
            }
            else if ((*it).total_dist == aux_dist)
            {
//                qDebug() << "\n+++++++++";
//                qDebug() << "equal";
//                qDebug() << "pos x: " << (*it).pos.x << " y: " << (*it).pos.y << " dist: " << (*it).total_dist;
//                qDebug() << "pos x: " << aux_node.pos.x << " y: " << aux_node.pos.y << " dist: " << aux_node.total_dist;
//                qDebug() << !prev_taken_x << " curr x: " << curr_node.pos.x << "new x: " << (*it).pos.x << (curr_node.pos.x != (*it).pos.x);
//                qDebug() <<  prev_taken_x << " curr y: " << curr_node.pos.y << "new y: " << (*it).pos.y << (curr_node.pos.y != (*it).pos.y);
                if (prev_taken_x && ((*it).parent_pos.y != (*it).pos.y))
                {
                    //qDebug() << "Take y";
                    aux_node = *it;
                    aux_dist = aux_node.total_dist;
                    aux_it = it;
                }
                else if (!prev_taken_x && ((*it).parent_pos.x != (*it).pos.x))
                {
                    //qDebug() << "Take x";
                    aux_node = *it;
                    aux_dist = aux_node.total_dist;
                    aux_it = it;
                }
                //qDebug() << "+++++++++\n";
            }
        }

        if (curr_node.pos.y != aux_node.pos.y)
            prev_taken_x = false;
        else if (curr_node.pos.x != aux_node.pos.x)
            prev_taken_x = true;

        curr_node = aux_node;

        //qDebug() << " -- x pos curr: " << curr_node.pos.x << " y pos curr: " << curr_node.pos.y << " --";

        curr_dist = curr_node.distance_walked;
        hold_nodes.erase(aux_it);
        //qDebug() << "hold size: " << hold_nodes.size();


        node_track[curr_node.pos.x][curr_node.pos.y].heap = 'r';

        int numNeighbors = get_neighbors(curr_node.pos.x, curr_node.pos.y, neighbors);    //get list of neighbors

        /*for each neighbor*/
        int cnt = 0;
        for(cnt = 0; cnt<numNeighbors; cnt++)   //for each found neighbor
        {
            node next = neighbors[cnt];
            //qDebug() << "neighbor: " << cnt;
            //qDebug() << "x pos : " << next.pos.x << " y pos: " << next.pos.y;
            //qDebug() << "x pos parent: " << next.parent_pos.x << " y pos parent: " << next.parent_pos.y;

            /*if neighbor is the goal, stop the search*/
                /*store the path that the robot is going to take*/
            if((next.pos.x == target_x) && (next.pos.y == target_y))
            {
                //qDebug() << "\npath found\n";
                reconstruct_path(next, system_x, system_y, node_track);
                found_path = true;
                num_astar_executions++;
                //astar_runtime += timer.elapsed();
                //qDebug() << "Elapsed A* time (ms): " << astar_runtime;
                for (int i = 0; i < width_/(safety_distance_); i++)
                    free(node_track[i]);
                free(node_track);
                return 1;
            }

            next.distance_walked = curr_dist + 1; // + 1 becuase all neighbors are 1 away, this would change for diagonal travel

            next.heuristic = heuristic(next.pos, cv::Point(target_x, target_y), heuristic_);

            next.total_dist = next.distance_walked + next.heuristic;

            //qDebug() << "checks";
            /*if a node with the same position as neighbor is in the OPEN list
                which has a lower total distance than neighbor, skip putting this neighbor onto the open set*/
            //check if node is on the open set already
            bool skip_push = false;
            if(node_track[next.pos.x][next.pos.y].heap == 'o')   //on the open set
            {
                if(node_track[next.pos.x][next.pos.y].total_dist <= next.total_dist)
                {
                    //qDebug() << "skip 1";
                    skip_push = true;     //skip putting this node onto the openset, a better one already on there
                }
            }

            /*if a node with the same position as neighbor is in the CLOSED list
                which has a lower f than neighbor, skip putting this neighbor onto the open set*/
            else if(node_track[next.pos.x][next.pos.y].heap == 'c')      //on closed set
            {
                //qDebug() << " total dist prev: " << node_track[next.pos.x][next.pos.y].total_dist;
                //qDebug() << " total dist new: " << next.total_dist;
                if(node_track[next.pos.x][next.pos.y].total_dist <= next.total_dist)
                {
                    //qDebug() << "skip 2";
                    skip_push = true;     //skip putting this node onto the openset, already part of a possible solution
                }
            }

            //qDebug() << "skip";
            /*if not skipping putting this node on the set, then push node onto the open set
                and update the dictionary to indicate the node is on the open set and set the parents of this node to the current node*/
                //(can't be an ordinary else to the things above because then it won't push onto the open set if already on open or closed set)
            if(!skip_push)
            {
                //qDebug() << "++ not skipped ++";
                node_track[next.pos.x][next.pos.y].heap = 'o';       //mark in nodetrack that this is going onto the open set
                node_track[next.pos.x][next.pos.y].parent_pos = curr_node.pos;   //set neighbor's parent position to current position
                node_track[next.pos.x][next.pos.y].pos = next.pos;
                node_track[next.pos.x][next.pos.y].heuristic = next.heuristic;
                node_track[next.pos.x][next.pos.y].distance_walked = next.distance_walked;
                node_track[next.pos.x][next.pos.y].total_dist = next.total_dist;
                //10.  push this neighbor on which queue?
                // Choose one of these two lines of code
                // IF openSet
                hold_nodes.insert(hold_nodes.begin(), next);
                //printf("Push to OpenList, Row=%d,Col=%d,g=%d,h=%d,f=%d\n",next.row,next.col,next.distTravelFromStart,next.distToGoal,next.totalDist);
            }
        }
        //qDebug() << "close node";
        node_track[curr_node.pos.x][curr_node.pos.y].heap = 'c';
        //11.  push q (current node) on which queue?
        // Choose one of these two lines of code
        // IF closedSet
        //qDebug() << "hold size end: " << hold_nodes.size();
    }

    qDebug() << "path not found";
    return 0;
}


std::vector<cv::Point> moved_pos_v;
void Simulation::run()
{
    map_.resize(width_,height_); //Map that will be created.
    create_cell_map();

    QElapsedTimer timer;

    int path_state = -1;
    int system_x = P_.getX()/(safety_distance_);
    int system_y = P_.getY()/(safety_distance_);
    int prev_sys_x = 0;
    int prev_sys_y = 0;
    std::vector<double> error_pos_v;
    uint prev_moved_size = 0;

    std::vector<cv::Point> navigated_points;
    int navigated_cells = 0;

//    for (int i = 0; i < height_/(safety_distance_) ; i++)
//        for (int j = 0; j < width_/(safety_distance_) ; j++)
//        {
//            qDebug() << "x: " << cells_m_[i][j].center.x() << "y: " << cells_m_[i][j].center.y();
//            qDebug() << "x_min: " << cells_m_[i][j].x_min << "x_max: " << cells_m_[i][j].x_max;
//            qDebug() << "y_min: " << cells_m_[i][j].y_min << "y_max: " << cells_m_[i][j].y_min;
//        }

    int pos = 0, prev_pos = 0; //Counters for managing readings.
    int x = 0, y = 0; //Variables to calculate where to print a line in the map.
    double *dist_ =(double*) std::malloc(sizeof(double)*8192); //Vector of distances from the laser ray.
    double *theta_ =(double*) std::malloc(sizeof(double)*8192); //Vector of angles from the laser ray.

    if (live_)
        count_ = 650;

    //While the map is not finished and there are still readings (pos < count).
    while(pos < count_ && !finished_ && started_)
    {  
        compute_cell_occupancy();
        system_x = P_.getX()/(safety_distance_);
        system_y = P_.getY()/(safety_distance_);
        //qDebug() << pos << finished_ << started_;
        if (live_)
            pos = 0;
        //while the simulation is stopped, save the map.
        while(!running_ && !map_.getMap().empty() && !finished_ && started_)
        {
            if(!started_)
                break;
            else
                cv::imwrite("map_sim_stop.png",map_.getMap());
        }
        //If the user has reset while the simulation was stopped, leave.
        if (!started_){break;}

        //Undraw the system from the map in order to clear that space.
        map_.undrawSystem(P_);

        //Manage readings.
        prev_pos = pos;
        //Total readings in this stage.
        int t_stg_readings = 0;

        if (navigating_)
        {
            if (system_x != prev_sys_x or system_y != prev_sys_y or started_new_path_)
            {
                started_new_path_ = false;
                navigated_cells++;
                compute_cell_occupancy();
                path_state = find_path();
            }

            if (path_state == 0)
            {
                navigating_ = false;
                moving_ = false;
                navigation_points_.clear();
                emit(path_not_found());
            }
            else if (path_state == 1)
            {
                if (navigation_points_.empty())
                    navigating_ = false;
                else
                {
                    moving_ = true;
//                    qDebug() << "_-_-_-_-_-_-_-";
//                    qDebug() << " Moving robot x : " << navigation_points_[0].x - P_.getX() << " y: " << navigation_points_[0].y - P_.getY();
//                    qDebug() << " robot x : " << P_.getX() << " y: " << P_.getY();
//                    qDebug() << " navigation x : " << navigation_points_[0].x << " y: " << navigation_points_[0].y;
//                    qDebug() << "_-_-_-_-_-_-_-";
                    emit(move_robot(navigation_points_[0].x - P_.getX(), navigation_points_[0].y - P_.getY()));
                    navigated_points.push_back(navigation_points_[0]);
                    navigation_points_.erase(navigation_points_.begin());
                }
            }

            prev_sys_x = system_x;
            prev_sys_y = system_y;
        }

        if (navigation_points_.empty())
            navigating_ = false;
        while(moving_);

        if (live_)
        {
            emit(wait_for_readings());
            while (!readings_ready_);
            readings_ready_ = false;
        }

        //While the angle is not superior to 359, get the angle and the distance and save it into a buffer.
        for (t_stg_readings = 0; (readings_[pos].angle_z_q14 * 90.f / (1 << 14)) < 358.0 ; t_stg_readings++)
        {
            //Angle in degrees
            theta_[t_stg_readings] = readings_[pos].angle_z_q14 * 90.f / (1 << 14);
            //Distance in cm
            dist_[t_stg_readings] = readings_[pos].dist_mm_q2 / 10.f / (1 << 2);

            pos++;
        }

        //qDebug() << " Passed\n";
        //If it is not the first reading and there has been readings (pos-prev_pos != 0), the position is estimated.
        if (start_ == 1 && pos-prev_pos != 0)
        {
            //If the selected algorithm is Levenberg Marquardt, then a solver is created.
            //The params for the solver would be to stop after 2000 iterations or after the error is under 0.000001.
            //The variation of each variable (x,y,angle) is algo set.
            if (levmarq_)
            {
                aruco::LevMarq<double> Solver;
                aruco::LevMarq<double>::eVector sol(3);
                Solver.setParams(2000,0.000001);
                sol(0) = P_.getX();
                sol(1) = P_.getY();
                sol(2) = P_.getAngle();
                Solver.setDervEpsilon({x_var_, y_var_, angle_var_});
                Solver.solve(sol, bind(error, std::placeholders::_1, std::placeholders::_2, map_, dist_, theta_, pos-prev_pos));
                P_.setX(sol(0));
                P_.setY(sol(1));
                P_.setAngle(sol(2));
            }
            else
            //If brute force is the chosen algorithm, the software will execute this code.
            {
                //timer.start();
                map_.bruteForce(P_, dist_, theta_, pos-prev_pos, range_x_, range_y_, range_angle_, x_var_, y_var_, angle_var_);
                //qDebug() << "Elapsed brute force time (ms): " << timer.elapsed();
            }
        }

        if (moved_pos_v.size() > prev_moved_size)
        {
            //qDebug() << "sys x: " << P_.getX() << "sys y: " << P_.getY();
            double aux_error = abs(P_.getX() - moved_pos_v[0].x)*1.0/width_ + abs(P_.getY() - moved_pos_v[0].y)*1.0/height_;
            error_pos_v.push_back(aux_error);
            prev_moved_size = moved_pos_v.size();
        }

        //Once the position is estimated, this code calculates where to print lines using trigonometry to obtain the x and y coordinates in the map and prints the lines.
        for (int i = 0; i < t_stg_readings ; i++)
        {
            //If the software should ignore the human, then ignores some angles of the readings.
            if (ignore_human_)
            {
                if(theta_[i] >125 || theta_[i] < 55)
                {
                    qDebug() << theta_[i];
                    x = (cos( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getX();
                    y = (sin( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getY();


                    if (x <= map_.getCols() && y <= map_.getRows() && dist_[i]>0.1)
                        map_.lineToObject(P_.getX(), P_.getY(), x, y, w_);
                }
            }
            else
            {
                x = (cos( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getX();
                y = (sin( (theta_[i]+P_.getAngle()) * PI / 180.0 ) * dist_[i]) + P_.getY();
                //printf("theta (degrees): %03.2f Dist (cm): %08.2f Pos: %d X: %d Y: %d  \n",  theta, dist[pos], pos,points[pos].x,points[pos].y);
                if (dist_[i]>0.1)
                    map_.lineToObject(P_.getX(), P_.getY(), x, y, w_);
            }
        }

        //The map is updated with the new position of the system and the new lines printed.
        map_.update(P_);
        //qDebug() << "Robot pos after readings x: " << P_.getX() << " y: " << P_.getY();

        for (int i = 0; i < height_/(safety_distance_) ; i++)
            for (int j = 0; j < width_/(safety_distance_) ; j++)
            {
                int x_min = cells_m_[i][j].x_min;
                int x_max = cells_m_[i][j].x_max;
                int y_min = cells_m_[i][j].y_min;
                int y_max = cells_m_[i][j].y_max;
                if (!cells_m_[i][j].free)
                {
                    map_.line(x_min, y_min, x_max, y_min, 0, 255, 0);
                    map_.line(x_min, y_min, x_min, y_max, 0, 255, 0);
                    map_.line(x_min, y_max, x_max, y_max, 0, 255, 0);
                    map_.line(x_max, y_min, x_max, y_max, 0, 255, 0);
                    map_.line(x_min+1, y_min+1, x_max-1, y_min+1, 0, 255, 0);
                    map_.line(x_min+1, y_min+1, x_min+1, y_max-1, 0, 255, 0);
                    map_.line(x_min+1, y_max-1, x_max-1, y_max-1, 0, 255, 0);
                    map_.line(x_max-1, y_min+1, x_max-1, y_max-1, 0, 255, 0);
                }
                else
                {
                    map_.line(x_min, y_min, x_max, y_min, 0, 0, 180);
                    map_.line(x_min, y_min, x_min, y_max, 0, 0, 180);
                    map_.line(x_min, y_max, x_max, y_max, 0, 0, 180);
                    map_.line(x_max, y_min, x_max, y_max, 0, 0, 180);
                    map_.line(x_min+1, y_min+1, x_max-1, y_min+1, 0, 0, 180);
                    map_.line(x_min+1, y_min+1, x_min+1, y_max-1, 0, 0, 180);
                    map_.line(x_min+1, y_max-1, x_max-1, y_max-1, 0, 0, 180);
                    map_.line(x_max-1, y_min+1, x_max-1, y_max-1, 0, 0, 180);
                }
            }

        for (int i = 0; uint(i) < navigation_points_.size(); i++)
        {
            map_.setValues(navigation_points_[i].x-1, navigation_points_[i].y,   0, 255, 0);
            map_.setValues(navigation_points_[i].x,   navigation_points_[i].y,   0, 255, 0);
            map_.setValues(navigation_points_[i].x+1, navigation_points_[i].y,   0, 255, 0);
            map_.setValues(navigation_points_[i].x-1, navigation_points_[i].y-1, 0, 255, 0);
            map_.setValues(navigation_points_[i].x,   navigation_points_[i].y-1, 0, 255, 0);
            map_.setValues(navigation_points_[i].x+1, navigation_points_[i].y-1, 0, 255, 0);
            map_.setValues(navigation_points_[i].x-1, navigation_points_[i].y+1, 0, 255, 0);
            map_.setValues(navigation_points_[i].x,   navigation_points_[i].y+1, 0, 255, 0);
            map_.setValues(navigation_points_[i].x+1, navigation_points_[i].y+1, 0, 255, 0);
        }

        for (int i = 0; uint(i) < navigated_points.size(); i++)
        {
            map_.setValues(navigated_points[i].x-1, navigated_points[i].y,   140, 140, 140);
            map_.setValues(navigated_points[i].x,   navigated_points[i].y,   140, 140, 140);
            map_.setValues(navigated_points[i].x+1, navigated_points[i].y,   140, 140, 140);
            map_.setValues(navigated_points[i].x-1, navigated_points[i].y-1, 140, 140, 140);
            map_.setValues(navigated_points[i].x,   navigated_points[i].y-1, 140, 140, 140);
            map_.setValues(navigated_points[i].x+1, navigated_points[i].y-1, 140, 140, 140);
            map_.setValues(navigated_points[i].x-1, navigated_points[i].y+1, 140, 140, 140);
            map_.setValues(navigated_points[i].x,   navigated_points[i].y+1, 140, 140, 140);
            map_.setValues(navigated_points[i].x+1, navigated_points[i].y+1, 140, 140, 140);
        }

        //The image is sent to the main window as a pixmap to be printed in the Qlabel.
        mat_ = map_.getMap();
        cv::resize(mat_,mat_,cv::Size(800,800));

        QImage img((const uchar*)mat_.data, mat_.cols, mat_.rows, mat_.step, QImage::Format_RGB888);
        emit(print_img(img.copy()));

        //if (path_state == 1)
        //    while(true){}

        //The time this process is asleep is proportional to the speed that the users sets.
        usleep((10000/speed_)*1000);

        //From now on, the software will know it is not the first frame.
        start_ = 1;

        //If there were some readings left, this code ignores them.
        while ((readings_[pos].angle_z_q14 * 90.f / (1 << 14)) > 357.0)
            pos++;
        if (live_)
            pos--;

        //If pos >= count it means the simulation should finish as there are no more readings left.
        //The image of the map is saved.
        if (pos >= count_)
        {
            running_ = false;
            finished_ = true;
            emit sim_finished();
        }
    }
    double moved_error_total = 0;
    for (int i = 0; uint(i) < error_pos_v.size(); i++)
        moved_error_total += error_pos_v[i];

    moved_error_total = moved_error_total/error_pos_v.size();

    qDebug() << "Mean relative error during simulation: " << moved_error_total;

    qDebug() << "Path size: " << navigated_cells;

    qDebug() << "Mean A* runtime: " << astar_runtime/num_astar_executions;

    if (!map_.getMap().empty())
        cv::imwrite("map_sim_finished.png",map_.getMap());
}

void Simulation::start_navigation(cv::Point target_p)
{
    target_p_ = target_p;
    navigating_ = true;
    navigation_points_.clear();
    cv::Point p0(P_.getX(),P_.getY()), p1(target_p_.x,target_p_.y);
    cv::Mat map = map_.getMap();

    cv::LineIterator it(map, p0, p1, 8);

    bool blocked = false;
    for(int i = 0; i < it.count; i++, ++it)
    {
        for (int j = 0; j < 16; j++)
            for (int k = 0; k < 16; k++)
            {
                if (it.pos().y-8+j > 0 && it.pos().y-8+j < map.rows && it.pos().x-8+k > 0 && it.pos().x-8+k < map.cols)
                {
                    cv::Vec3b aux_pos = map.at<cv::Vec3b>(it.pos().y-8+j, it.pos().x-8+k);
                    if (aux_pos[2] > 220 && aux_pos[1] < 180 && aux_pos[0] < 180)
                        blocked = true;
                }
            }
    }

    if (blocked == false)
    {
        cv::LineIterator it(map, p0, p1, 8);

        for(int i = 0; i < it.count; i++, ++it)
        {
            if (i%5 == 0)
                navigation_points_.push_back(it.pos());
        }
    }
}


void Simulation::moved_robot(cv::Point pos)
{
    moved_pos_v.insert(moved_pos_v.begin(), pos);
    //qDebug() << "Moved x: " << pos.x << "Moved y: " << pos.y;
    //qDebug() << "sys x: " << P_.getX() << "sys y: " << P_.getY();
    moving_ = false;
}

void Simulation::finished_navigation()
{
    navigating_ = false;
}

void Simulation::set_parameters(slam::Position p, int width, int height, int x_var, int y_var, double angle_var, double w, double speed, bool levmarq, int range_x, int range_y, int range_angle, bool ignore_human, int  safety_distance, int  perc_occupancy, char heuristic)
{
    P_.setX(p.getX());
    P_.setY(p.getY());
    P_.setAngle(p.getAngle());
    x_var_= x_var;
    y_var_= y_var;
    width_= width;
    height_ =height;
    w_ = w;
    angle_var_ = angle_var;
    speed_ = speed;
    range_x_ = range_x;
    range_y_ = range_y;
    range_angle_ = range_angle;
    levmarq_ = levmarq;
    ignore_human_ = ignore_human;
    safety_distance_ = safety_distance;
    perc_occupancy_ = perc_occupancy;
    heuristic_ = heuristic;
}

void Simulation::clear()
{
    //Default values for parameters and free memory.
    set_parameters(slam::Position(400,400,0), 800, 800, 1, 1, 1.5, 0.7, 100, true, 20, 20, 60, true, 20, 1, 'm');
    finished_ = true;
    started_ = false;
    running_ = false;
    loaded_ = false;
    live_ = false;
    readings_ready_ = false;
    navigating_ = false;
    moving_ = false;
    cv::waitKey(1000);
    start_ = 0;
    count_ = 0;
    started_new_path_ = false;

    readings_ =(rplidar_response_measurement_node_hq_t*) std::malloc(sizeof(rplidar_response_measurement_node_hq_t)*200000);
    for (int i = 0; i < 200000; i++)
    {
        readings_[i].angle_z_q14 = 0;
        readings_[i].dist_mm_q2 = 0;
        readings_[i].quality = 0;
        readings_[i].flag = 0;
    }

    delete[] readings_;
}


void Simulation::update_readings(rplidar_response_measurement_node_hq_t *readings, int count)
{
    //qDebug() << "\nReadings ready\n";
    for (int i=0; i < count; i++)
    {
        readings_[i].angle_z_q14 = readings[i].angle_z_q14;
        readings_[i].dist_mm_q2 = readings[i].dist_mm_q2;
        readings_[i].quality = readings[i].quality;
        readings_[i].flag = readings[i].flag;
    }
    for (int i = count; i < 650; i++)
    {
        readings_[i].angle_z_q14 = 0;
        readings_[i].dist_mm_q2 = 0;
        readings_[i].quality = 0;
        readings_[i].flag = 0;
    }
    readings_ready_ = true;
    count_ = count + 1;
    //qDebug() << "\nReceived --\n";
}



