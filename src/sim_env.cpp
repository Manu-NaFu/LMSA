#include "sim_env.h"
#include <QDebug>
#include <time.h>

void Sim_env::line(const int x0, const int y0, const int x1, const int y1, const int r, const int g, const int b)
{
    cv::Point p0(x0,y0), p1(x1,y1);

    cv::LineIterator it(this->map_, p0, p1, 8);

    for(int i = 0; i < it.count; i++, ++it)
        this->setValues(it.pos(), r, g, b);
}

void Sim_env::setValues(const cv::Point & pos, const int & r, const int & g, const int & b)
{
    if (this->map_.rows >= pos.y && this->map_.cols >= pos.x && pos.y>=0 && pos.x>=0)
    {
        map_.at<cv::Vec3b>(pos)[0] = b;
        map_.at<cv::Vec3b>(pos)[1] = g;
        map_.at<cv::Vec3b>(pos)[2] = r;
    }
}

//cv::Mat setValues_another_map(cv::Mat map, const cv::Point & pos, const int & r, const int & g, const int & b)
//{
//    if (map.rows >= pos.y && map.cols >= pos.x && pos.y>=0 && pos.x>=0)
//    {
//        map.at<cv::Vec3b>(pos)[0] = b;
//        map.at<cv::Vec3b>(pos)[1] = g;
//        map.at<cv::Vec3b>(pos)[2] = r;
//    }
//    return map;
//}

//cv::Mat line_another_map(cv::Mat map, const int x0, const int y0, const int x1, const int y1, const int r, const int g, const int b)
//{
//    cv::Point p0(x0,y0), p1(x1,y1);

//    cv::LineIterator it(map, p0, p1, 8);

//    for(int i = 0; i < it.count; i++, ++it)
//        map = setValues_another_map(map, it.pos(), r, g, b);
//    return map;
//}

void Sim_env::draw_system()
{
    int r = 0, g = 0, b = 255;
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
        {
            map_.at<cv::Vec3b>(P_.getY()-2+j,P_.getX()-2+i)[0] = b;
            map_.at<cv::Vec3b>(P_.getY()-2+j,P_.getX()-2+i)[1] = g;
            map_.at<cv::Vec3b>(P_.getY()-2+j,P_.getX()-2+i)[2] = r;
        }
    int x_sys = (cos( (P_.getAngle()+270) * PI / 180.0 ) * 10) + P_.getX();
    int y_sys = (sin( (P_.getAngle()+270) * PI / 180.0 ) * 10) + P_.getY();
    line(P_.getX(), P_.getY(), x_sys, y_sys, r, g, b);
}

void Sim_env::create_readings()
{
    num_readings_ = rand() % 100 + 530;
    readings_ =(rplidar_response_measurement_node_hq_t*) std::malloc(sizeof(rplidar_response_measurement_node_hq_t)*num_readings_);

    int new_x =   x_speed_ * 4;
    int new_y = - y_speed_ * 4;
    move_robot(new_x, new_y);

    for (int i=0; i < num_readings_; i++)
    {
        readings_[i].angle_z_q14 = (360.0/num_readings_)*i * (1 << 14) / 90.f;
        readings_[i].quality = 255;
        readings_[i].flag = 2;

        int x = (cos( ((360.0/num_readings_)*i+P_.getAngle()) * PI / 180.0 ) * laser_range_) + P_.getX();
        int y = (sin( ((360.0/num_readings_)*i+P_.getAngle()) * PI / 180.0 ) * laser_range_) + P_.getY();

        cv::Point p0(P_.getX(),P_.getY()), p1(x,y);

        cv::LineIterator it(this->map_, p0, p1, 8);

        int dist = laser_range_;
        int x_dist = 0;
        int y_dist = 0;

        for(int i = 0; i < it.count; i++, ++it)
        {
            if (it.pos().x >= map_.cols || it.pos().y >= map_.rows || it.pos().x < 1 || it.pos().y < 1)
                break;
            cv::Vec3b aux_pos = map_.at<cv::Vec3b>(it.pos().y, it.pos().x);
            if (aux_pos[2] > 220 && aux_pos[1] < 180 && aux_pos[0] < 180)
            {
                x_dist = it.pos().x - P_.getX();
                y_dist = it.pos().y - P_.getY();
                dist = sqrt(pow(x_dist,2) + pow(y_dist,2));
                break;
            }
            this->setValues(it.pos(), 50, 50, 50);
        }

        dist = dist + (2 - rand() % 4);
        dist = dist * 10.f * (1 << 2);
        readings_[i].dist_mm_q2 = dist;
    }
    draw_system();
    emit (emit_readings(readings_, num_readings_));
}

void Sim_env::move_robot(int inc_x, int inc_y)
{
    int new_x = inc_x+P_.getX();
    int new_y = inc_y+P_.getY();
    if (new_x > 0 && new_x < width() && new_y > 0 && new_y < height())
    {
        bool set = true;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 5; j++)
            {
                if (new_y-2+j > 0 && new_y-2+j < height() && new_x-2+i > 0 && new_x-2+i < width())
                {
                    cv::Vec3b aux_pos = map_.at<cv::Vec3b>(new_y-2+j, new_x-2+i);
                    if (aux_pos[2] > 220 && aux_pos[1] < 180 && aux_pos[0] < 180)
                        set = false;
                }
                else
                    set = false;
            }

        if (set)
        {
            P_.setX(new_x);
            P_.setY(new_y);
        }
    }

    //qDebug() << "Moved robot to x: " << P_.getX() << " y:" << P_.getY();
    emit(moved(P_.getPos()));
}

void Sim_env::init()
{
    running_ = false;
    started_ = false;
    finished_ = false;
    num_readings_ = 620;
    laser_range_ = 800;
    x_speed_ = 0;
    y_speed_ = 0;
    srand (time(NULL));
}

void Sim_env::set_parameters(slam::Position p, QString filename, double speed)
{
    map_file_ = filename;
    P_.setX(p.getX());
    P_.setY(p.getY());
    speed_ = speed;
}

void Sim_env::run()
{
    map_ = cv::imread(map_file_.toStdString());
    cv::namedWindow("Real map",cv::WINDOW_AUTOSIZE);
    cv::imshow("Real map",map_);
    started_ = true;

    while (started_ && !finished_)
    {
        cv::imshow("Real map",map_);
    }

    cv::destroyWindow("Real map");
}

void Sim_env::clear()
{
    //Default values for parameters and free memory.
    set_parameters(slam::Position(400,400,0), "", 100);
    finished_ = true;
    started_ = false;
    running_ = false;
    num_readings_ = 600;
    laser_range_ = 800;
    x_speed_ = 0;
    y_speed_ = 0;
    cv::waitKey(1000);

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

