#ifndef SIM_ENV_H
#define SIM_ENV_H

#include "sdk/include/rplidar.h"
#include "slam.hpp"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <QObject>
#include <QPoint>
#include <QRunnable>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

class Sim_env : public QObject, public QRunnable
{
    Q_OBJECT

private:
    slam::Position P_;
    rplidar_response_measurement_node_hq_t * readings_;
    QString map_file_;
    bool finished_;
    bool started_;
    bool running_;
    double speed_;
    int num_readings_;
    int laser_range_;
    float x_speed_;
    float y_speed_;
    bool navigating_;
    cv::Mat  map_;
    void draw_system();

public:
    void init();
    void run();
    void set_parameters(slam::Position p, QString filename, double speed);
    inline void setMapFile(QString filename) {map_file_ = filename;}
    inline bool running() const {return this->running_;}
    inline bool started() const {return this->started_;}
    inline bool finished() const {return this->finished_;}
    inline float getSpeed() const {return this->speed_;}
    inline void setRunning(const bool &running) {this->running_ = running;}
    inline void setStarted(const bool &started) {this->started_ = started;}
    inline void setFinished(const bool &finished) {this->finished_ = finished;}
    inline void setSpeed(const float &speed) {this->speed_ = speed;}
    inline void setNavigating(const bool navigating) {this->navigating_ = navigating;}
    inline int width() {return this->map_.cols;}
    inline int height() {return this->map_.rows;}

    void clear();
    void line(const int x0, const int y0, const int x1, const int y1, const int r, const int g, const int b);
    void setValues(const cv::Point & pos, const int & r, const int & g, const int & b);

signals:
    void emit_readings(rplidar_response_measurement_node_hq_t *readings, int count);
    void moved(cv::Point pos);
    void finished_navigation();

public slots:
    void create_readings();
    inline void update_x_speed(float speed){if (started_) x_speed_ = speed;}
    inline void update_y_speed(float speed){if (started_) y_speed_ = speed;}
    void move_robot(int inc_x, int inc_y);
};

#endif // SIM_ENV_H
