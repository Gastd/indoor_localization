/**
 * 
 */
#ifndef FILTER_H
#define FILTER_H

#include <Eigen/Dense>

class Filter
{
public:
    Filter();

    virtual Eigen::MatrixXd getState() = 0;

    ~Filter();

protected:
    virtual void predict() = 0;
    virtual void update() = 0;
};

#endif
