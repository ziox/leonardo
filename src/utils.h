#ifndef UTILS_H
#define UTILS_H

#include <tf/transform_datatypes.h>
#include <cmath>


template <int N>
class MAFilter
{
public:
    MAFilter()
    {}

    inline double operator()(double in)
    {
        double out = 0.0;
        for(int i = 0; i < N - 1; ++i) {
            values[i] = values[i + 1];
            out += values[i];
        }
        values[N - 1] = in;
        out += in;
        return out / N;
    }

private:
    double values[N];
};

template <int N>
class MAVectorFilter
{
public:
    MAVectorFilter()
    {
    }

    inline tf::Vector3 filter(tf::Vector3 in)
    {
        tf::Vector3 out(0, 0, 0);
        for(int i = 0; i < N - 1; ++i) {
            values[i] = values[i + 1];
            out += values[i];
        }
        values[N - 1] = in;
        out += in;
        return out / N;
    }

private:
    tf::Vector3 values[N];
};


struct Limit
{
    Limit(double limit)
        : limit(limit)
    {}

    double operator()(double value)
    {
        return std::max(std::min(value, limit), -limit);
    }

private:
    const double limit;
};

struct Threshold
{
    Threshold(double threshold)
        : threshold(threshold)
    {}

    double operator()(double value)
    {
        return fabs(value) > threshold ? value : 0.0;
    }

private:
    const double threshold;
};


class MotionProfile
{
public:
    MotionProfile() = default;
    virtual ~MotionProfile() = default;

    virtual double position(double t) const = 0;
};

class SinProfile : public MotionProfile
{
public:
    double position(double t) const override
    {
        return 0.5 + std::sin(3.14 * t - 1.56) / 2.0;
    }
};

class SuperProfile : public MotionProfile
{
public:
    double position(double t) const override
    {
        if (t < 1./3.)
        {
            return 9./4. * t * t;
        }
        else if (t < 2./3.)
        {
            return 1./4. + 3./2. * (t - 1./3.);
        }
        else if (t < 1.)
        {
            return -9./4. * t * t + 9./2. * t - 5./4.;
        }
        else
        {
            return 1.0;
        }
    }
};

#endif
