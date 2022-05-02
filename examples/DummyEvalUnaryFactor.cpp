#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <cstring>
#include <fstream>
#include <iostream>

using namespace gtsam;
using namespace std;

class UnaryFactor : public NoiseModelFactor1<Pose2> {
  public:
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel &model)
        : NoiseModelFactor1<Pose2>(model, j), mX(x), mY(y) {}

    inline Vector evaluateError(const Pose2 &q, boost::optional<Matrix &> H = boost::none) const {
        const Rot2 &R = q.rotation();
        if (H)
            (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0., R.s(), R.c(), 0.).finished();
        return (Vector(2) << q.x() - mX, q.y() - mY).finished();
    }

  private:
    double mX, mY; // measurements
};