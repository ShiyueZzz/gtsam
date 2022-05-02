#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/dataset.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

// For loading the data
#include "SFMdata.h"
// Camera observations of landmarks will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::L; // for landmarks
using symbol_shorthand::X; // for poses

int main(int argc, char *argv[]) {

    /**
     * Modeling a simple  factor graph
     * f0(x1)->X1->f1(x1, x2;o1)->X2->f2(x2, x3;o2)->X3
     * X1-3: robot pose over time
     * o1-2: odometry measurements
     **/
    NonlinearFactorGraph graph;

    // Add a Gaussian prior on pose x_1
    Pose2 priorMean(0., 0., 0.);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

    // Add two odometry factors
    Pose2 odometry(2., 0., 0.);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, odometry, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, odometry, odometryNoise);
    graph.print("\nFactor Graph:\n"); // print
}