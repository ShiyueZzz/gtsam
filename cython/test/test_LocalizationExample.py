import unittest
from gtsam import *
from math import *
import numpy as np

class TestLocalizationExample(unittest.TestCase):

    def test_LocalizationExample(self):
        # Create the graph (defined in pose2SLAM.h, derived from
        # NonlinearFactorGraph)
        graph = NonlinearFactorGraph()

        # Add two odometry factors
        # create a measurement for both factors (the same in this case)
        odometry = Pose2(2.0, 0.0, 0.0)
        odometryNoise = noiseModel_Diagonal.Sigmas(
            np.array([0.2, 0.2, 0.1]))  # 20cm std on x,y, 0.1 rad on theta
        graph.add(BetweenFactorPose2(0, 1, odometry, odometryNoise))
        graph.add(BetweenFactorPose2(1, 2, odometry, odometryNoise))

        # Add three "GPS" measurements
        # We use Pose2 Priors here with high variance on theta
        groundTruth = Values()
        groundTruth.insertPose2(0, Pose2(0.0, 0.0, 0.0))
        groundTruth.insertPose2(1, Pose2(2.0, 0.0, 0.0))
        groundTruth.insertPose2(2, Pose2(4.0, 0.0, 0.0))
        model = noiseModel_Diagonal.Sigmas(np.array([0.1, 0.1, 10.]))
        for i in range(3):
            graph.add(PriorFactorPose2(i, groundTruth.atPose2(i), model))

        # Initialize to noisy points
        initialEstimate = Values()
        initialEstimate.insertPose2(0, Pose2(0.5, 0.0, 0.2))
        initialEstimate.insertPose2(1, Pose2(2.3, 0.1, -0.2))
        initialEstimate.insertPose2(2, Pose2(4.1, 0.1, 0.1))

        # Optimize using Levenberg-Marquardt optimization with an ordering from
        # colamd
        optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate)
        result = optimizer.optimizeSafely()

        # Plot Covariance Ellipses
        marginals = Marginals(graph, result)
        P = [None] * result.size()
        for i in range(0, result.size()):
            pose_i = result.atPose2(i)
            self.assertTrue(pose_i.equals(groundTruth.atPose2(i), 1e-4))
            P[i] = marginals.marginalCovariance(i)

if __name__ == "__main__":
    unittest.main()