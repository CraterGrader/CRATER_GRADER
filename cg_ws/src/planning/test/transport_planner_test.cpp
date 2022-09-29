#include <gtest/gtest.h>
#include <planning/transport_planner.hpp>
#include <planning/ceres_helloworld.hpp>

TEST(TransportPlannerTest, helloworld){
  float expected = 0.0f;
  float actual = 0.0f;
  float absolute_range = 0.0f;
  EXPECT_NEAR(expected, actual, absolute_range);
}

TEST(TransportPlannerTest, ceres_helloworld){
    //   google::InitGoogleLogging(argv[0]);
    //   // The variable to solve for with its initial value. It will be
    //   // mutated in place by the solver.
    //   double x = 0.5;
    //   const double initial_x = x;
    //   // Build the problem.
    //   Problem problem;
    //   // Set up the only cost function (also known as residual). This uses
    //   // auto-differentiation to obtain the derivative (jacobian).
    //   CostFunction* cost_function =
    //       new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    //   problem.AddResidualBlock(cost_function, nullptr, &x);
    //   // Run the solver!
    //   Solver::Options options;
    //   options.minimizer_progress_to_stdout = true;
    //   Solver::Summary summary;
    //   Solve(options, &problem, &summary);
    //   std::cout << summary.BriefReport() << "\n";
    //   std::cout << "x : " << initial_x << " -> " << x << "\n";

  float expected = 0.0f;
  float actual = 0.0f;
  float absolute_range = 0.0f;
  EXPECT_NEAR(expected, actual, absolute_range);

}