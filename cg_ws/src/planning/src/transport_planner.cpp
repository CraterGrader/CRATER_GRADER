#include <planning/transport_planner.hpp>

namespace cg {
namespace planning {

size_t TransportPlanner::xy_to_index(size_t x, size_t y, size_t width){
    return (x + (y * width));
}

cg_msgs::msg::Pose2D TransportPlanner::getGoalPose(const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::SiteMap &map) {
  (void)map; // TODO: actually use this
  return agent_pose; // TODO: return the actual goal pose
}

using namespace operations_research;
float TransportPlanner::basicExample()
{
  // Example from ortools, should always return optimal value of 4: https://developers.google.com/optimization/introduction/cpp#complete-program
  // Create the linear solver with the GLOP backend.
  std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("GLOP"));

  // Create the variables x and y.
  MPVariable *const x = solver->MakeNumVar(0.0, 1, "x");
  MPVariable *const y = solver->MakeNumVar(0.0, 2, "y");

  LOG(INFO) << "Number of variables = " << solver->NumVariables();

  // Create a linear constraint, 0 <= x + y <= 2.
  MPConstraint *const ct = solver->MakeRowConstraint(0.0, 2.0, "ct");
  ct->SetCoefficient(x, 1);
  ct->SetCoefficient(y, 1);

  LOG(INFO) << "Number of constraints = " << solver->NumConstraints();

  // Create the objective function, 3 * x + y.
  MPObjective *const objective = solver->MutableObjective();
  objective->SetCoefficient(x, 3);
  objective->SetCoefficient(y, 1);
  objective->SetMaximization();

  solver->Solve();

  LOG(INFO) << "Solution:" << std::endl;
  LOG(INFO) << "Objective value = " << objective->Value();
  LOG(INFO) << "x = " << x->solution_value();
  LOG(INFO) << "y = " << y->solution_value();
  return objective->Value();
}

float TransportPlanner::solveEMDtoy(){
    
    TransportNode y1 = {.x = -1, .y = -3.5, .height= 0.3};
    TransportNode y2 = {.x = -1.5, .y = 1.8, .height= 0.4};
    TransportNode y3 = {.x = 2.5, .y = 1.5, .height= 0.3};
    std::vector<TransportNode> source_nodes{y1,y2,y3};
    // n source nodes = 3

    TransportNode x1 = {.x = -2, .y = -3, .height= 0.4};
    TransportNode x2 = {.x = -1, .y = 2, .height= 0.1};
    TransportNode x3 = {.x = 2, .y = 1, .height= 0.3};
    TransportNode x4 = {.x = 3, .y = 1, .height= 0.2};
    std::vector<TransportNode> sink_nodes{x1,x2,x3,x4};    
    // m sink nodes = 4 

    std::vector<float> distance_nodes;
    // source then sink loop makes vector row major order 
    for (size_t i=0; i < source_nodes.size(); ++i) {
        for (size_t j=0; j < sink_nodes.size(); ++j) {
            // TODO, make distance function, common.cpp
            float distance = std::sqrt((pow((source_nodes[i].x - sink_nodes[j].x),2))+(pow((source_nodes[i].y - sink_nodes[j].y),2)));
            distance_nodes.push_back(distance);
        }
    }

    // Declare the solver.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("GLOP"));

    const double infinity = solver->infinity();

    MPVariable *const t11 = solver->MakeNumVar(0.0, infinity, "t11");
    MPVariable *const t12 = solver->MakeNumVar(0.0, infinity, "t12");
    MPVariable *const t13 = solver->MakeNumVar(0.0, infinity, "t13");
    MPVariable *const t14 = solver->MakeNumVar(0.0, infinity, "t14");
    MPVariable *const t21 = solver->MakeNumVar(0.0, infinity, "t21");
    MPVariable *const t22 = solver->MakeNumVar(0.0, infinity, "t22");
    MPVariable *const t23 = solver->MakeNumVar(0.0, infinity, "t23");
    MPVariable *const t24 = solver->MakeNumVar(0.0, infinity, "t24");
    MPVariable *const t31 = solver->MakeNumVar(0.0, infinity, "t31");
    MPVariable *const t32 = solver->MakeNumVar(0.0, infinity, "t32");
    MPVariable *const t33 = solver->MakeNumVar(0.0, infinity, "t33");
    MPVariable *const t34 = solver->MakeNumVar(0.0, infinity, "t34");

    // Define the constraints.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float M = 20;
    int b = 1; 

    // constraint on all transports in transport plan lower bound sink max
    MPConstraint *const c_sink_1 = solver->MakeRowConstraint(-infinity, x1.height);
    c_sink_1->SetCoefficient(t11, 1);
    c_sink_1->SetCoefficient(t21, 1);
    c_sink_1->SetCoefficient(t31, 1);
    MPConstraint *const c_sink_2 = solver->MakeRowConstraint(-infinity, x2.height);
    c_sink_2->SetCoefficient(t12, 1);
    c_sink_2->SetCoefficient(t22, 1);
    c_sink_2->SetCoefficient(t32, 1);
    MPConstraint *const c_sink_3 = solver->MakeRowConstraint(-infinity, x3.height);
    c_sink_3->SetCoefficient(t13, 1);
    c_sink_3->SetCoefficient(t23, 1);
    c_sink_3->SetCoefficient(t33, 1);
    MPConstraint *const c_sink_4 = solver->MakeRowConstraint(-infinity, x4.height);
    c_sink_4->SetCoefficient(t14, 1);
    c_sink_4->SetCoefficient(t24, 1);
    c_sink_4->SetCoefficient(t34, 1);

    // constraint on all transports in transport plan lower bound source max
    MPConstraint *const c_source_1 = solver->MakeRowConstraint(-infinity, y1.height);
    c_source_1->SetCoefficient(t11, 1);
    c_source_1->SetCoefficient(t12, 1);
    c_source_1->SetCoefficient(t13, 1);
    c_source_1->SetCoefficient(t14, 1);
    MPConstraint *const c_source_2 = solver->MakeRowConstraint(-infinity, y2.height);
    c_source_2->SetCoefficient(t21, 1);
    c_source_2->SetCoefficient(t22, 1);
    c_source_2->SetCoefficient(t23, 1);
    c_source_2->SetCoefficient(t24, 1);
    MPConstraint *const c_source_3 = solver->MakeRowConstraint(-infinity, y3.height);
    c_source_3->SetCoefficient(t31, 1);
    c_source_3->SetCoefficient(t32, 1);
    c_source_3->SetCoefficient(t33, 1);
    c_source_3->SetCoefficient(t34, 1);

    // mixted integer constraint sink 
    MPConstraint *const c_mlp_sink_1 = solver->MakeRowConstraint(x1.height-(M*(1-b)), infinity);
    c_mlp_sink_1->SetCoefficient(t11, 1);
    c_mlp_sink_1->SetCoefficient(t21, 1);
    c_mlp_sink_1->SetCoefficient(t31, 1);
    MPConstraint *const c_mlp_sink_2 = solver->MakeRowConstraint(x2.height-(M*(1-b)), infinity);
    c_mlp_sink_2->SetCoefficient(t12, 1);
    c_mlp_sink_2->SetCoefficient(t22, 1);
    c_mlp_sink_2->SetCoefficient(t32, 1);
    MPConstraint *const c_mlp_sink_3 = solver->MakeRowConstraint(x3.height-(M*(1-b)), infinity);
    c_mlp_sink_3->SetCoefficient(t13, 1);
    c_mlp_sink_3->SetCoefficient(t23, 1);
    c_mlp_sink_3->SetCoefficient(t33, 1);
    MPConstraint *const c_mlp_sink_4 = solver->MakeRowConstraint(x4.height-(M*(1-b)), infinity);
    c_mlp_sink_4->SetCoefficient(t14, 1);
    c_mlp_sink_4->SetCoefficient(t24, 1);
    c_mlp_sink_4->SetCoefficient(t34, 1);

    // mixted integer constraint source 
    MPConstraint *const c_mlp_source_1 = solver->MakeRowConstraint(y1.height-(M*b), infinity);
    c_mlp_source_1->SetCoefficient(t11, 1);
    c_mlp_source_1->SetCoefficient(t12, 1);
    c_mlp_source_1->SetCoefficient(t13, 1);
    c_mlp_source_1->SetCoefficient(t14, 1);
    MPConstraint *const c_mlp_source_2 = solver->MakeRowConstraint(y2.height-(M*b), infinity);
    c_mlp_source_2->SetCoefficient(t21, 1);
    c_mlp_source_2->SetCoefficient(t22, 1);
    c_mlp_source_2->SetCoefficient(t23, 1);
    c_mlp_source_2->SetCoefficient(t24, 1);
    MPConstraint *const c_mlp_source_3 = solver->MakeRowConstraint(y3.height-(M*b), infinity);
    c_mlp_source_3->SetCoefficient(t31, 1);
    c_mlp_source_3->SetCoefficient(t32, 1);
    c_mlp_source_3->SetCoefficient(t33, 1);
    c_mlp_source_3->SetCoefficient(t34, 1);

    // Define the objective function.
    // // Create the objective function, 3 * x + y.
    MPObjective *const objective = solver->MutableObjective();
    objective->SetCoefficient(t11, distance_nodes.at(0));
    objective->SetCoefficient(t12, distance_nodes.at(1));
    objective->SetCoefficient(t13, distance_nodes.at(2));    
    objective->SetCoefficient(t14, distance_nodes.at(3));    
    objective->SetCoefficient(t21, distance_nodes.at(4));    
    objective->SetCoefficient(t22, distance_nodes.at(5));    
    objective->SetCoefficient(t23, distance_nodes.at(6));    
    objective->SetCoefficient(t24, distance_nodes.at(7)); 
    objective->SetCoefficient(t31, distance_nodes.at(8)); 
    objective->SetCoefficient(t32, distance_nodes.at(9));
    objective->SetCoefficient(t33, distance_nodes.at(10));
    objective->SetCoefficient(t34, distance_nodes.at(11));

    objective->SetMinimization();

    LOG(INFO) << "Number of variables = " << solver->NumVariables();

    LOG(INFO) << "Number of constraints = " << solver->NumConstraints();


    // Invoke the solver and display the results.
    solver->Solve();
    
    LOG(INFO) << "Solution:" << std::endl;
    LOG(INFO) << "Objective value = " << objective->Value();
    // LOG(INFO) << "x = " << x->solution_value();
    // LOG(INFO) << "y = " << y->solution_value();

    float objective_test = objective->Value();

    return objective_test;

}

float TransportPlanner::solveEMDtoyLoop(){
    
    TransportNode y1 = {.x = -1, .y = -3.5, .height= 0.3};
    TransportNode y2 = {.x = -1.5, .y = 1.8, .height= 0.4};
    TransportNode y3 = {.x = 2.5, .y = 1.5, .height= 0.3};
    std::vector<TransportNode> source_nodes{y1,y2,y3};
    // n source nodes = 3

    TransportNode x1 = {.x = -2, .y = -3, .height= 0.4};
    TransportNode x2 = {.x = -1, .y = 2, .height= 0.1};
    TransportNode x3 = {.x = 2, .y = 1, .height= 0.3};
    TransportNode x4 = {.x = 3, .y = 1, .height= 0.2};
    std::vector<TransportNode> sink_nodes{x1,x2,x3,x4};    
    // m sink nodes = 4 

    std::vector<float> distance_nodes;
    // source then sink loop makes vector row major order 
    for (size_t i=0; i < source_nodes.size(); i++) {
        for (size_t j=0; j < sink_nodes.size(); j++) {
            // TODO, make distance function, common.cpp
            float distance = std::sqrt((pow((source_nodes[i].x - sink_nodes[j].x),2))+(pow((source_nodes[i].y - sink_nodes[j].y),2)));
            distance_nodes.push_back(distance);
        }
    }

    // Declare the solver.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("GLOP"));

    const double infinity = solver->infinity();

    // Create the variables.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    size_t n_policy = source_nodes.size();    
    size_t m_policy = sink_nodes.size();
    size_t num_cells_policy = m_policy * n_policy;


    // do a for loop to create all items in the transport matrix policiy, mxn
    std::vector<MPVariable*> transport_plan;
    std::vector<std::string> varNames;

    for (size_t t = 0; t < num_cells_policy; t++){
        std::string numVarName = 't' + std::to_string(t);
        varNames.push_back(numVarName);
        transport_plan.push_back(solver->MakeNumVar(0.0, infinity, numVarName));
    }


    // Define the constraints.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float M = 20;
    int b = 1; 

    std::vector<MPConstraint*> constraints_sink_lb;

    // constraint on all transports in transport plan lower bound sink max
    for (size_t j = 0; j < m_policy; j++){
        // create a constraint for each sink
        constraints_sink_lb.push_back(solver->MakeRowConstraint(-infinity, sink_nodes.at(j).height));
        for (size_t i = 0; i < n_policy; i++){
            // this should capture all of the transport colmns 
            size_t index = xy_to_index(j, i, m_policy);
            constraints_sink_lb.at(j)->SetCoefficient(transport_plan.at(index), 1);
        }
    }

    std::vector<MPConstraint*> constraints_source_lb;
    // constraint on all transports in transport plan lower bound source max
    for (size_t i = 0; i < n_policy; i++){
        // create a constraint for each sink
        constraints_source_lb.push_back(solver->MakeRowConstraint(-infinity, source_nodes.at(i).height));
        for (size_t j = 0; j < m_policy; j++){
            // want all the rows
            size_t index = xy_to_index(j, i, m_policy);
            constraints_source_lb.at(i)->SetCoefficient(transport_plan.at(index), 1);
        }
    }


    std::vector<MPConstraint*> constraints_sink_milp;
    // mixted integer constraint sink
    for (size_t j = 0; j < m_policy; j++){
        // create a constraint for each sink
        constraints_sink_milp.push_back(solver->MakeRowConstraint((sink_nodes.at(j).height-(M*(1-b))), infinity));
        for (size_t i = 0; i < n_policy; i++){
            size_t index = xy_to_index(j, i, m_policy);
            constraints_sink_milp.at(j)->SetCoefficient(transport_plan.at(index), 1);
        }
    }

   
    std::vector<MPConstraint*> constraints_source_milp;
    // constraint on all transports in transport plan lower bound source max
    for (size_t i = 0; i < n_policy; i++){
        // create a constraint for each sink
        constraints_source_milp.push_back(solver->MakeRowConstraint(source_nodes.at(i).height-(M*b), infinity));
        for (size_t j = 0; j < m_policy; j++){
            size_t index = xy_to_index(j, i, m_policy);
            constraints_source_milp.at(i)->SetCoefficient(transport_plan.at(index), 1);
        }
    }

    MPObjective *const objective = solver->MutableObjective();
    for (size_t t = 0; t < num_cells_policy; t++){
        objective->SetCoefficient(transport_plan.at(t), distance_nodes.at(t));
    }

    objective->SetMinimization();

    LOG(INFO) << "Number of variables = " << solver->NumVariables();
    LOG(INFO) << "Number of constraints = " << solver->NumConstraints();

    // Invoke the solver and display the results.
    solver->Solve();
    
    LOG(INFO) << "Solution:" << std::endl;
    LOG(INFO) << "Objective value = " << objective->Value();

    float objective_test = objective->Value();


    for (size_t t = 0; t < num_cells_policy; t++){
        LOG(INFO) << varNames.at(t) << transport_plan.at(t)->solution_value();
    }


    return objective_test;
}

float TransportPlanner::solveEMDhardMap(){

    // declare height-map vector 
    std::vector<float> heightMapStandIn{0,    0,    0,    0,    0,    0,    0,    0,    0, 
                                        0,    0,    0,    0,    0,    0,    0,    0,    0,
                                        0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,
                                        0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,
                                        0,    0,  1.0, -2.0, -4.0, -2.0,  1.0,    0,    0,
                                        0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,
                                        0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,
                                        0,    0,    0,    0,    0,    0,    0,    0,    0,
                                        0,    0,    0,    0,    0,    0,    0,    0,    0};

    std::vector<float> designTOPO{0,    0,    0,    0,    0,    0,    0,    0,    0,
                                    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                    0,    0,    0,    0,    0,    0,    0,    0,    0,
                                    0,    0,    0,    0,    0,    0,    0,    0,    0, 
                                    0,    0,    0,    0,    0,    0,    0,    0,    0};
    
    float threshold_z = 0.001;
    
    float vol_sink = 0.0f;
    float vol_source = 0.0f;

    // ---------------------------------------------------
    // TODO turn this into a function 

    // declare source vector 
    std::vector<TransportNode> source_nodes;
    
    // declare sink vector 
    std::vector<TransportNode> sink_nodes;

    // todo get from map object
    size_t map_height = 9;
    size_t map_width = 9;
    size_t num_cells = map_height*map_width;
    float resolution = 0.1;

    // Loop through height map and assign points to either source or sink
    for (size_t i = 0; i < num_cells; i++) {

        // Get associated coordinates with these indices
        // TODO, make map util
        float x = (i % map_width)*resolution;
        float y = (std::floor(i / map_width))*resolution;

        float height = heightMapStandIn.at(i);

        TransportNode node;

        // Positive height becomes a source; positive volume in +z
        if (height > (designTOPO.at(i)+threshold_z)){
            node.x = x; 
            node.y = y; 
            node.height = height;
            vol_source += height;
            source_nodes.push_back(node);
        }

        // # Negative height becomes a sink; for solver the sinks also must have positive volume, defined as positive in -z
        if (height < designTOPO.at(i)-threshold_z){
            node.x = x; 
            node.y = y; 
            node.height = -height;
            vol_sink += -height;
            sink_nodes.push_back(node);
        }

    }
    // ---------------------------------------------------

    
    std::vector<float> distance_nodes;
    // source then sink loop makes vector row major order 
    for (size_t i=0; i < source_nodes.size(); i++) {
        for (size_t j=0; j < sink_nodes.size(); j++) {
            // TODO, make distance function, common.cpp
            float distance = std::sqrt((pow((source_nodes[i].x - sink_nodes[j].x),2))+(pow((source_nodes[i].y - sink_nodes[j].y),2)));
            distance_nodes.push_back(distance);
        }
    }

    // Declare the solver.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("GLOP"));

    const double infinity = solver->infinity();

    // Create the variables.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    size_t n_policy = source_nodes.size();    
    size_t m_policy = sink_nodes.size();
    size_t num_cells_policy = m_policy * n_policy;


    // do a for loop to create all items in the transport matrix policiy, mxn
    std::vector<MPVariable*> transport_plan;
    std::vector<std::string> varNames;

    for (size_t t = 0; t < num_cells_policy; t++){
        std::string numVarName = 't' + std::to_string(t);
        varNames.push_back(numVarName);
        transport_plan.push_back(solver->MakeNumVar(0.0, infinity, numVarName));
    }


    // Define the constraints.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float M = std::max(vol_sink, vol_source);
    int b; 

    // todo make better 
    if (vol_sink > vol_source){
        b = 0;
    }
    else{
        b = 1;
    }

    std::vector<MPConstraint*> constraints_sink_lb;

    // constraint on all transports in transport plan lower bound sink max
    for (size_t j = 0; j < m_policy; j++){
        // create a constraint for each sink
        constraints_sink_lb.push_back(solver->MakeRowConstraint(-infinity, sink_nodes.at(j).height));
        for (size_t i = 0; i < n_policy; i++){
            // this should capture all of the transport colmns 
            size_t index = xy_to_index(j, i, m_policy);
            constraints_sink_lb.at(j)->SetCoefficient(transport_plan.at(index), 1);
        }
    }

    std::vector<MPConstraint*> constraints_source_lb;
    // constraint on all transports in transport plan lower bound source max
    for (size_t i = 0; i < n_policy; i++){
        // create a constraint for each sink
        constraints_source_lb.push_back(solver->MakeRowConstraint(-infinity, source_nodes.at(i).height));
        for (size_t j = 0; j < m_policy; j++){
            // want all the rows
            size_t index = xy_to_index(j, i, m_policy);
            constraints_source_lb.at(i)->SetCoefficient(transport_plan.at(index), 1);
        }
    }


    std::vector<MPConstraint*> constraints_sink_milp;
    // mixted integer constraint sink
    for (size_t j = 0; j < m_policy; j++){
        // create a constraint for each sink
        constraints_sink_milp.push_back(solver->MakeRowConstraint((sink_nodes.at(j).height-(M*(1-b))), infinity));
        for (size_t i = 0; i < n_policy; i++){
            size_t index = xy_to_index(j, i, m_policy);
            constraints_sink_milp.at(j)->SetCoefficient(transport_plan.at(index), 1);
        }
    }

   
    std::vector<MPConstraint*> constraints_source_milp;
    // constraint on all transports in transport plan lower bound source max
    for (size_t i = 0; i < n_policy; i++){
        // create a constraint for each sink
        constraints_source_milp.push_back(solver->MakeRowConstraint(source_nodes.at(i).height-(M*b), infinity));
        for (size_t j = 0; j < m_policy; j++){
            size_t index = xy_to_index(j, i, m_policy);
            constraints_source_milp.at(i)->SetCoefficient(transport_plan.at(index), 1);
        }
    }

    MPObjective *const objective = solver->MutableObjective();
    for (size_t t = 0; t < num_cells_policy; t++){
        objective->SetCoefficient(transport_plan.at(t), distance_nodes.at(t));
    }

    objective->SetMinimization();

    LOG(INFO) << "Number of variables = " << solver->NumVariables();
    LOG(INFO) << "Number of constraints = " << solver->NumConstraints();

    // Invoke the solver and display the results.
    solver->Solve();
    
    LOG(INFO) << "Solution:" << std::endl;
    LOG(INFO) << "Objective value = " << objective->Value();

    float objective_test = objective->Value();


    for (size_t t = 0; t < num_cells_policy; t++){
        LOG(INFO) << varNames.at(t) << transport_plan.at(t)->solution_value();
    }


    return objective_test;
}

} // namespace planning
} // namespace cg
