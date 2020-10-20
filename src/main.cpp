#include <Eigen/Core>


//#include "custom_types/vertex_pose.h"
//#include "custom_types/edge_pose_pose.h"
//#include "custom_types/register_types.h"

//#include "custom_types/camera_projection.h"


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam2d/se2.h>

//#include <g2o/types/slam2d/vertex_point_xy.h>
//#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/apps/g2o_viewer/run_g2o_viewer.h>



#include <iostream>
#include <iomanip>
#include <fstream>
#include<string>


#include <istream>

// Landmark generation parameters
const int NUM_POSES = 20;
const int NUM_CONNECTIONS_PER_VERTEX = 4;

// noise to be added to initial estimates
const double POSE_NOISE = 10.0;

// noise to be added to relative pose edge measurement
const double EDGE_NOISE = 0.05;

// should be done as singleton
class UniqueId
{
public:
  UniqueId():unique_id(0){}
  int getUniqueId()
  {
    return unique_id++;
  }
private:
  int unique_id;
};
static UniqueId uniqueId;

/*
double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void addNoiseToTranslation(Eigen::Isometry3d & trafo, const double noise)
{
  trafo.translation()[0] += fRand(-noise, noise);
  trafo.translation()[1] += fRand(-noise, noise);
  trafo.translation()[2] += fRand(-noise, noise);
}


Eigen::Isometry3d isometryFromArray7D(const double* v)
{
  Eigen::Isometry3d result;

  result = Eigen::Quaterniond(v[6], v[3], v[4], v[5]).normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
  return result;
}

void generatePoses(std::vector<Eigen::Isometry3d>& pose_list)
{
  // TODO
  //1. retrive poses from text file
  //2. convert rotation to quaternions
  //3. make psotion and quaternion into a single double array
  //4. push them into pose_list
  const double step_arr [7] = {0.0, 0.0, 10.0, 0.0, -0.05, 0.0, 1.0};
  const double initial_arr [7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  Eigen::Isometry3d step = isometryFromArray7D(step_arr);
  pose_list.front() = isometryFromArray7D(initial_arr);
  for(std::vector<Eigen::Isometry3d>::iterator itr = pose_list.begin() + 1; itr != pose_list.end(); itr++)
  {
    Eigen::Isometry3d & cur_trafo = *itr;
    Eigen::Isometry3d & prev_trafo = *(itr-1);
    cur_trafo = prev_trafo * step;
  }
}


void addPosesToGraph(std::vector<Eigen::Isometry3d>& pose_list, std::vector<int>& poseNum2Id, g2o::OptimizableGraph* graph_ptr)
{
  for(std::vector<Eigen::Isometry3d>::const_iterator itr = pose_list.begin(); itr != pose_list.end(); itr++)
  {
    const Eigen::Isometry3d & pose = *itr;
    VertexPose * v = new VertexPose();

    // add some noise to translation component
    Eigen::Isometry3d pose_estimate(pose);
    addNoiseToTranslation(pose_estimate, POSE_NOISE);

    // get id
    const int id = uniqueId.getUniqueId();
    poseNum2Id.push_back(id);

    // populate g2o vertex object and add to graph
    v->setEstimate(pose_estimate);
    v->setId(id);
    v->setFixed(false);
    graph_ptr->addVertex(v);
  }
}
*/

void addPosesToGraph(std::vector<g2o::SE2>& pose_list, std::vector<int>& poseNum2Id, g2o::OptimizableGraph* graph_ptr)
{
  for(std::vector<g2o::SE2>::const_iterator itr = pose_list.begin(); itr != pose_list.end(); itr++)
  {
    //const Eigen::Isometry3d & pose = *itr;
    //VertexPose * v = new VertexPose();

    // add some noise to translation component
    //Eigen::Isometry3d pose_estimate(pose);
    //addNoiseToTranslation(pose_estimate, POSE_NOISE);

    const g2o::SE2 & pose = *itr;
    g2o::VertexSE2* v = new g2o::VertexSE2;

    // get id
    const int id = uniqueId.getUniqueId();
    poseNum2Id.push_back(id);

    // populate g2o vertex object and add to graph
    v->setEstimate(pose);
    v->setId(id);
    v->setFixed(false);
    graph_ptr->addVertex(v);
  }
  std::cout << "added all poses to the graph" << std::endl;
}


void addEdge(const int id_a, const int id_b,
             g2o::OptimizableGraph* graph_ptr)
{
  //EdgePosePose * e = new EdgePosePose;
  g2o::EdgeSE2* e = new g2o::EdgeSE2;

  // retrieve vertex pointers from graph with id's
  g2o::OptimizableGraph::Vertex * pose_a_vertex
        = dynamic_cast<g2o::OptimizableGraph::Vertex*>
          (graph_ptr->vertices()[id_a]);

  g2o::OptimizableGraph::Vertex * pose_b_vertex
        = dynamic_cast<g2o::OptimizableGraph::Vertex*>
           (graph_ptr->vertices()[id_b]);

  std::vector<double> data1,data2;

  pose_a_vertex->getEstimateData(data1);
  pose_b_vertex->getEstimateData(data2);

  g2o::SE2 vertex1(data1[0], data1[1], data1[2]);
  g2o::SE2 vertex2(data2[0], data2[1], data2[2]);
  g2o::SE2 transform = vertex1.inverse() * vertex2;

  // error check vertices and associate them with the edge
  assert(pose_a_vertex!=NULL);
  assert(pose_a_vertex->dimension() == 3);
  e->vertices()[0] = pose_a_vertex;

  assert(pose_b_vertex!=NULL);
  assert(pose_b_vertex->dimension() == 3);
  e->vertices()[1] = pose_b_vertex;

  // add information matrix
  Eigen::Matrix<double, 3, 3> Lambda;
  Lambda.setIdentity();

  // set the observation and imformation matrix
  e->setMeasurement(transform);
  e->information() = Lambda;

  // finally add the edge to the graph
  if(!graph_ptr->addEdge(e))
  {
    assert(false);
  }
  std::cout << "edge added: " << id_a << " - " << id_b << std::endl;
}
/*
void addOdomVertex(double x, double y, double theta, int id, bool first)
{
    g2o::SE2 pose(x, y, theta);
    g2o::VertexSE2* vertex = new g2o::VertexSE2;
    vertex->setId(id);
    vertex->setEstimate(pose);
    graph_.addVertex(vertex);
    vertex_set_.insert(vertex);
    robot_pose_ids_.push_back(id);
    if(first)
        vertex->setFixed(true);
}


void addOdomEdge(int id1, int id2)
{
    std::vector<double> data1,data2;

    graph_.vertex(id1)->getEstimateData(data1);
    graph_.vertex(id2)->getEstimateData(data2);

    g2o::SE2 vertex1(data1[0], data1[1], data1[2]);
    g2o::SE2 vertex2(data2[0], data2[1], data2[2]);

    g2o::SE2 transform = vertex1.inverse() * vertex2;
    g2o::EdgeSE2* edge = new g2o::EdgeSE2;
    edge->vertices()[0] = graph_.vertex(id1);
    edge->vertices()[1] = graph_.vertex(id2);
    edge->setMeasurement(transform);
    edge->setInformation(odom_noise_);

    graph_.addEdge(edge);
    edge_set_.insert(edge);
    std::cout << "added odometry edge: " << id1 << " - " << id2 << std::endl;
}
*/

std::vector<std::string> split(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

std::vector<g2o::SE2> getPoseListfromFile(const std::string& path, std::vector<int> fileColumnIdx, int lineSkip)
{
  std::ifstream inFile;
    
    inFile.open(path);
    if (!inFile) {
        std::cout << "Unable to open file!\n";
        exit(1); // terminate with error
    }

    std::cout << "file opened successfully!\n";
    std::string s;

    //std::vector<std::vector<double>> pose_list;
    std::vector<g2o::SE2> pose_list;
    int i=0;
    while (std::getline(inFile, s))	
    {
      if(i > lineSkip)
      {
        std::vector<std::string> results;
        //std::vector<double> curr_pose;
        results = split(s,' ');
        
        double x = std::stod(results.at(fileColumnIdx[0]));
        double y = std::stod(results.at(fileColumnIdx[1]));
        double theta = std::stod(results.at(fileColumnIdx[2]));
        
        g2o::SE2 curr_pose(x, y, theta);

        pose_list.push_back(curr_pose);
      }
      i++;
    }
    inFile.close();
    return pose_list;
}

/*
int run(int argc, char** argv, CommandArgs& arg)
{
   std::string inputFilename;
   arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);
   arg.parseArgs(argc, argv);
 
   MainWindow mw;
   mw.updateDisplayedSolvers();
   mw.updateRobustKernels();
   mw.show();
 
   // redirect the output that normally goes to cerr to the textedit in the viewer
   StreamRedirect redirect(std::cerr, mw.plainTextEdit);
 
   // setting up the optimizer
   SparseOptimizer* optimizer = new SparseOptimizer();
   mw.viewer->graph = optimizer;
 
   // set up the GUI action
   GuiHyperGraphAction guiHyperGraphAction;
   guiHyperGraphAction.viewer = mw.viewer;
   //optimizer->addPostIterationAction(&guiHyperGraphAction);
   optimizer->addPreIterationAction(&guiHyperGraphAction);
 
   if (inputFilename.size() > 0) {
     mw.loadFromFile(QString::fromStdString(inputFilename));
   }
 
   QCoreApplication* myapp = QApplication::instance();
   while (mw.isVisible()) {
     guiHyperGraphAction.dumpScreenshots = mw.actionDump_Images->isChecked();
     if (myapp)
       myapp->processEvents();
     SleepThread::msleep(10);
   }
 
   delete optimizer;
   return 0;
}
*/ 



int main(int argc, char** argv)
{
  //init_g2o_types();
 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // initialize graph

  g2o::SparseOptimizer graph;

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // poses
  std::cout << "Generating Poses...\n";
  std::string poseFilePath = "../../MARS/LongB_cmb_hz_100Hz_TSA.txt";

  std::vector<int> fileColumnIdx={5,4,9};
  int lineSkip=6;
  std::vector<g2o::SE2> pose_list = getPoseListfromFile(poseFilePath, fileColumnIdx, lineSkip);
  std::cout << pose_list[0].translation()[0] <<std::endl;

  //std::vector<Eigen::Isometry3d> pose_list(NUM_POSES);
  //generatePoses(pose_list);

  

  std::vector<int> poseNum2Id;  //used to associate pose number to g2o vertex id
  addPosesToGraph(pose_list, poseNum2Id, &graph);

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Edges
  std::cout << "Generating edges...\n";

  // iterator over poses
  for(std::vector<g2o::SE2>::const_iterator itr = pose_list.begin(); itr != pose_list.end()-1; itr++)
  {
    /*
    const Eigen::Isometry3d & w_T_a = *itr;
    for(int i = 1; i <= NUM_CONNECTIONS_PER_VERTEX; i++)
    {
      std::vector<Eigen::Isometry3d>::const_iterator itr_other = itr + i;
      if(itr_other < pose_list.end())
      {
        const int pose_a_id = poseNum2Id[itr - pose_list.begin()];
        const int pose_b_id = poseNum2Id[itr_other - pose_list.begin()];
        const Eigen::Isometry3d & w_T_b = *itr_other;

        Eigen::Isometry3d a_T_b = w_T_a.inverse() * w_T_b;
        //addNoiseToTranslation(a_T_b, EDGE_NOISE);
        addEdge(a_T_b, pose_a_id, pose_b_id, &graph);
      }
      
      
    }
    */
    const int pose_a_id = poseNum2Id[itr - pose_list.begin()];
    const int pose_b_id = poseNum2Id[itr+1 - pose_list.begin()];
    addEdge(pose_a_id, pose_b_id, &graph);
  }

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // finished populating graph, export and quit
  graph.initializeOptimization();
  std::cout << "Saving Graph to example_g2o.g2o...\n";
  graph.save("example_g2o.g2o");
  std::cout << "Finished\n";
  //std::cout << "./g2o_viewer";
  //std::system("./g2o_viewer");
  //g2o_viewer example_g2o.g2o\n";
  
  
}