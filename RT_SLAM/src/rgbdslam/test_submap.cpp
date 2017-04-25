#include "OpenniWrapper.h"
#include <string>
#include <sstream>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "qt_gui.h"
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include <cv_bridge/cv_bridge.h>
#include "parameter_server.h"
#include "misc.h"
#include "ros_service_ui.h"
#include "GraphWrapper.h"
#include "DuoGraph.h"

using namespace std;

///Connect Signals and Slots for the ui control
void ui_connections(QObject* ui, GraphManager* graph_mgr, OpenNIListener* listener)
{
  Qt::ConnectionType ctype = Qt::AutoConnection;
  if (ParameterServer::instance()->get<bool>("concurrent_io")) 
    ctype = Qt::DirectConnection;
  QObject::connect(ui, SIGNAL(reset()), graph_mgr, SLOT(reset()), ctype);
  QObject::connect(ui, SIGNAL(optimizeGraph()), graph_mgr, SLOT(optimizeGraph()), ctype);
  QObject::connect(ui, SIGNAL(togglePause()), listener, SLOT(togglePause()), ctype);
  QObject::connect(ui, SIGNAL(toggleBagRecording()), listener, SLOT(toggleBagRecording()), ctype);
  QObject::connect(ui, SIGNAL(getOneFrame()), listener, SLOT(getOneFrame()), ctype);
  QObject::connect(ui, SIGNAL(deleteLastFrame()), graph_mgr, SLOT(deleteLastFrame()), ctype);
  QObject::connect(ui, SIGNAL(sendAllClouds()), graph_mgr, SLOT(sendAllClouds()), ctype);
  QObject::connect(ui, SIGNAL(saveAllClouds(QString)), graph_mgr, SLOT(saveAllClouds(QString)), ctype);
  // QObject::connect(ui, SIGNAL(saveOctomapSig(QString)), graph_mgr, SLOT(saveOctomap(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveAllFeatures(QString)), graph_mgr, SLOT(saveAllFeatures(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveIndividualClouds(QString)), graph_mgr, SLOT(saveIndividualClouds(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveTrajectory(QString)), graph_mgr, SLOT(saveTrajectory(QString)), ctype);
  QObject::connect(ui, SIGNAL(toggleMapping(bool)), graph_mgr, SLOT(toggleMapping(bool)), ctype);
  QObject::connect(ui, SIGNAL(saveG2OGraph(QString)), graph_mgr, SLOT(saveG2OGraph(QString)), ctype);
}

///Connect Signals and Slots only relevant for the graphical interface
void gui_connections(Graphical_UI* gui, GraphManager* graph_mgr, OpenNIListener* listener)
{
  QObject::connect(listener,  SIGNAL(newVisualImage(QImage)), gui, SLOT(setVisualImage(QImage)));
  QObject::connect(listener,  SIGNAL(newFeatureFlowImage(QImage)), gui, SLOT(setFeatureFlowImage(QImage)));
  QObject::connect(listener,  SIGNAL(newDepthImage(QImage)), gui, SLOT(setDepthImage(QImage)));
  QObject::connect(graph_mgr, SIGNAL(sendFinished()), gui, SLOT(sendFinished()));
  QObject::connect(graph_mgr, SIGNAL(iamBusy(int, const char*, int)), gui, SLOT(showBusy(int, const char*, int)));
  QObject::connect(graph_mgr, SIGNAL(progress(int, const char*, int)), gui, SLOT(setBusy(int, const char*, int)));
  QObject::connect(graph_mgr, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
  QObject::connect(graph_mgr, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
  QObject::connect(gui, SIGNAL(printEdgeErrors(QString)), graph_mgr, SLOT(printEdgeErrors(QString)));
  QObject::connect(gui, SIGNAL(pruneEdgesWithErrorAbove(float)), graph_mgr, SLOT(pruneEdgesWithErrorAbove(float)));
  QObject::connect(gui, SIGNAL(clearClouds()), graph_mgr, SLOT(clearPointClouds()));
  if (ParameterServer::instance()->get<bool>("use_glwidget") && gui->getGLViewer() != NULL) {
    GLViewer* glv = gui->getGLViewer();
    QObject::connect(graph_mgr, SIGNAL(setPointCloud(pointcloud_type *, QMatrix4x4)), glv, SLOT(addPointCloud(pointcloud_type *, QMatrix4x4)), Qt::BlockingQueuedConnection ); //Needs to block, otherwise the opengl list compilation makes the app unresponsive. This effectively throttles the processing rate though
    typedef const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* cnst_ft_vectors;
    QObject::connect(graph_mgr, SIGNAL(setFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*)), glv, SLOT(addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*))); //, Qt::DirectConnection);
    QObject::connect(graph_mgr, SIGNAL(setGraphEdges(const QList<QPair<int, int> >*)), glv, SLOT(setEdges(const QList<QPair<int, int> >*)));
    QObject::connect(graph_mgr, SIGNAL(updateTransforms(QList<QMatrix4x4>*)), glv, SLOT(updateTransforms(QList<QMatrix4x4>*)));
    QObject::connect(graph_mgr, SIGNAL(deleteLastNode()), glv, SLOT(deleteLastNode()));
    QObject::connect(graph_mgr, SIGNAL(resetGLViewer()),  glv, SLOT(reset()));
    if(!ParameterServer::instance()->get<bool>("store_pointclouds")) {
        QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type const *)), graph_mgr, SLOT(clearPointCloud(pointcloud_type const *))); // 
    } else if(ParameterServer::instance()->get<double>("voxelfilter_size") > 0.0) {
        QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type const *)), graph_mgr, SLOT(reducePointCloud(pointcloud_type const *))); // 
    }
  }
  QObject::connect(listener, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
  QObject::connect(listener, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
  QObject::connect(graph_mgr, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
}

int main(int argc, char* argv[])
{
  //create thread object, to run the ros event processing loop in parallel to the qt loop
  QtROS qtRos(argc, argv, "rgbdslam"); //ros node name & namespace

  //Depending an use_gui on the Parameter Server, a gui- or a headless application is used
  QApplication app(argc, argv, ParameterServer::instance()->get<bool>("use_gui")); 
  
  ParameterServer * p = ParameterServer::instance();
  // p->set<std::string>("bagfile_name", "/home/davidz/work/ros/fuetre/stacks/rgbdslam/bags/rgbd_dataset_freiburg2_large_with_loop.bag");
  p->set<std::string>("bagfile_name", "/home/davidz/work/ros/fuetre/stacks/rgbdslam/bags/rgbd_dataset_freiburg3_long_office_household.bag");
  // p->set<std::string>("bagfile_name", "/home/davidz/work/ros/fuetre/stacks/rgbdslam/bags/rgbd_dataset_freiburg1_xyz.bag");
  // p->set<std::string>("bagfile_name", "/home/davidz/work/exprdata/3DKinect/rgbd_dataset_freiburg1_desk.bag");
  p->set<std::string>("topic_image_mono", "/camera/rgb/image_color");
  p->set<std::string>("topic_image_depth", "/camera/depth/image");
  p->set<int>("optimizer_skip_step", -1);
  
  // GraphManager graph_mgr;
  // CGraphWrapper graph_mgr;
  CDuoGraph graph_mgr;
  //Instantiate the kinect image listener
  // OpenNIListener listener(&graph_mgr);
  COpenniWrapper* listerner_w = new COpenniWrapper(&graph_mgr);
  OpenNIListener* listener = static_cast<OpenNIListener*>(listerner_w);
  // OpenNIListener* listener = new OpenNIListener(&graph_mgr);
  std::string bagfile_name = ParameterServer::instance()->get<std::string>("bagfile_name");
  if(!bagfile_name.empty()) 
  {
    QObject::connect(listener, SIGNAL(bagFinished()), &app, SLOT(quit()));
    QObject::connect(listener, SIGNAL(bagFinished()), &qtRos, SLOT(quitNow()));
    QtConcurrent::run(listener, &OpenNIListener::loadBag, bagfile_name);
  }

  Graphical_UI* gui = NULL;
	if (app.type() == QApplication::GuiClient){
      gui = new Graphical_UI();
      gui->show();
      gui_connections(gui, graph_mgr.getFrontGraph(), listener);
      ui_connections(gui, graph_mgr.getFrontGraph(), listener);//common connections for the user interfaces

      gui_connections(gui, graph_mgr.getBackGraph(), listener);
      // ui_connections(gui, graph_mgr.getBackGraph(), listener);//common connections for the user interfaces

      // gui_connections(gui, &graph_mgr, listener);
      // ui_connections(gui, &graph_mgr, listener);//common connections for the user interfaces
  } else {
      ROS_WARN("Running without graphical user interface! See README or wiki page for how to interact with RGBDSLAM.");
  }
  //Create Ros service interface with or without gui
  RosUi ui("rgbdslam"); //ui namespace for service calls
  ui_connections(&ui, graph_mgr.getFrontGraph(), listener);
  // ui_connections(&ui, &graph_mgr, listener);//common connections for the user interfaces

  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));

  qtRos.start();// Run main loop.
  app.exec();

  return 0;
}


/*   string path("./data1/");
    std_msgs::Header header;
    for(int i=1;i<=6;i++)
    {
        stringstream s1,s2;
        s1<<i<<"depth.jpg";
        s2<<i<<"image.jpg";
        cv::Mat img, dpt, dpt_8mono;
        dpt = cv::imread(s1.str().c_str());
        img = cv::imread(s2.str().c_str());
        depthToCV8UC1(dpt, dpt_8mono);
        cv_bridge::CvImage dpt_pImg(header, "mono8", dpt_8mono);
        cv_bridge::CvImage img_pImg(header, "rgb8", img);
        sensor_msgs::ImagePtr fdpt = dpt_pImg.toImageMsg();
        sensor_msgs::ImagePtr fimg = img_pImg.toImageMsg();
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = 
        // color_pc_ptr pc = createXYZRGBPointCloud()
    }
    */
