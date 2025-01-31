// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Integrated Script
// =============================================================================
//
// Demo showing Viper Rover on SCM Terrain with obstacles, sensors, and ROS integration.
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::viper;
using namespace chrono::sensor;
using namespace chrono::ros;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
double mesh_resolution = 0.02;

int main(int argc, char* argv[]) {
    std::cout << "Chrono-ROS Integrated Viper Simulation\nChrono Version: " << CHRONO_VERSION << std::endl;

    // Create the Chrono physical system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    std::cout << "Terrain done" << std::endl;

    // Create the terrain
    vehicle::SCMTerrain terrain(&sys);
    terrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.5)));
    terrain.Initialize(15, 15, mesh_resolution);
    terrain.SetSoilParameters(0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4);

    std::cout << "Creating rover" << std::endl;
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(driver);
    viper.Initialize(ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT));

    std::cout << "Manager start" << std::endl;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    std::cout << "Scene setup" << std::endl;
    manager->scene->AddPointLight({-10, 0, 50}, {1.f,1.f,1.f}, 1000);
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);

    std::cout << "Adding sensors" << std::endl;
    auto offset_pose = ChFrame<>(ChVector3d(1.5, 0, 0.4), QuatFromAngleZ(0));

    auto lidar = chrono_types::make_shared<ChLidarSensor>(
        viper.GetChassis()->GetBody(), 50, offset_pose, 480, 300, 
        (float)(2 * CH_PI), (float)(CH_PI / 12), (float)-CH_PI / 3, 140.0f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(960, 600, 0.25, "Lidar Point Cloud"));
    manager->AddSensor(lidar);

    auto radar = chrono_types::make_shared<ChRadarSensor>(
        viper.GetChassis()->GetBody(), 50, offset_pose, 240, 120, 
        (float)(CH_PI / 1.5), (float)(CH_PI / 5), 100.f);
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZReturn>("Radar Data"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZVisualize>(960, 480, 0.2, "Radar View"));
    manager->AddSensor(radar);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        viper.GetChassis()->GetBody(), 50, offset_pose, 960, 480, CH_PI / 3);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(960, 480, "Camera View"));
    manager->AddSensor(cam);

    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSViperDCMotorControlHandler>(25, driver, "~/input/driver_inputs"));
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSBodyHandler>(25, viper.GetChassis()->GetBody(), "~/output/rover/state"));
    ros_manager->Initialize();

    while (ros_manager->Update(sys.GetChTime(), 1e-3)) {
        sys.DoStepDynamics(1e-3);
        viper.Update();
        manager->Update();
    }

    return 0;
}