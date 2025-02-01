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
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create terrain
    vehicle::SCMTerrain terrain(&sys);
    terrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.5)));
    terrain.Initialize(15, 15, mesh_resolution);
    terrain.SetSoilParameters(0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4);

    // Create Viper Rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(driver);
    viper.Initialize(ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT));

    // Add Rocks (Positions are now EXACTLY as in your reference code)
    std::vector<ChVector3d> rock_positions = {
        {1.0, -0.5, 0.0}, {-0.5, -0.5, 0.0}, {2.4, 0.4, 0.0}, {0.6, 1.0, 0.0}, {5.5, 1.2, 0.0},
        {1.2, 2.1, 0.0}, {-0.3, -2.1, 0.0}, {0.4, 2.5, 0.0}, {4.2, 1.4, 0.0}, {5.0, 2.4, 0.0},
        {0.6, -1.2, 0.0}, {4.8, -1.2, 0.0}, {2.5, 2.2, 0.0}, {4.7, -2.2, 0.0}, {-1.7, 1.5, 0.0},
        {-2.0, -1.1, 0.0}, {-5.0, -2.1, 0.0}, {1.5, -0.8, 0.0}, {-2.6, 1.6, 0.0}, {-2.0, 1.8, 0.0}
    };

    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChContactMaterial> rockSurfaceMaterial = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

    for (int i = 0; i < 20; i++) {
        std::string rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock" + std::to_string(i % 3 + 1) + ".obj");

        auto rock_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        rock_mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(0.15));
        rock_mesh->RepairDuplicateVertexes(1e-9);

        auto rock_body = chrono_types::make_shared<ChBodyAuxRef>();
        rock_body->SetMass(50.0);
        rock_body->SetPos(rock_positions[i]);
        rock_body->SetFixed(false);

        auto rock_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSurfaceMaterial, rock_mesh, false, false, 0.005);
        rock_body->AddCollisionShape(rock_shape);
        rock_body->EnableCollision(true);

        auto rock_visual = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_visual->SetMesh(rock_mesh);
        rock_body->AddVisualShape(rock_visual);

        sys.Add(rock_body);
        rocks.push_back(rock_body);
    }

    // Sensor Manager
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({-10, 0, 50}, {1.f, 1.f, 1.f}, 1000);
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);

    // Lidar Sensor
    auto offset_pose = ChFrame<>(ChVector3d(1.5, 0, 0.4), QuatFromAngleZ(0));
    auto lidar = chrono_types::make_shared<ChLidarSensor>(
        viper.GetChassis()->GetBody(), 50, offset_pose, 480, 300, (float)(2 * CH_PI), (float)(CH_PI / 12), (float)-CH_PI / 3, 140.0f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(960, 600, 0.25, "Lidar Point Cloud"));
    manager->AddSensor(lidar);

    // Radar Sensor (Matches Reference Code)
    auto radar = chrono_types::make_shared<ChRadarSensor>(
        viper.GetChassis()->GetBody(), 50, offset_pose, 240, 120, (float)(CH_PI / 1.5), (float)(CH_PI / 5), 100.f);
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZReturn>("Front Radar"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZVisualize>(960, 480, 0.2, "Radar View"));
    manager->AddSensor(radar);

    // Camera Sensor
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        viper.GetChassis()->GetBody(), 50, offset_pose, 960, 480, CH_PI / 3);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(960, 480, "Camera View"));
    manager->AddSensor(cam);

    while (true) {
        sys.DoStepDynamics(1e-3);
        manager->Update();
    }

    return 0;
}
