// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo illkustrating the co-simulation of a Chrono wheeled vehicle FMU, 4 tire
// FMUs, and a path-follower driver FMU.
//
// =============================================================================

#include <array>

#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicleFMI.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fmi/fmi2/ChFmuToolsImport.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------

std::string VEHICLE_FMU_MODEL_IDENTIFIER = "FMU2_WheeledVehicle";
std::string TIRE_FMU_MODEL_IDENTIFIER = "FMU2_ForceElementTire";
std::string DRIVER_FMU_MODEL_IDENTIFIER = "FMU2_PathFollowerDriver";

std::string VEHICLE_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + VEHICLE_FMU_MODEL_IDENTIFIER + std::string("/");
std::string TIRE_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + TIRE_FMU_MODEL_IDENTIFIER + std::string("/");
std::string DRIVER_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + DRIVER_FMU_MODEL_IDENTIFIER + std::string("/");

std::string VEHICLE_FMU_FILENAME = VEHICLE_FMU_DIR + VEHICLE_FMU_MODEL_IDENTIFIER + std::string(".fmu");
std::string TIRE_FMU_FILENAME = TIRE_FMU_DIR + TIRE_FMU_MODEL_IDENTIFIER + std::string(".fmu");
std::string DRIVER_FMU_FILENAME = DRIVER_FMU_DIR + DRIVER_FMU_MODEL_IDENTIFIER + std::string(".fmu");

std::string VEHICLE_UNPACK_DIR =
    CHRONO_VEHICLE_FMU_DIR + std::string("tmp_unpack_") + VEHICLE_FMU_MODEL_IDENTIFIER + std::string("/");
std::string TIRE_UNPACK_DIR =
    CHRONO_VEHICLE_FMU_DIR + std::string("tmp_unpack_") + TIRE_FMU_MODEL_IDENTIFIER + std::string("/");
std::string DRIVER_UNPACK_DIR =
    CHRONO_VEHICLE_FMU_DIR + std::string("tmp_unpack_") + DRIVER_FMU_MODEL_IDENTIFIER + std::string("/");

// -----------------------------------------------------------------------------

void CreateVehicleFMU(FmuChronoUnit& vehicle_fmu,
                      const std::string& instance_name,
                      double step_size,
                      double start_time,
                      double stop_time,
                      const std::vector<std::string>& logCategories,
                      const std::string& out_path,
                      bool visible,
                      double fps) {
    try {
        vehicle_fmu.Load(fmi2Type::fmi2CoSimulation, VEHICLE_FMU_FILENAME, VEHICLE_UNPACK_DIR);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Vehicle FMU version:  " << vehicle_fmu.GetVersion() << std::endl;
    std::cout << "Vehicle FMU platform: " << vehicle_fmu.GetTypesPlatform() << std::endl;

    // Instantiate FMU
    vehicle_fmu.Instantiate(instance_name, false, visible);

    // Set debug logging
    vehicle_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    vehicle_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                                start_time,             // start time
                                fmi2False, stop_time);  // use stop time

    // Set I/O fixed parameters
    vehicle_fmu.SetVariable("out_path", out_path);

    // Set fixed parameters - use vehicle JSON files from the Chrono::Vehicle data directory
    std::string data_path = "../data/vehicle/";
    std::string vehicle_JSON = vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json");
    std::string engine_JSON = vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineShafts.json");
    std::string transmission_JSON = vehicle::GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json");

    vehicle_fmu.SetVariable("data_path", data_path);
    vehicle_fmu.SetVariable("vehicle_JSON", vehicle_JSON);
    vehicle_fmu.SetVariable("engine_JSON", engine_JSON);
    vehicle_fmu.SetVariable("transmission_JSON", transmission_JSON);
    vehicle_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);

    ////ChVector3d g_acc(0, 0, 0);
    ////vehicle_fmu.SetVecVariable("g_acc", g_acc);
}

// -----------------------------------------------------------------------------

void CreateDriverFMU(FmuChronoUnit& driver_fmu,
                     const std::string& instance_name,
                     double step_size,
                     double start_time,
                     double stop_time,
                     const std::vector<std::string>& logCategories,
                     const std::string& out_path,
                     bool visible,
                     double fps) {
    try {
        driver_fmu.Load(fmi2Type::fmi2CoSimulation, DRIVER_FMU_FILENAME, DRIVER_UNPACK_DIR);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Driver FMU version:  " << driver_fmu.GetVersion() << "\n";
    std::cout << "Driver FMU platform: " << driver_fmu.GetTypesPlatform() << "\n";

    // Instantiate FMU
    driver_fmu.Instantiate(instance_name, false, visible);

    // Set debug logging
    driver_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    driver_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                               start_time,             // start time
                               fmi2False, stop_time);  // use stop time

    // Set I/O fixed parameters
    driver_fmu.SetVariable("out_path", out_path);

    // Set fixed parameters
    std::string path_file = vehicle::GetDataFile("paths/ISO_double_lane_change.txt");
    ////std::string path_file = vehicle::GetDataFile("paths/ISO_double_lane_change2.txt");
    double throttle_threshold = 0.2;
    double look_ahead_dist = 5.0;

    driver_fmu.SetVariable("path_file", path_file);
    driver_fmu.SetVariable("throttle_threshold", throttle_threshold, FmuVariable::Type::Real);
    driver_fmu.SetVariable("look_ahead_dist", look_ahead_dist, FmuVariable::Type::Real);
    driver_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);
}

// -----------------------------------------------------------------------------

void CreateTireFMU(FmuChronoUnit& tire_fmu,
                   const std::string& instance_name,
                   double step_size,
                   double start_time,
                   double stop_time,
                   const std::vector<std::string>& logCategories,
                   const std::string& out_path) {
    try {
        tire_fmu.Load(fmi2Type::fmi2CoSimulation, TIRE_FMU_FILENAME, TIRE_UNPACK_DIR);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Tire FMU version:  " << tire_fmu.GetVersion() << std::endl;
    std::cout << "Tire FMU platform: " << tire_fmu.GetTypesPlatform() << std::endl;

    // Instantiate FMU
    tire_fmu.Instantiate(instance_name);

    // Set debug logging
    tire_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    tire_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                             start_time,             // start time
                             fmi2False, stop_time);  // use stop time
    // Set I/O fixed parameters
    tire_fmu.SetVariable("out_path", out_path);

    // Set fixed parameters
    std::string tire_JSON = vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json");

    tire_fmu.SetVariable("tire_JSON", tire_JSON);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Names of FMU instances
    std::string vehicle_instance_name = "WheeledVehicleFmuComponent";
    std::string driver_instance_name = "DriverFmuComponent";
    std::string tire_instance_name = "HandlingTireFmuComponent";

    // Create (if needed) output directories
    std::string out_dir = GetChronoOutputPath() + "./DEMO_WHEELEDVEHICLE_FMI_COSIM_B";
    std::string vehicle_out_dir = out_dir + "/" + vehicle_instance_name;
    std::string driver_out_dir = out_dir + "/" + driver_instance_name;
    std::string tire_out_dir = out_dir + "/" + tire_instance_name;

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(vehicle_out_dir))) {
        std::cout << "Error creating directory " << vehicle_out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(driver_out_dir))) {
        std::cout << "Error creating directory " << driver_out_dir << std::endl;
        return 1;
    }
    for (int i = 0; i < 4; i++) {
        if (!filesystem::create_directory(filesystem::path(tire_out_dir + "_" + std::to_string(i)))) {
            std::cout << "Error creating directory " << tire_out_dir + "_" + std::to_string(i) << std::endl;
            return 1;
        }  
    }

    std::vector<std::string> logCategories = {"logAll"};

    double start_time = 0;
    double stop_time = 16;
    double step_size = 1e-3;

    bool vehicle_visible = true;
    bool driver_visible = true;

    double fps = 60;
    bool save_img = false;

    // Create the 2 FMUs
    ////std::cout << "Vehicle FMU dir: >" << VEHICLE_FMU_DIR << "<" << std::endl;
    ////std::cout << "Tire FMU dir:    >" << TIRE_FMU_DIR << "<" << std::endl;
    ////std::cout << "Driver FMU dir:  >" << DRIVER_FMU_DIR << "<" << std::endl;
    ////std::cout << "Vehicle FMU filename: >" << VEHICLE_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Tire FMU filename:    >" << TIRE_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Driver FMU filename:  >" << DRIVER_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Vehicle FMU unpack directory: >" << VEHICLE_UNPACK_DIR << "<" << std::endl;
    ////std::cout << "Tire FMU unpack directory:    >" << TIRE_UNPACK_DIR << "<" << std::endl;
    ////std::cout << "Driver FMU unpack directory:  >" << DRIVER_UNPACK_DIR << "<" << std::endl;

    FmuChronoUnit vehicle_fmu;
    FmuChronoUnit driver_fmu;
    FmuChronoUnit tire_fmu[4];
    try {
        CreateVehicleFMU(vehicle_fmu, vehicle_instance_name, step_size, start_time, stop_time, logCategories,
                         vehicle_out_dir, vehicle_visible, fps);
    } catch (std::exception& e) {
        std::cout << "ERROR loading vehicle FMU: " << e.what() << "\n";
        return 1;
    }
    try {
        CreateDriverFMU(driver_fmu, driver_instance_name, step_size, start_time, stop_time, logCategories,
                        driver_out_dir, driver_visible, fps);
    } catch (std::exception& e) {
        std::cout << "ERROR loading driver FMU: " << e.what() << "\n";
        return 1;
    }
    for (int i = 0; i < 4; i++) {
        try {
            CreateTireFMU(tire_fmu[i], tire_instance_name + "_" + std::to_string(i), step_size, start_time, stop_time,
                          logCategories, tire_out_dir + "_" + std::to_string(i));
        } catch (std::exception& e) {
            std::cout << "ERROR loading tire FMU: " << e.what() << "\n";
            return 1;
        }
    }

    // Initialize FMUs
    driver_fmu.EnterInitializationMode();
    driver_fmu.ExitInitializationMode();

    vehicle_fmu.EnterInitializationMode();
    {
        // Set initial vehicle location
        ChVector3d init_loc;
        double init_yaw;
        driver_fmu.GetVecVariable("init_loc", init_loc);
        driver_fmu.GetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
        vehicle_fmu.SetVecVariable("init_loc", init_loc);
        vehicle_fmu.SetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
    }
    vehicle_fmu.ExitInitializationMode();

    for (int i = 0; i < 4; i++) {
        tire_fmu[i].EnterInitializationMode();
        tire_fmu[i].ExitInitializationMode();
    }

    // Create terrain
    FlatTerrain terrain(0.0, 0.8f);

    // Initialize output
    utils::ChWriterCSV csv;
    csv.SetDelimiter(" ");

    // Enable/disable saving snapshots
    vehicle_fmu.SetVariable("save_img", save_img);
    driver_fmu.SetVariable("save_img", save_img);

     // Simulation loop
    std::string wheel_id[4] = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};

    double time = 0;
    ChTimer timer;
    timer.start();

    while (time < stop_time) {
        std::cout << "\r" << time << "\r";

        // ----------- Set FMU control variables
        double target_speed = 12;
        driver_fmu.SetVariable("target_speed", target_speed, FmuVariable::Type::Real);

        // --------- Exchange data between vehicle and driver FMUs
        double steering;
        double throttle;
        double braking;
        driver_fmu.GetVariable("steering", steering, FmuVariable::Type::Real);
        driver_fmu.GetVariable("throttle", throttle, FmuVariable::Type::Real);
        driver_fmu.GetVariable("braking", braking, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("steering", steering, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("throttle", throttle, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("braking", braking, FmuVariable::Type::Real);

        ChFrameMoving<> ref_frame;
        vehicle_fmu.GetFrameMovingVariable("ref_frame", ref_frame);
        driver_fmu.SetFrameMovingVariable("ref_frame", ref_frame);

        // ----------- Exchange data between vehicle and tire FMUs
        for (int i = 0; i < 4; i++) {
            WheelState state;
            vehicle_fmu.GetVecVariable(wheel_id[i] + ".pos", state.pos);
            vehicle_fmu.GetQuatVariable(wheel_id[i] + ".rot", state.rot);
            vehicle_fmu.GetVecVariable(wheel_id[i] + ".lin_vel", state.lin_vel);
            vehicle_fmu.GetVecVariable(wheel_id[i] + ".ang_vel", state.ang_vel);
            tire_fmu[i].SetVecVariable("wheel_state.pos", state.pos);
            tire_fmu[i].SetQuatVariable("wheel_state.rot", state.rot);
            tire_fmu[i].SetVecVariable("wheel_state.lin_vel", state.lin_vel);
            tire_fmu[i].SetVecVariable("wheel_state.ang_vel", state.ang_vel);

            TerrainForce load;
            tire_fmu[i].GetVecVariable("wheel_load.point", load.point);
            tire_fmu[i].GetVecVariable("wheel_load.force", load.force);
            tire_fmu[i].GetVecVariable("wheel_load.moment", load.moment);
            vehicle_fmu.SetVecVariable(wheel_id[i] + ".point", load.point);
            vehicle_fmu.SetVecVariable(wheel_id[i] + ".force", load.force);
            vehicle_fmu.SetVecVariable(wheel_id[i] + ".moment", load.moment);
        }

        // ------------ Set terrain information for each tire FMU
        for (int i = 0; i < 4; i++) {
            ChVector3d query_point;
            tire_fmu[i].GetVecVariable("query_point", query_point);

            double terrain_height = terrain.GetHeight(query_point);
            ChVector3d terrain_normal = terrain.GetNormal(query_point);
            double terrain_mu = (double)terrain.GetCoefficientFriction(query_point);
            tire_fmu[i].SetVariable("terrain_height", terrain_height, FmuVariable::Type::Real);
            tire_fmu[i].SetVecVariable("terrain_normal", terrain_normal);
            tire_fmu[i].SetVariable("terrain_mu", terrain_mu, FmuVariable::Type::Real);
        }

        // ----------- Save output
        csv << time << ref_frame.GetPos() << std::endl;

        // ----------- Advance FMUs
        auto status_vehicle = vehicle_fmu.DoStep(time, step_size, fmi2True);
        auto status_driver = driver_fmu.DoStep(time, step_size, fmi2True);
        if (status_vehicle == fmi2Discard || status_driver == fmi2Discard)
            break;
        for (int i = 0; i < 4; i++) {
            auto status_tire = tire_fmu[i].DoStep(time, step_size, fmi2True);
            if (status_tire == fmi2Discard)
                break;
        }

        time += step_size;
    }

    timer.stop();
    std::cout << "Sim time: " << time << std::endl;
    std::cout << "Run time: " << timer() << std::endl;

    std::string out_file = out_dir + "/vehicle.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/vehicle.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("x");
    gplot.SetLabelY("y");
    gplot.SetTitle("Vehicle path");
    gplot.Plot(out_file, 2, 3, "path", " with lines lt 1 lw 2");
#endif

    return 0;
}
