#ifndef PX4_VEHICLE_UTILS_HPP 
#define PX4_VEHICLE_UTILS_HPP

#include "pxr/usd/usd/stage.h"
#include <pxr/usd/usdUtils/stageCache.h>
#include <omni/fabric/usd/PathConversion.h>

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/scenegraph/usd/usd/prim.h>
#include <usdrt/scenegraph/base/gf/vec3f.h>
#include <usdrt/scenegraph/base/gf/quatf.h>
#include <usdrt/scenegraph/usd/usd/tokens.h>
#include <usdrt/scenegraph/usd/sdf/path.h>
#include <usdrt/scenegraph/usd/usd/attribute.h>
#include <usdrt/gf/vec.h>


#include <usdrt/scenegraph/usd/usdPhysics/revoluteJoint.h>
#include <usdrt/scenegraph/usd/usdPhysics/rigidBodyAPI.h>
#include <usdrt/scenegraph/usd/physxSchema/physxDeformableAPI.h>
#include <usdrt/scenegraph/usd/physxSchema/physxForceAPI.h>
#include <usdrt/scenegraph/usd/usdPhysics/articulationRootAPI.h>
#include <usdrt/scenegraph/usd/usdPhysics/driveAPI.h>

#include <algorithm>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <execution>
#include <regex>
#include <array>
#include <random>
#include <iostream>

#include "px4_math_utils.hpp"
#include "xoshiro256ss.h"


namespace isaac::px4_sitl::bridge{


static usdrt::UsdStageRefPtr getActiveStage(){
    const std::vector<PXR_NS::UsdStageRefPtr> stages = PXR_NS::UsdUtilsStageCache::Get().GetAllStages();
    if (stages.size() != 1){
        return nullptr;
    }
    auto stage_id = PXR_NS::UsdUtilsStageCache::Get().GetId(stages[0]).ToLongInt();
    return usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stage_id));
}



class Px4Multirotor{
    public:
        Px4Multirotor(){
            rotor_joint_group_.reserve(16);
            rotor_body_group_.reserve(16);
            actuator_joint_group_.reserve(6);
            actuator_body_group_.reserve(6);
        }
        ~Px4Multirotor(){}

        struct VehicleParameters{
            float mass = 0.0f;                         // Vehicle mass in kg.
            usdrt::GfVec3f diagonal_inertias{0, 0, 0}; // Diagonal inertias of the vehicle (Ixx, Iyy, Izz) in kg*m^2.
            usdrt::GfVec3f center_of_mass{0, 0, 0};    // Center of mass of the vehicle (x, y, z) in meters.

            // the params for motors from the motor thrust curve of the required BLDC manufacturer
            float moment_constants = 0.0000001738f; // Moment constants
            float force_constants = 0.00001211f;    // Force constants
            float max_rotor_velocity = 1032.0f;     // Maximum rotor velocity in rad/s.
            float params_std_dev = 0.05f;           // Standard deviation
            float rotor_time_constants = 0.05f;     // Time constants

            // servo specs
            float max_torque = 0.11f; // Maximum torque in kg*m^-1
            float max_angle = 180.0f; // Maximum angle in degrees
            float servo_time_constants = 0.2f;     // Time constants

            // https://ardupilot.org/copter/docs/airspeed-estimation.html
            usdrt::GfVec3f rotor_drag_coef{0, 0, 0}; // Rotor drag coefficients
            float airframe_drag = 0.20f;             // Airframe drag coefficient
        };
        struct VehicleMotion{
            usdrt::GfVec3d translate{0.0, 0.0, 0.0}; // drone/UAV pos (m)
            usdrt::GfQuatd orient{1.0, 0.0, 0.0, 0.0}; // drone/UAV orientations in quaternion (w,x,y,z)
            usdrt::GfVec3f velocity{0.0, 0.0, 0.0}; // drone/UAV velocity (m/s)
            usdrt::GfVec3f angular_velocity{0.0, 0.0, 0.0}; // drone/UAV rot rates (deg/s)
        };
        struct ActuatorControl{
            // reserve control cmds and setpoints for max allowed number of rotors/active joints
            std::array<float, 16> actuator_cmd;
            std::array<float, 16> actuator_setpoint; // why array though why not vector ?? (optimal ??)
        };
        VehicleParameters &getParameters() { return parameters_; }
        size_t getRotorCount()const{ return rotor_joint_group_.size(); }
        size_t getActuatorCount()const{ return actuator_joint_group_.size(); }
       
       
        // reset vehicle and its params on reset scene (one iteration block)
        const void reset(){
            setVehicleForceAndTorque(usdrt::GfVec3f(0, 0, 0), usdrt::GfVec3f(0, 0, 0)); // set vechicle for and torque here after complete that func
            // reset joint velocity and forces
            for(auto joint_itr=rotor_joint_group_.begin(); joint_itr!=rotor_joint_group_.end(); ++joint_itr){
                size_t idx = joint_itr-rotor_joint_group_.begin();
                setRotorVelocity(0, static_cast<int>(idx));
            };
            for(auto body_itr=rotor_body_group_.begin(); body_itr!=rotor_body_group_.end(); ++body_itr){
                size_t idx = body_itr-rotor_body_group_.begin();
                setRotorForce(0, static_cast<int>(idx));
            };
            vehicle_base_link_ = usdrt::UsdPrim{nullptr};
            vehicle_articulation_root_ = usdrt::UsdPrim{nullptr};
            rotor_joint_group_.clear();
            rotor_body_group_.clear();
            actuator_joint_group_.clear();
            actuator_body_group_.clear();
        }

        /**
         * @brief Load a vehicle from USD stage.
         *
         * It expects the vehicle to be represented as an articulation root with a base link
         * as a child. The base link is expected to be a RigidBody with a name matching the
         * pattern "base[_]?link.*".
         *
         * @param target The path to the vehicle prim in the USD stage.
         * @return true if the vehicle is successfully loaded, false otherwise.
         */
        bool loadVehicle(const omni::fabric::PathC &target){
            auto stage = getActiveStage();
            if(!stage){
                return false;
            }
            const std::string path = omni::fabric::toSdfPath(target).GetString();
            usdrt::UsdPrim prim = stage->GetPrimAtPath(target);
            std::cout << "Loading UAV Target from prim path: [" << path << "]" << std::endl;
            if(!prim){
                return false;
            }

            const usdrt::TfToken rigidBodyAPI = usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType();
            const usdrt::TfToken articulationRootAPI = usdrt::UsdPhysicsArticulationRootAPI::_GetStaticTfType();
            std::regex pattern("[bB]ase[_]?[lL]ink.*"); // looks for "base_link" or "Base_Link"
            // check for baselinks and articualtion root
            if(prim.HasAPI(articulationRootAPI)){
                for(auto& child: prim.GetAllChildren()){
                    if(child.HasAPI(rigidBodyAPI) && std::regex_match(child.GetName().GetText(), pattern)){
                        vehicle_articulation_root_ = prim;
                        vehicle_base_link_ = child;
                        return initVehicle();
                    }
                }
            }else if(prim.HasAPI(rigidBodyAPI) && std::regex_match(prim.GetName().GetText(), pattern)){
                // check if parent has articulation root
                auto parent = prim.GetParent();
                if(parent.HasAPI(articulationRootAPI)){
                    vehicle_articulation_root_= parent;
                    vehicle_base_link_ = prim;
                    return initVehicle();
                }
            }
            CARB_LOG_WARN("%s is not base_link or does not have RigidBody/ArticulationRoot API.", path.c_str());
            return false;
        }

        bool isVehiclePathEqual(omni::fabric::PathC& target){
            if(!vehicle_articulation_root_ || !vehicle_base_link_){
                return false;
            }
            std::string target_path = omni::fabric::toSdfPath(target).GetString();
            std::string vehicle_root_path = vehicle_articulation_root_.GetPrimPath().GetString();
            std::string vehicle_baselink_path = vehicle_base_link_.GetPrimPath().GetString();
            return vehicle_root_path == target_path || vehicle_baselink_path == target_path;
        }

        const void getVehicleMotion(VehicleMotion& motions){
            if(!vehicle_base_link_.IsValid()){return;}
            vehicle_base_link_.GetAttribute(usdrt::TfToken("xformOp:translate")).Get<usdrt::GfVec3d>(&motions.translate);
            vehicle_base_link_.GetAttribute(usdrt::TfToken("xformOp:orient")).Get<usdrt::GfQuatd>(&motions.orient);
            vehicle_base_link_.GetAttribute(usdrt::TfToken("physics:velocity")).Get<usdrt::GfVec3f>(&motions.velocity);
            vehicle_base_link_.GetAttribute(usdrt::TfToken("physics:angularVelocity")).Get<usdrt::GfVec3f>(&motions.angular_velocity);
        }

        const void getRotorTorqueAxis(const float &moment, usdrt::GfVec3f &torque_axis, int index){
            if(rotor_body_group_.empty() || index < 0 || index >= static_cast<int>(rotor_body_group_.size())){return;}
            usdrt::GfQuatd orient(0, 0, 0, 0);
            rotor_body_group_[index].GetAttribute(usdrt::TfToken("xformOp:orient")).Get<usdrt::GfQuatd>(&orient);
            torque_axis = moment * getQuatAxis(static_cast<usdrt::GfQuatf>(orient));
        }

        // Black Magic Voodoo Drone Dynamics
        void computeDynamics(ActuatorControl &m_actuator_control, std::vector<int> &dir){
            // params setup and seed init
            std::random_device rd;
            xoshiro256ss rng(rd());
            std::normal_distribution<float> vel_normal_dist(parameters_.max_rotor_velocity, parameters_.max_rotor_velocity * parameters_.params_std_dev);
            std::normal_distribution<float> km_normal_dist(parameters_.moment_constants, parameters_.moment_constants * parameters_.params_std_dev);
            std::normal_distribution<float> kf_normal_dist(parameters_.force_constants, parameters_.force_constants * parameters_.params_std_dev);
            std::normal_distribution<float> tau_normal_dist(parameters_.rotor_time_constants, parameters_.rotor_time_constants * parameters_.params_std_dev);
            const float omega = vel_normal_dist(rng);
            const float omega_squared = std::pow(omega, 2.0f);
            std::vector<usdrt::GfVec3f> rotor_torque(getRotorCount());
            const int unused_cmd = 16 - getRotorCount() - getActuatorCount();

            for(size_t idx=0; idx<m_actuator_control.actuator_cmd.size()-unused_cmd; ++idx){
                const float &cmd = m_actuator_control.actuator_cmd[idx];
                float tau = std::clamp(tau_normal_dist(rng), 0.0f, 1.0f);
                if(idx < getRotorCount()){
                    // update for rotors (exponential smoothing)
                    // current_setpoint = prev_setpoint + tau * (sqrt(|cmd|) - prev_setpoint) 
                    m_actuator_control.actuator_setpoint[idx] += tau * (std::sqrt(std::abs(cmd)) - m_actuator_control.actuator_setpoint[idx]); // clever smooth updation 
                    float throttle = m_actuator_control.actuator_setpoint[idx];
                    float rotor_velocity = dir[idx]*omega*throttle;
                    if(throttle > 5 * parameters_.params_std_dev){
                        // if throttle inputs are small (under std_dev tol) ignore updation
                        setRotorVelocity(rotor_velocity, idx);
                    }
                    float throttle_squared = std::pow(throttle, 2.0f);
                    float rotor_thrust = throttle_squared * std::abs(kf_normal_dist(rng) * omega_squared);
                    
                    // ---------------- DEBUG ----------------
                    std::cout << "rotor_" << idx << " velocity: " << rotor_velocity << std::endl;
                    std::cout << "rotor" << idx << " thrust: " << rotor_thrust << std::endl;
                    // ---------------------------------------

                    if(throttle > 5 * parameters_.params_std_dev){
                        // ignore thrust update if throttle is too small
                        setRotorForce(rotor_thrust, idx);
                    }

                    float rotor_moment = -dir[idx] * throttle_squared * std::abs(km_normal_dist(rng) * omega_squared);
                    usdrt::GfVec3f torque_axis(0, 0, 0);
                    getRotorTorqueAxis(rotor_moment, torque_axis, idx);
                    rotor_torque[idx] = torque_axis;
                }else if(idx >= getRotorCount() && idx < getRotorCount()+getActuatorCount()){
                    // update for generic actuators (s-curve)
                    // current_setpoint = lerp(prev_setpoint, target_setpoint, smoothstep)
                    float smoothstep = parameters_.servo_time_constants * parameters_.servo_time_constants * (3 - 2 * parameters_.servo_time_constants); // lerp update rate
                    m_actuator_control.actuator_setpoint[idx] = lerp(m_actuator_control.actuator_setpoint[idx], parameters_.max_angle * cmd, smoothstep);
                    setActuatorPosition(m_actuator_control.actuator_setpoint[idx], idx - getRotorCount());
                    setActuatorForce(parameters_.max_torque, idx - getRotorCount());
                }
                
            }

            usdrt::GfVec3f velocity(0.0f, 0.0f, 0.0f);
            getVehicleVelocity(velocity);
            usdrt::GfMatrix3f rotMatrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            getVehicleRotationMatrix(rotMatrix);

            usdrt::GfVec3f body_drag = -1.0f * parameters_.airframe_drag * velocity.GetLength() * velocity;
            usdrt::GfVec3f rotor_drag = -1.0f * rotMatrix * parameters_.rotor_drag_coef * rotMatrix.GetTranspose() * velocity.GetLength();
            /** @note
            / We are applying additional body drag assuming similar reference area in X,Y plane and first-order rotor drag to the vehicle base link.
            / The collective thrusts are applied to center of mass of the vehicle and the gravity is already acting on the rigid body.
            / see https://discuss.px4.io/t/wind-estimator-tuning-equation/28053 and https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf
            / Should consider https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph/node-library/nodes/omni-physx-forcefields/omni-physx-forcefields-forcefieldwind-1.html?
            / Global downwash and equilibrium actuator action acting on main body is not taken into account, assuming negligible.
            */
            usdrt::GfVec3f force_ = body_drag + rotor_drag;
            usdrt::GfVec3f torque_ = std::reduce(rotor_torque.begin(), rotor_torque.end());

            setVehicleForceAndTorque(force_, torque_, true);
        }

        std::vector<int> getAirframeConfig(const std::string &airframe){
            std::unordered_map<std::string, std::vector<int>> air_frame_configs = {
                {"QuadX", {1, 1, -1, -1}},
                {"HexX", {-1, 1, -1, 1, 1, -1}},
                {"OctX", {-1, -1, 1, 1, 1, 1, -1, -1}}
            };
            auto itr = air_frame_configs.find(airframe);
            if(itr!=air_frame_configs.end()){
                return air_frame_configs[airframe];
            }
            return {};
        }

private:
    std::vector<usdrt::UsdPrim> rotor_joint_group_;
    std::vector<usdrt::UsdPrim> rotor_body_group_;
    std::vector<usdrt::UsdPrim> actuator_joint_group_;
    std::vector<usdrt::UsdPrim> actuator_body_group_;
    VehicleParameters parameters_;
    usdrt::UsdPrim vehicle_articulation_root_;
    usdrt::UsdPrim vehicle_base_link_;

    /**
     * Initializes the vehicle by setting up force API, mass, inertia, and rotor groups.
     *
     * @return True if the rotor joint group is not empty and matches the
     *         size of the rotor body group, otherwise false.
     */
    bool initVehicle(){
        setForceAPI(vehicle_base_link_, true); // for this global frame of reference for forces
        setMassAndInertia();
        setRotorActuatorGroup();

        return (!rotor_joint_group_.empty()) &&
                (rotor_joint_group_.size()==rotor_body_group_.size()) &&
                (actuator_joint_group_.size()==actuator_body_group_.size());
    }

    // -------------- HELPER FUNCS -------------------
    void setRotorActuatorGroup(){
        if (!vehicle_base_link_.IsValid() || !vehicle_articulation_root_.IsValid()){
            return;
        }

        rotor_joint_group_.clear();
        rotor_body_group_.clear();
        actuator_joint_group_.clear();
        actuator_body_group_.clear();
        std::regex rotor_pattern("[rR]otor[_]?.*"); // eg(ein Bispiel): rotor0, rotor1, etc.... 
        std::regex actuator_pattern("[aA]ctuator[_]?.*");

        // process all child links and extract and set force apis to the rotor and actuators
        auto processChild = [&](const usdrt::UsdPrim& child){
            // std::cout << "child component name: [" << child.GetName().GetText() << "]" << std::endl;
            if(std::regex_match(child.GetName().GetText(), rotor_pattern)){
                // check for revolute joints
                if(child.IsA(usdrt::UsdPhysicsRevoluteJoint::_GetStaticTfType())){
                    std::cout << "child component name: [" << child.GetName().GetText() << "]" << std::endl;
                    rotor_joint_group_.emplace_back(child);
                }
                // set forces to body
                if(child.IsA(usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType()) || child.HasAPI(usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType())){
                    std::cout << "child component name: [" << child.GetName().GetText() << "]" << std::endl;
                    rotor_body_group_.emplace_back(child);
                    setForceAPI(child); // set force here after the func is complete
                }
            }
            if(std::regex_match(child.GetName().GetText(), actuator_pattern)){
                if(child.IsA(usdrt::UsdPhysicsRevoluteJoint::_GetStaticTfType())){
                    actuator_joint_group_.emplace_back(child);
                }
                if(child.IsA(usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType())){
                    actuator_body_group_.emplace_back(child);
                    setForceAPI(child); // set force api here after
                }
                // soft body actuator (if any)
                if(child.IsA(usdrt::PhysxSchemaPhysxDeformableAPI::_GetStaticTfType())){
                    actuator_body_group_.emplace_back(child);
                }
            }
        };
        for(const auto &child: vehicle_base_link_.GetAllChildren()){
            processChild(child);
        }
        for(const auto &child: vehicle_articulation_root_.GetAllChildren()){
            processChild(child);
        }
    }
    
    // set mass and inertia of vehicle
    void setMassAndInertia(){
        if(!vehicle_base_link_.IsValid()){
            return;
        }
        usdrt::TfToken mass = usdrt::TfToken("physics:mass");
        if(vehicle_base_link_.HasAttribute(mass)){
            vehicle_base_link_.GetAttribute(mass).Get<float>(&parameters_.mass);
        }
        usdrt::TfToken com = usdrt::TfToken("physics:center_of_mass");
        if (vehicle_base_link_.HasAttribute(com)){
            vehicle_base_link_.GetAttribute(com).Get<usdrt::GfVec3f>(&parameters_.center_of_mass);
        }
        usdrt::TfToken inertia = usdrt::TfToken("physics:diagonalInertia");
        if(vehicle_base_link_.HasAttribute(inertia)){
            vehicle_base_link_.GetAttribute(inertia).Get<usdrt::GfVec3f>(&parameters_.diagonal_inertias);
        }
    }
    
    // set force on bodies (in body frame of inertial frame of reference)
    void setForceAPI(const usdrt::UsdPrim &prim, bool isGlobal = false){
        if(!prim.HasAPI(usdrt::PhysxSchemaPhysxForceAPI::_GetStaticTfType())){
            usdrt::PhysxSchemaPhysxForceAPI::Apply(prim);    
        }
        usdrt::PhysxSchemaPhysxForceAPI forceSchema(prim);

        if(!prim.HasAttribute(usdrt::TfToken("physxForce:force"))){
            forceSchema.CreateForceAttr().Set(usdrt::GfVec3f(0, 0, 0));
        }
        if(!prim.HasAttribute(usdrt::TfToken("physxForce:torque"))){
            forceSchema.CreateTorqueAttr().Set(usdrt::GfVec3f(0, 0, 0));
        }
        if(!prim.HasAttribute(usdrt::TfToken("physxForce:forceEnabled"))){
            forceSchema.CreateForceEnabledAttr().Set(true);
        }
        if(!prim.HasAttribute(usdrt::TfToken("physxForce:worldFrameEnabled"))){
            forceSchema.CreateWorldFrameEnabledAttr().Set(isGlobal);
        }
        if(!prim.HasAttribute(usdrt::TfToken("physxForce:mode"))){
            forceSchema.CreateModeAttr().Set(usdrt::TfToken("force"));
        }
    }

    void getVehicleVelocity(usdrt::GfVec3f& velocity){
        if(!vehicle_base_link_.IsValid()){return;}
        vehicle_base_link_.GetAttribute(usdrt::TfToken("physics:velocity")).Get<usdrt::GfVec3f>(&velocity);
    }

    void getVehicleRotationMatrix(usdrt::GfMatrix3f& rot_mat){
        if(!vehicle_base_link_.IsValid()){return;}
        usdrt::GfQuatd orient(0, 0, 0, 0);
        vehicle_base_link_.GetAttribute(usdrt::TfToken("xformOp:orient")).Get<usdrt::GfQuatd>(&orient);
        rot_mat.SetRotate(orient);
    }

    void setRotorVelocity(const float velocity, const int idx = 0){
        if(rotor_joint_group_.empty() || idx < 0 || idx >= static_cast<int>(rotor_joint_group_.size())){
            return;
        }
        // Ensure DriveAPI attribute exists (A weird as bug in isaacsim 4.5.0 API is not letting me set targetVelocity to joints even though the attribute already exists in the prims of USD file
        if (!rotor_joint_group_[idx].GetAttribute(usdrt::TfToken("drive:angular:physics:targetVelocity")).IsValid()) {
            usdrt::UsdPhysicsDriveAPI(rotor_joint_group_[idx], usdrt::UsdPhysicsTokens->angular)
                .CreateTargetVelocityAttr();
        }
        rotor_joint_group_[idx].GetAttribute(usdrt::TfToken("drive:angular:physics:targetVelocity")).Set(velocity*RAD2DEG);
        // rotor_joint_group_[idx].GetAttribute(usdrt::TfToken("state:angular:physics:velocity")).Set(velocity * RAD2DEG); // don't use as of now (isaacsim 4.5.0)
    }

    void setRotorForce(const float thrust, int idx = 0){
        if(rotor_body_group_.empty() || idx < 0 || idx >= static_cast<int>(rotor_body_group_.size())){
            return;
        }
        usdrt::GfVec3f force(0, 0, thrust);
        rotor_body_group_[idx].GetAttribute(usdrt::TfToken("physxForce:force")).Set(force); // for this to work your rigid body should have "normal physics:Force API added onto the prim" (can do it via GUI) 
    }

    void setActuatorPosition(const float position, int idx = 0){
        if(actuator_joint_group_.empty() || idx < 0 || idx >= static_cast<int>(actuator_joint_group_.size())){
            return;
        }
        actuator_joint_group_[idx].GetAttribute(usdrt::TfToken("drive:angular:physics:targetPosition")).Set(position);
        // actuator_joint_group_[idx].GetAttribute(usdrt::TfToken("state:angular:physics:position")).Set(position); // don't use as of now (isaacsim 4.5.0)
    }

    void setActuatorForce(const float force, int idx = 0){
        if (actuator_body_group_.empty() || idx < 0 || idx >= static_cast<int>(actuator_body_group_.size())){
            return;
        }
        usdrt::GfVec3f applied_force(force, 0, 0);
        actuator_body_group_[idx].GetAttribute(usdrt::TfToken("physxForce:force")).Set(applied_force);
    }

    void setVehicleForceAndTorque(const usdrt::GfVec3f& force, const usdrt::GfVec3f& torque, const bool isGlobal = true){
        if (!vehicle_base_link_.IsValid()){return;}
        vehicle_base_link_.GetAttribute(usdrt::TfToken("physxForce:worldFrameEnabled")).Set(isGlobal);
        vehicle_base_link_.GetAttribute(usdrt::TfToken("physxForce:force")).Set(force);
        vehicle_base_link_.GetAttribute(usdrt::TfToken("physxForce:torque")).Set(torque);
    }


    void printPrimAttributes(const usdrt::UsdPrim& prim){
        if (!prim.IsValid()) {
            std::cout << "[ERROR] Invalid prim\n";
            return;
        }
        std::cout << "\n[INFO] Attributes for prim: " << prim.GetPrimPath().GetString() << "\n";

        auto attrs = prim.GetAttributes();
        for (const auto& attr : attrs){
            std::cout << "  - " << attr.GetName().GetString();
            // Optional: print type
            auto typeName = attr.GetTypeName();
            std::cout << " | type: " << typeName.GetAsToken().GetString();
            // Optional: check if it has a value
            if (attr.HasValue()) {
                std::cout << " | hasValue";
            }
            std::cout << "\n";
        }
    }
    void printAppliedSchemas(const usdrt::UsdPrim& prim){
        if (!prim.IsValid()) return;
        std::cout << "\n[INFO] Applied schemas for prim: " << prim.GetPrimPath().GetString() << "\n";
        auto schemas = prim.GetAppliedSchemas();
        for (const auto& s : schemas){
            std::cout << "  - " << s.GetString() << "\n";
        }
    }

    // ---------------------------------------
};
}

#endif
