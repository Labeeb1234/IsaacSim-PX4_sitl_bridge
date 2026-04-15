#include <StreamHILSensorNodeDatabase.h>
#include <MavlinkServerManager.hpp>
#include "px4_sensor_utils.hpp"

#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>
#include <carb/events/EventsUtils.h>

using omni::graph::core::Type;
using omni::graph::core::BaseDataType;
using omni::graph::core::AttributeRole;


namespace isaac::px4_sitl::bridge{

class StreamHILSensorNode{
public:
    static void initialize(const GraphContextObj &context, const NodeObj &nodeObj){
        constexpr u_char kDefaultSystemId = 1;
        const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
        auto &state = StreamHILSensorNodeDatabase::sPerInstanceState<StreamHILSensorNode>(nodeObj, graphInstanceIndex);

        state.m_vehicle_ = std::make_unique<Px4Multirotor>(); 
        state.m_vehicle_->reset();
        state.m_imu_ = std::make_unique<ImuSensor>();
        ImuSensor::ImuParameters &imu_params = state.m_imu_->getParameters();
        state.m_mag_ = std::make_unique<MagnetoSensor>();
        MagnetoSensor::MagParameters &mag_params = state.m_mag_->getParameters();
        state.m_baro_ = std::make_unique<BaroSensor>();
        BaroSensor::BaroParameters &baro_params = state.m_baro_->getParameters();

        // onValueChanged callback
        auto cb = [](const omni::graph::core::AttributeObj &attr, const void *value){
            const NodeObj nodeObj = attr.iAttribute->getNode(attr);
            const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
            auto &state = StreamHILSensorNodeDatabase::sPerInstanceState<StreamHILSensorNode>(nodeObj, graphInstanceIndex);
            ImuSensor::ImuParameters &imu_params = state.m_imu_->getParameters();
            MagnetoSensor::MagParameters &mag_params = state.m_mag_->getParameters();
            BaroSensor::BaroParameters &baro_params = state.m_baro_->getParameters();
            
            // accelerometer arttributes extraction
            if(attr.iAttribute->getNameToken(attr) == state::offsetTranslationIMU.token()){
                imu_params.offset_translate_xyz = *static_cast<const usdrt::GfVec3d *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::offsetEulerRadIMU.token()){
                imu_params.offset_orient_euler_rad = *static_cast<const usdrt::GfVec3d *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::accelerometerNoiseDensity.token()){
                imu_params.accelerometer_noise_density = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::accelerometerRandomWalk.token()){
                imu_params.accelerometer_random_walk = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::accelerometerBiasCorrelationTime.token()){
                imu_params.accelerometer_bias_correlation_time = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::accelerometerTurnOnBiasSigma.token()){
                imu_params.accelerometer_turn_on_bias_sigma = *static_cast<const float *>(value);
                state.m_imu_->updateBias();
            }
            // gyroscope attribute extraction
            if(attr.iAttribute->getNameToken(attr) == state::gyroscopeNoiseDensity.token()){
                imu_params.gyroscope_noise_density = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::gyroscopeRandomWalk.token()){
                imu_params.gyroscope_random_walk = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::gyroscopeBiasCorrelationTime.token()){
                imu_params.gyroscope_bias_correlation_time = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::gyroscopeTurnOnBiasSigma.token()){
                imu_params.gyroscope_turn_on_bias_sigma = *static_cast<const float *>(value);
                state.m_imu_->updateBias();
            }
            // magnetometer attributes extraction
            if(attr.iAttribute->getNameToken(attr) == state::magnetometerNoiseDensity.token()){
                mag_params.magnetometer_noise_density = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::magnetometerRandomWalk.token()){
                mag_params.magnetometer_random_walk = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::magnetometerBiasCorrelationTime.token()){
                mag_params.magnetometer_bias_correlation_time = *static_cast<const float *>(value);
            }
            // Barometer attributes extraction
            if(attr.iAttribute->getNameToken(attr) == state::barometerDriftPaPerSecond.token()){
                baro_params.drift_pa_per_second = *static_cast<const float *>(value);
            }
        };

        AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, state::systemId.token());
        attr.iAttribute->setDefaultValue(attr, omni::fabric::BaseDataType::eUChar, &kDefaultSystemId, 0);
        // Helper function to initialize attributes
        auto initializeAttribute = [&](const auto &token, omni::fabric::BaseDataType type, const void *defaultValue){
            AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, token);
            attr.iAttribute->setDefaultValue(attr, type, defaultValue, 0);
            attr.iAttribute->registerValueChangedCallback(attr, cb, true);
        };
        // Initialize attributes
        initializeAttribute(state::offsetTranslationIMU.token(), omni::fabric::BaseDataType::eDouble, &imu_params.offset_translate_xyz);
        initializeAttribute(state::offsetEulerRadIMU.token(), omni::fabric::BaseDataType::eDouble, &imu_params.offset_orient_euler_rad);
        initializeAttribute(state::accelerometerNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &imu_params.accelerometer_noise_density);
        initializeAttribute(state::accelerometerRandomWalk.token(), omni::fabric::BaseDataType::eFloat, &imu_params.accelerometer_random_walk);
        initializeAttribute(state::accelerometerBiasCorrelationTime.token(), omni::fabric::BaseDataType::eFloat, &imu_params.accelerometer_bias_correlation_time);
        initializeAttribute(state::accelerometerTurnOnBiasSigma.token(), omni::fabric::BaseDataType::eFloat, &imu_params.accelerometer_turn_on_bias_sigma);
        initializeAttribute(state::gyroscopeNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &imu_params.gyroscope_noise_density);
        initializeAttribute(state::gyroscopeRandomWalk.token(), omni::fabric::BaseDataType::eFloat, &imu_params.gyroscope_random_walk);
        initializeAttribute(state::gyroscopeBiasCorrelationTime.token(), omni::fabric::BaseDataType::eFloat, &imu_params.gyroscope_bias_correlation_time);
        initializeAttribute(state::gyroscopeTurnOnBiasSigma.token(), omni::fabric::BaseDataType::eFloat, &imu_params.gyroscope_turn_on_bias_sigma);
        initializeAttribute(state::magnetometerNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &mag_params.magnetometer_noise_density);
        initializeAttribute(state::magnetometerRandomWalk.token(), omni::fabric::BaseDataType::eFloat, &mag_params.magnetometer_random_walk);
        initializeAttribute(state::magnetometerBiasCorrelationTime.token(), omni::fabric::BaseDataType::eFloat, &mag_params.magnetometer_bias_correlation_time);
        initializeAttribute(state::barometerDriftPaPerSecond.token(), omni::fabric::BaseDataType::eFloat, &baro_params.drift_pa_per_second);

        if(auto timeline = omni::timeline::getTimeline()){
            state.m_timelineEventsSubscription = carb::events::createSubscriptionToPop(
                timeline->getTimelineEventStream(),
                [&state](carb::events::IEvent *timelineEvent){
                    if(static_cast<omni::timeline::TimelineEventType>(timelineEvent->type) == omni::timeline::TimelineEventType::eStop){
                        // Reset sensor's accumulated bias and readings on timeline stopped
                        state.imu_last_sampled_us_ = 0U;
                        state.mag_last_sampled_us_ = 0U;
                        state.baro_last_sampled_us_ = 0U;
                        state.m_imu_->reset();
                        state.m_mag_->reset();
                        state.m_baro_->reset();
                        state.m_vehicle_->reset();
                        CARB_LOG_INFO("Reset sensor's accumulated bias and readings");
                    }
                });
        }
    }

    static bool compute(StreamHILSensorNodeDatabase& db){  
        const auto &time = db.inputs.time();
        const auto &home = db.inputs.homeCoordinate();
        const auto &rate_imu = db.inputs.rateIMU();
        const auto &rate_mag = db.inputs.rateMag();
        const auto &rate_baro = db.inputs.rateBaro();
        const auto &px4_instance = db.inputs.px4Instance();
        const auto &vehicle_in = db.inputs.vehiclePrim();

        // sanatize inputs LOL
        if(vehicle_in.empty()){
            CARB_LOG_WARN("No vehicle input at %s", db.abi_node().iNode->getPrimPath(db.abi_node()));
            return false;
        }
        if(time <= 0){return false;}
        auto &state = db.internalState<StreamHILSensorNode>();
        auto &server = MavlinkServerManager::getInstance();
        if(!server.isConnected(px4_instance)){
            return false;
        }
        omni::fabric::PathC vehiclePath = vehicle_in[0];
        if(!state.m_vehicle_->isVehiclePathEqual(vehiclePath)){
            if(!state.m_vehicle_->loadVehicle(vehiclePath)){
                return false;
            }
        }

        // data processing part
        Px4Multirotor::VehicleMotion motion;
        state.m_vehicle_->getVehicleMotion(motion);
        uint32_t updated_fields = 0U;
        uint64_t current_time_us = static_cast<uint64_t>(time * 1e6);     

        if(rate_imu > 0){
            uint64_t imu_period_us = static_cast<uint64_t>(1e6 / rate_imu);
            uint64_t imu_elapsed = current_time_us - state.imu_last_sampled_us_;
            if(imu_elapsed >= imu_period_us){
                double dt = imu_elapsed * 1e-6;
                state.m_imu_->sample(motion, dt);
                updated_fields |= ImuSensor::MAVLINK_FIELD_ID_IMU;
                state.imu_last_sampled_us_ = current_time_us;
            } 
        }
        if(rate_mag > 0){
            uint64_t mag_period_us = static_cast<uint64_t>(1e6 / rate_mag);
            uint64_t mag_elapsed = current_time_us - state.mag_last_sampled_us_;
            if(mag_elapsed >= mag_period_us){
                double dt = mag_elapsed * 1e-6;
                state.m_mag_->sample(motion, home[0], home[1], dt);
                updated_fields |= MagnetoSensor::MAVLINK_FIELD_ID_MAG;
                state.mag_last_sampled_us_ = current_time_us;
            }
        }
        if(rate_baro > 0){
            uint64_t baro_period_us = static_cast<uint64_t>(1e6 / rate_baro);
            uint64_t baro_elapsed = current_time_us - state.baro_last_sampled_us_;
            if(baro_elapsed >= baro_period_us){
                double dt = baro_elapsed * 1e-6;
                state.m_baro_->sample(motion, home[2], dt);
                updated_fields |= BaroSensor::MAVLINK_FIELD_ID_BARO;
                state.baro_last_sampled_us_ = current_time_us;
            }
        }
    
        auto imu_data = state.m_imu_->getImuReadings();
        auto mag_data = state.m_mag_->getMagReadings();
        auto baro_data = state.m_baro_->getBaroReadings();

        // sending mavlink sensor msgs to FCU
        mavlink_message_t msg;
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        // converted to proper frame FLU -> FRD, ENU -> NED
        mavlink_msg_hil_sensor_pack(
            db.state.systemId(), MAV_COMP_ID_IMU, &msg,
            current_time_us,                              // time_usec
            imu_data.linear_acceleration_noisy[0],        // xacc
            -1.0 * imu_data.linear_acceleration_noisy[1], // yacc
            -1.0 * imu_data.linear_acceleration_noisy[2], // zacc
            imu_data.angular_velocity_noisy[0],           // xgyro
            -1.0 * imu_data.angular_velocity_noisy[1],    // ygyro
            -1.0 * imu_data.angular_velocity_noisy[2],    // zgyro
            mag_data.mag_field_noisy[1],                  // xmag
            -1.0 * mag_data.mag_field_noisy[0],           // ymag
            mag_data.mag_field_noisy[2],                  // zmag
            baro_data.absolute_pressure_noisy,            // abs_pressure (sea level standard)
            0.0f,                                         // diff_pressure (multirotor unused)
            baro_data.pressure_altitude_noisy,            // pressure_alt
            baro_data.temperature_noisy,                  // temperature
            updated_fields,                               // fields_updated (all fields)
            0                                             // id
        );
        int len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
        return server.send(px4_instance, buffer.data(), len);
    }

private:
    carb::ObjectPtr<carb::events::ISubscription> m_timelineEventsSubscription;
    std::unique_ptr<Px4Multirotor> m_vehicle_;
    uint64_t imu_last_sampled_us_ = 0U;
    uint64_t mag_last_sampled_us_ = 0U;
    uint64_t baro_last_sampled_us_ = 0U;
    std::unique_ptr<ImuSensor> m_imu_;
    std::unique_ptr<MagnetoSensor> m_mag_;
    std::unique_ptr<BaroSensor> m_baro_;
};

REGISTER_OGN_NODE()
}