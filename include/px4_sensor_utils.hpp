#ifndef PX4_SENSOR_UTILS_HPP
#define PX4_SENSOR_UTILS_HPP

#include "px4_vehicle_utils.hpp"

// all sensor plugin's logic ported over from gz plugins
// any new custom plugins to be added here (if the data need to be visualized in QGC a driver for the same should be created in the px4_firmware)


namespace isaac::px4_sitl::bridge{


class ImuSensor{

public:
    static constexpr uint32_t MAVLINK_FIELD_ID_IMU = 63U;
    struct ImuParameters{
        // extrinsics
        usdrt::GfVec3d offset_translate_xyz = usdrt::GfVec3d(0, 0, 0);
        usdrt::GfVec3d offset_orient_euler_rad = usdrt::GfVec3d(0, 0, 0);
        // intrinsics
        // https://docs.holybro.com/autopilot/pixhawk-6c-mini/technical-specification
        usdrt::GfVec3f prev_linear_velocity = usdrt::GfVec3f(0, 0, 0);
        usdrt::GfVec3f accel_bias = usdrt::GfVec3f(0, 0, 0);
        usdrt::GfVec3f gyro_bias = usdrt::GfVec3f(0, 0, 0);
        float accelerometer_noise_density = 0.00186f;       // Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
        float accelerometer_random_walk = 0.006f;           // Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
        float accelerometer_bias_correlation_time = 300.0f; // Accelerometer bias correlation time constant [s]
        float accelerometer_turn_on_bias_sigma = 0.0196f;   // Accelerometer turn on bias standard deviation [m/s^2]
        float gyroscope_noise_density = 0.00018665f;        // Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
        float gyroscope_random_walk = 0.000038785f;         // Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
        float gyroscope_bias_correlation_time = 1000.0f;    // Gyroscope bias correlation time constant [s]
        float gyroscope_turn_on_bias_sigma = 0.0087f;       // Gyroscope turn on bias standard deviation [rad/s]
    };
    struct ImuReadings{
        usdrt::GfVec3f linear_acceleration_noisy = usdrt::GfVec3f(0, 0, 0); // m/s^2
        usdrt::GfVec3f angular_velocity_noisy = usdrt::GfVec3f(0, 0, 0);    // rad/s
    };
    ImuParameters &getParameters() { return parameters_; }
    const ImuReadings &getImuReadings() { return readings_; }

    // main sensor data extraction/sampling logic (taken from drone/UAV rigidBody/Articulation poses) sampled @200Hz (FCU IMU simulation)
    const void sample(const Px4Multirotor::VehicleMotion &motion, const double delta_time = 0.005f){
        std::random_device rd;
        xoshiro256ss rng(rd());
        // Z ~ N(μ,σ) = N(0, 0.05) should expose the param?
        std::normal_distribution<float> normal_dist(0.0f, 1.0f);
        const float dt = static_cast<float>(delta_time);

        // Position and orientation calculations
        const auto pos_offset = quatRotate(motion.orient, parameters_.offset_translate_xyz);
        const auto offset_quat = omni::math::linalg::eulerAnglesToQuaternion(parameters_.offset_orient_euler_rad, omni::math::linalg::EulerRotationOrder::XYZ);
        const auto rot = motion.orient * offset_quat;

        // Velocity and acceleration calculations
        const auto angular_vel = motion.angular_velocity * DEG2RAD;
        const auto linear_vel = motion.velocity + GfCross(angular_vel, static_cast<usdrt::GfVec3f>(pos_offset));
        auto linear_acc = SMOOTHING_FACTOR * (linear_vel - parameters_.prev_linear_velocity) / dt;
        linear_acc[2] += GRAVITY;

        parameters_.prev_linear_velocity = linear_vel;
        // Transform to body frame
        const auto lin_acc_b = quatRotateInverse(static_cast<usdrt::GfQuatf>(rot), usdrt::GfVec3f(linear_acc));
        const auto ang_vel_b = quatRotateInverse(static_cast<usdrt::GfQuatf>(rot), usdrt::GfVec3f(angular_vel));

        const auto sqrt_dt = std::sqrt(dt);
        // Accelerometer noise parameters_
        const float tau_a = parameters_.accelerometer_bias_correlation_time;
        const float sigma_a_d = parameters_.accelerometer_noise_density / sqrt_dt;
        const float sigma_b_a = parameters_.accelerometer_random_walk;
        const float sigma_b_a_d = std::sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 * (std::exp(-2.0 * dt / tau_a) - 1.0));
        const float phi_a_d = std::exp(-1.0 / tau_a * dt);
        // Gyroscope noise parameters_
        const float tau_g = parameters_.gyroscope_bias_correlation_time;
        const float sigma_g_d = parameters_.gyroscope_noise_density / sqrt_dt;
        const float sigma_b_g = parameters_.gyroscope_random_walk;
        const float sigma_b_g_d = std::sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (std::exp(-2.0 * dt / tau_g) - 1.0));
        const float phi_g_d = std::exp(-1.0 / tau_g * dt);

        // Add noise & update bias, assuming each axis is uncorrelated
        for(int i = 0; i < 3; ++i){
            // Accelerometer
            parameters_.accel_bias[i] = phi_a_d * parameters_.accel_bias[i] + sigma_b_a_d * normal_dist(rng);
            readings_.linear_acceleration_noisy[i] = lin_acc_b[i] + parameters_.accel_bias[i] + sigma_a_d * normal_dist(rng);
            // Gyroscope
            parameters_.gyro_bias[i] = phi_g_d * parameters_.gyro_bias[i] + sigma_b_g_d * normal_dist(rng);
            readings_.angular_velocity_noisy[i] = ang_vel_b[i] + parameters_.gyro_bias[i] + sigma_g_d * normal_dist(rng);
        }
    }

    const void updateBias(){
        std::random_device rd;
        xoshiro256ss rng(rd());
        std::normal_distribution<float> normal_dist(0.0f, 1.0f);
        for(int i = 0; i < 3; ++i){
            parameters_.accel_bias[i] = parameters_.accelerometer_turn_on_bias_sigma * normal_dist(rng);
            parameters_.gyro_bias[i] = parameters_.gyroscope_turn_on_bias_sigma * normal_dist(rng);
        }
    }

    const void reset(){
        parameters_.accel_bias *= 0;
        parameters_.gyro_bias *= 0;
        parameters_.prev_linear_velocity *= 0;
        readings_ = ImuReadings();
        updateBias();
    }

private:
    ImuParameters parameters_;
    ImuReadings readings_;
    static constexpr float SMOOTHING_FACTOR = 0.90f;
    static constexpr float GRAVITY = 9.80665f;
};

/**
 * MagnetoSensor class
 *
 * @see https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/gazebo_magnetometer_plugin.cpp
 * and https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/geo_mag_declination.cpp
 */
class MagnetoSensor{
public:
	static constexpr uint32_t MAVLINK_FIELD_ID_MAG = 448U;
	struct MagParameters{
		// intrinsics
		usdrt::GfVec3f mag_bias = usdrt::GfVec3f(0, 0, 0);
		float magnetometer_noise_density = 0.0004f;        // Noise density in gauss / sqrt(Hz)
		float magnetometer_random_walk = 0.0000064f;       // Random walk in gauss * sqrt(Hz)
		float magnetometer_bias_correlation_time = 600.0f; // Bias correlation time in seconds
	};
	struct MagReadings{
		usdrt::GfVec3f mag_field_noisy = usdrt::GfVec3f(0,0,0);
	};

	MagnetoSensor(){}

	MagParameters& getParameters(){ return parameters_; }
	const MagReadings& getMagReadings(){ return readings_; }

	// Mag sensor readings/data sampling/data extraction logic here sampled @100Hz
	const void sample(
		const Px4Multirotor::VehicleMotion& motion, 
		const float latitude = 1.440455f,    // degree
        const float longitude = 103.616115f, // degree
		const double delta_time=0.01f
	){
        std::random_device rd;
        xoshiro256ss rng(rd());
        std::normal_distribution<float> normal_dist(0.0f, 1.0f);
        const float dt = static_cast<float>(delta_time);

		const auto pos = motion.translate;
        const auto latlon = reprojectEarth(
			pos,
			latitude * M_PI / 180.0f,   // convert to radian
            longitude * M_PI / 180.0f	// convert to radian
		);
        const float lat = latlon[0] * 180.0f / M_PI;                   // back to degree
        const float lon = latlon[1] * 180.0f / M_PI;                   // back to degree
        const auto declination_rad = get_mag_declination(lat, lon);
        const auto inclination_rad = get_mag_inclination(lat, lon);
        
		const auto strength_gauss = get_mag_strength(lat, lon) * 0.01f; // convert to gauss
        const auto H = strength_gauss * std::cos(inclination_rad);
        const auto Z = std::tan(inclination_rad) * H;
        const auto X = H * std::cos(declination_rad);
        const auto Y = H * std::sin(declination_rad);
        const usdrt::GfVec3f mag_field{X, Y, Z};

        // Magnetometer noise parameters
        const float tau_m = parameters_.magnetometer_bias_correlation_time;
        const float sigma_d = parameters_.magnetometer_noise_density / std::sqrt(dt);
        const float sigma_m = parameters_.magnetometer_random_walk;
        const float sigma_m_d = std::sqrt(-sigma_m * sigma_m * tau_m / 2.0 *(std::exp(-2.0 * dt / tau_m) - 1.0));
        const float phi_m_d = std::exp(-1.0 / tau_m * dt);
		for(int i = 0; i < 3; ++i){
            parameters_.mag_bias[i] = phi_m_d * parameters_.mag_bias[i] + sigma_m_d * normal_dist(rng);
            readings_.mag_field_noisy[i] = mag_field[i] + parameters_.mag_bias[i] + sigma_d * normal_dist(rng);
        }
	}

	const void reset(){
		parameters_.mag_bias *= 0;
		readings_ = MagReadings();
	}

private:
	MagParameters parameters_;
	MagReadings readings_;
    static constexpr float SAMPLING_RES = 10.0f;
    static constexpr float SAMPLING_MIN_LAT = -60.0f;
    static constexpr float SAMPLING_MAX_LAT = 60.0f;
    static constexpr float SAMPLING_MIN_LON = -180.0f;
    static constexpr float SAMPLING_MAX_LON = 180.0f;
  	// declination data in degrees
    static constexpr const int8_t declination_table[13][37] ={
		{47, 46, 45, 43, 42, 41, 39, 37, 33, 29, 23, 16, 10, 4, -1, -6, -10, -15, -20, -27, -34, -42, -49, -56, -62, -67, -72, -74, -75, -73, -61, -22, 26, 42, 47, 48, 47},
		{31, 31, 31, 30, 30, 30, 30, 29, 27, 24, 18, 11, 3, -4, -9, -13, -15, -18, -21, -27, -33, -40, -47, -52, -56, -57, -56, -52, -44, -30, -14, 2, 14, 22, 27, 30, 31},
		{22, 23, 23, 23, 22, 22, 22, 23, 22, 19, 13, 5, -4, -12, -17, -20, -22, -22, -23, -25, -30, -36, -41, -45, -46, -44, -39, -31, -21, -11, -3, 4, 10, 15, 19, 21, 22},
		{17, 17, 17, 18, 17, 17, 17, 17, 16, 13, 8, -1, -10, -18, -22, -25, -26, -25, -22, -20, -21, -25, -29, -32, -31, -28, -23, -16, -9, -3, 0, 4, 7, 11, 14, 16, 17},
		{13, 13, 14, 14, 14, 13, 13, 12, 11, 9, 3, -5, -14, -20, -24, -25, -24, -21, -17, -12, -9, -11, -14, -17, -18, -16, -12, -8, -3, -0, 1, 3, 6, 8, 11, 12, 13},
		{11, 11, 11, 11, 11, 10, 10, 10, 9, 6, -0, -8, -15, -21, -23, -22, -19, -15, -10, -5, -2, -2, -4, -7, -9, -8, -7, -4, -1, 1, 1, 2, 4, 7, 9, 10, 11},
		{10, 9, 9, 9, 9, 9, 9, 8, 7, 3, -3, -10, -16, -20, -20, -18, -14, -9, -5, -2, 1, 2, 0, -2, -4, -4, -3, -2, -0, 0, 0, 1, 3, 5, 7, 9, 10},
		{9, 9, 9, 9, 9, 9, 9, 8, 6, 1, -4, -11, -16, -18, -17, -14, -10, -5, -2, -0, 2, 3, 2, 0, -1, -2, -2, -1, -0, -1, -1, -1, 1, 3, 6, 8, 9},
		{8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -6, -12, -15, -16, -15, -11, -7, -4, -1, 1, 3, 4, 3, 2, 1, 0, -0, -0, -1, -2, -3, -4, -2, 0, 3, 6, 8},
		{7, 9, 10, 11, 12, 12, 12, 9, 5, -1, -7, -13, -15, -15, -13, -10, -6, -3, 0, 2, 3, 4, 4, 4, 3, 2, 1, 0, -1, -3, -5, -6, -6, -3, 0, 4, 7},
		{5, 8, 11, 13, 14, 15, 14, 11, 5, -2, -9, -15, -17, -16, -13, -10, -6, -3, 0, 3, 4, 5, 6, 6, 6, 5, 4, 2, -1, -5, -8, -9, -9, -6, -3, 1, 5},
		{3, 8, 11, 15, 17, 17, 16, 12, 5, -4, -12, -18, -19, -18, -16, -12, -8, -4, -0, 3, 5, 7, 9, 10, 10, 9, 7, 4, -1, -6, -10, -12, -12, -9, -5, -1, 3},
		{3, 8, 12, 16, 19, 20, 18, 13, 4, -8, -18, -24, -25, -23, -20, -16, -11, -6, -1, 3, 7, 11, 14, 16, 17, 17, 14, 8, -0, -8, -13, -15, -14, -11, -7, -2, 3},
    };
    // inclination data in degrees
    static constexpr const int8_t inclination_table[13][37] ={
		{-78, -76, -74, -72, -70, -68, -65, -63, -60, -57, -55, -54, -54, -55, -56, -57, -58, -59, -59, -59, -59, -60, -61, -63, -66, -69, -73, -76, -79, -83, -86, -87, -86, -84, -82, -80, -78},
		{-72, -70, -68, -66, -64, -62, -60, -57, -54, -51, -49, -48, -49, -51, -55, -58, -60, -61, -61, -61, -60, -60, -61, -63, -66, -69, -72, -76, -78, -80, -81, -80, -79, -77, -76, -74, -72},
		{-64, -62, -60, -59, -57, -55, -53, -50, -47, -44, -41, -41, -43, -47, -53, -58, -62, -65, -66, -65, -63, -62, -61, -63, -65, -68, -71, -73, -74, -74, -73, -72, -71, -70, -68, -66, -64},
		{-55, -53, -51, -49, -46, -44, -42, -40, -37, -33, -30, -30, -34, -41, -48, -55, -60, -65, -67, -68, -66, -63, -61, -61, -62, -64, -65, -66, -66, -65, -64, -63, -62, -61, -59, -57, -55},
		{-42, -40, -37, -35, -33, -30, -28, -25, -22, -18, -15, -16, -22, -31, -40, -48, -55, -59, -62, -63, -61, -58, -55, -53, -53, -54, -55, -55, -54, -53, -51, -51, -50, -49, -47, -45, -42},
		{-25, -22, -20, -17, -15, -12, -10, -7, -3, 1, 3, 2, -5, -16, -27, -37, -44, -48, -50, -50, -48, -44, -41, -38, -38, -38, -39, -39, -38, -37, -36, -35, -35, -34, -31, -28, -25},
		{-5, -2, 1, 3, 5, 8, 10, 13, 16, 20, 21, 19, 12, 2, -10, -20, -27, -30, -30, -29, -27, -23, -19, -17, -17, -17, -18, -18, -17, -16, -16, -16, -16, -15, -12, -9, -5},
		{15, 18, 21, 22, 24, 26, 29, 31, 34, 36, 37, 34, 28, 20, 10, 2, -3, -5, -5, -4, -2, 2, 5, 7, 8, 7, 7, 6, 7, 7, 7, 6, 5, 6, 8, 11, 15},
		{31, 34, 36, 38, 39, 41, 43, 46, 48, 49, 49, 46, 42, 36, 29, 24, 20, 19, 20, 21, 23, 25, 28, 30, 30, 30, 29, 29, 29, 29, 28, 27, 25, 25, 26, 28, 31},
		{43, 45, 47, 49, 51, 53, 55, 57, 58, 59, 59, 56, 53, 49, 45, 42, 40, 40, 40, 41, 43, 44, 46, 47, 47, 47, 47, 47, 47, 47, 46, 44, 42, 41, 40, 42, 43},
		{53, 54, 56, 57, 59, 61, 64, 66, 67, 68, 67, 65, 62, 60, 57, 55, 55, 54, 55, 56, 57, 58, 59, 59, 60, 60, 60, 60, 60, 60, 59, 57, 55, 53, 52, 52, 53},
		{62, 63, 64, 65, 67, 69, 71, 73, 75, 75, 74, 73, 70, 68, 67, 66, 65, 65, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 70, 69, 67, 65, 63, 62, 62, 62},
		{71, 71, 72, 73, 75, 77, 78, 80, 81, 81, 80, 79, 77, 76, 74, 73, 73, 73, 73, 73, 73, 74, 74, 75, 76, 77, 78, 78, 78, 78, 77, 75, 73, 72, 71, 71, 71},
    };
    // strength data in centi-Tesla
    static constexpr const int8_t strength_table[13][37] ={
		{62, 60, 58, 56, 54, 52, 49, 46, 43, 41, 38, 36, 34, 32, 31, 31, 30, 30, 30, 31, 33, 35, 38, 42, 46, 51, 55, 59, 62, 64, 66, 67, 67, 66, 65, 64, 62},
		{59, 56, 54, 52, 50, 47, 44, 41, 38, 35, 32, 29, 28, 27, 26, 26, 26, 25, 25, 26, 28, 30, 34, 39, 44, 49, 54, 58, 61, 64, 65, 66, 65, 64, 63, 61, 59},
		{54, 52, 49, 47, 45, 42, 40, 37, 34, 30, 27, 25, 24, 24, 24, 24, 24, 24, 24, 24, 25, 28, 32, 37, 42, 48, 52, 56, 59, 61, 62, 62, 62, 60, 59, 56, 54},
		{49, 47, 44, 42, 40, 37, 35, 33, 30, 28, 25, 23, 22, 23, 23, 24, 25, 25, 26, 26, 26, 28, 31, 36, 41, 46, 51, 54, 56, 57, 57, 57, 56, 55, 53, 51, 49},
		{43, 41, 39, 37, 35, 33, 32, 30, 28, 26, 25, 23, 23, 23, 24, 25, 26, 28, 29, 29, 29, 30, 32, 36, 40, 44, 48, 51, 52, 52, 51, 51, 50, 49, 47, 45, 43},
		{38, 36, 35, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 24, 25, 26, 28, 30, 31, 32, 32, 32, 33, 35, 38, 42, 44, 46, 47, 46, 45, 45, 44, 43, 41, 40, 38},
		{34, 33, 32, 32, 31, 31, 31, 30, 30, 30, 29, 28, 27, 27, 27, 28, 29, 31, 32, 33, 33, 33, 34, 35, 37, 39, 41, 42, 43, 42, 41, 40, 39, 38, 36, 35, 34},
		{33, 33, 32, 32, 33, 33, 34, 34, 35, 35, 34, 33, 32, 31, 30, 30, 31, 32, 33, 34, 35, 35, 36, 37, 38, 40, 41, 42, 42, 41, 40, 39, 37, 36, 34, 33, 33},
		{34, 34, 34, 35, 36, 37, 39, 40, 41, 41, 40, 39, 37, 35, 35, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 45, 45, 43, 41, 39, 37, 35, 34, 34},
		{37, 37, 38, 39, 41, 42, 44, 46, 47, 47, 46, 45, 43, 41, 40, 39, 39, 40, 41, 41, 42, 43, 45, 46, 47, 48, 49, 50, 50, 50, 48, 46, 43, 41, 39, 38, 37},
		{42, 42, 43, 44, 46, 48, 50, 52, 53, 53, 52, 51, 49, 47, 45, 45, 44, 44, 45, 46, 46, 47, 48, 50, 51, 53, 54, 55, 56, 55, 54, 52, 49, 46, 44, 43, 42},
		{48, 48, 49, 50, 52, 53, 55, 56, 57, 57, 56, 55, 53, 51, 50, 49, 48, 48, 48, 49, 49, 50, 51, 53, 55, 56, 58, 59, 60, 60, 58, 56, 54, 52, 50, 49, 48},
		{54, 54, 54, 55, 56, 57, 58, 58, 59, 58, 58, 57, 56, 54, 53, 52, 51, 51, 51, 51, 52, 53, 54, 55, 57, 58, 60, 61, 62, 61, 61, 59, 58, 56, 55, 54, 54},
    };

	static unsigned get_lookup_table_index(float* val, float min, float max){
        /* for the rare case of hitting the bounds exactly
         * the rounding logic wouldn't fit, so enforce it.
         */
        /* limit to table bounds - required for maxima even when table spans full globe range */
        /* limit to (table bounds - 1) because bilinear interpolation requires checking (index + 1) */
        *val = constrain(*val, min, max - SAMPLING_RES);
        return static_cast<unsigned>((-(min) + *val) / SAMPLING_RES);
	}
	static float get_table_data(float lat, float lon, const int8_t table[13][37]){
        /*
         * If the values exceed valid ranges, return zero as default
         * as we have no way of knowing what the closest real value
         * would be.
         */
        if (lat < -90.0f || lat > 90.0f ||
            lon < -180.0f || lon > 180.0f){
            return 0.0f;
        }
        /* round down to nearest sampling resolution */
        float min_lat = int(lat / SAMPLING_RES) * SAMPLING_RES;
        float min_lon = int(lon / SAMPLING_RES) * SAMPLING_RES;

        /* find index of nearest low sampling point */
        unsigned min_lat_index = get_lookup_table_index(&min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
        unsigned min_lon_index = get_lookup_table_index(&min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

        const float data_sw = table[min_lat_index][min_lon_index];
        const float data_se = table[min_lat_index][min_lon_index + 1];
        const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
        const float data_nw = table[min_lat_index + 1][min_lon_index];

        /* perform bilinear interpolation on the four grid corners */
        const float lat_scale = constrain((lat - min_lat) / SAMPLING_RES, 0.0f, 1.0f);
        const float lon_scale = constrain((lon - min_lon) / SAMPLING_RES, 0.0f, 1.0f);

        const float data_min = lon_scale * (data_se - data_sw) + data_sw;
        const float data_max = lon_scale * (data_ne - data_nw) + data_nw;

        return lat_scale * (data_max - data_min) + data_min;
	}
    float get_mag_declination(float lat, float lon){
        return get_table_data(lat, lon, declination_table);
    }
    float get_mag_inclination(float lat, float lon){
        return get_table_data(lat, lon, inclination_table);
    }
    float get_mag_strength(float lat, float lon){
        return get_table_data(lat, lon, strength_table);
    }
};


/**
 * BaroSensor class
 *
 * @see https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/gazebo_barometer_plugin.cpp
 */
class BaroSensor{
public:
	static constexpr uint32_t MAVLINK_FIELD_ID_BARO = 6656U;
	struct BaroParameters{
        // intrinsics
        bool first_read;
        float starting_altitude = 0.0f;
        float baro_drift = 0.0f;
        float drift_pa_per_second = 0.0f;
	};
	struct BaroReadings{
        float absolute_pressure_noisy = 0.0f;
        float pressure_altitude_noisy = 0.0f;
        float temperature_noisy = 0.0f;
	};
	BaroParameters& getParameters(){return parameters_;}
	BaroReadings& getBaroReadings(){return readings_;}

	// barometer readings/data extraction/sampling logic; sampling @50Hz
	const void sample(
		const Px4Multirotor::VehicleMotion& motion,
		const float home_altitude = 24.0f, // m
		const double delta_time = 0.02f   // 50Hz
	){
        std::random_device rd;
        xoshiro256ss rng(rd());
        std::normal_distribution<float> normal_dist(0.0f, 1.0f);
        const float dt = static_cast<float>(delta_time);
        auto pos = motion.translate;
        if(!parameters_.first_read){
            parameters_.starting_altitude = pos[2];
            parameters_.first_read = true;
        }

		// using the princle of hydrostatic pressure we calculate the atitude of the system on which the sensor is mounted (just read it online mate its easy fluids concept)
        const auto alt_amsl = home_altitude + static_cast<float>(pos[2]) - parameters_.starting_altitude; // Z-axis altitude ENU
        const auto temperature_local = TEMPERATURE_MSL - L * alt_amsl;
        const auto absolute_pressure = PRESSURE_MSL / std::pow(TEMPERATURE_MSL / temperature_local, kBaro);
        const auto air_density = AIR_DENSITY_MSL / std::pow(TEMPERATURE_MSL / temperature_local, kBaro1);
        const auto baro_drift = parameters_.baro_drift + parameters_.drift_pa_per_second * dt; // update baro_drift
        parameters_.baro_drift = baro_drift;
        // add noise and drift
        readings_.absolute_pressure_noisy = (absolute_pressure + normal_dist(rng) + baro_drift) * 0.01f; // convert to hPa
        readings_.pressure_altitude_noisy = alt_amsl - ((normal_dist(rng) + baro_drift) / (G * air_density));
        readings_.temperature_noisy = temperature_local + normal_dist(rng) - ABSOLUTE_ZERO_C; // convert to Celsius
	}

	const void reset(){
        parameters_.first_read = false;
        parameters_.starting_altitude = 0.0f;
        parameters_.baro_drift = 0.0f;
        readings_ = BaroReadings();
	}

private:
    BaroParameters parameters_;
    BaroReadings readings_;
    static constexpr float TEMPERATURE_MSL = 288.15f;
    static constexpr float PRESSURE_MSL = 101325.0f;
    static constexpr float AIR_DENSITY_MSL = 1.225f;
    static constexpr float ABSOLUTE_ZERO_C = 273.15f;
    static constexpr float G = 9.80665f;                 // Gravitational Constant
    static constexpr float R = 8.3144598f;               // Universal Gas Constant
    static constexpr float M = 0.0289644f;               // Molar mass of Earth's air
    static constexpr float L = 0.0065f;                  // Temperature Lapse Rate
    static constexpr float kBaro = G * M / (R * L);      // Barometric constant
    static constexpr float kBaro1 = G * M / (R * L) - 1; // Barometric constant - 1
};

/**
 * GpsSensor class
 *
 * @see https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/gazebo_gps_plugin.cpp
 */
class GPSSensor{

public:
	struct GPSParameters{
        // intrinsics
        // https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps/standard-m10-m9n-m8n-gps/overview
        // https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf
        usdrt::GfVec3f gps_bias = usdrt::GfVec3f(0, 0, 0);
        float gps_xy_noise_density = 0.0002f;
        float gps_z_noise_density = 0.0004f;
        float gps_vxy_noise_density = 0.001f;
        float gps_vz_noise_density = 0.002f;
        float gps_xy_random_walk = 0.005f;
        float gps_z_random_walk = 0.005f;
        float gps_correlation_time = 60.0f;
	};
	struct GPSReadings{
        usdrt::GfVec3f gps_latlonalt = usdrt::GfVec3f(0, 0, 0);
        usdrt::GfVec3f gps_vel_enu = usdrt::GfVec3f(0, 0, 0);
	};
	GPSParameters& getParameters(){return parameters_;}
	GPSReadings& getGPSReadings(){return readings_;}

	// GPS sensor data sampled @10Hz
	const void sample(
		const Px4Multirotor::VehicleMotion& motion,
		const usdrt::GfVec3f& home,
		const double delta_time=0.1f
	){
        std::random_device rd;
        xoshiro256ss rng(rd());
        std::normal_distribution<float> normal_dist(0.0f, 1.0f);
        const float dt = static_cast<float>(delta_time);
        const float sqrt_dt_inv = 1.0f / std::sqrt(dt);
        const auto pos = motion.translate;
        const auto vel = motion.velocity;
	
        const float noise_xy = normal_dist(rng) * parameters_.gps_xy_noise_density * sqrt_dt_inv;
        const float noise_z = normal_dist(rng) * parameters_.gps_z_noise_density * sqrt_dt_inv;
        const float noise_vxy = normal_dist(rng) * parameters_.gps_vxy_noise_density * sqrt_dt_inv;
        const float noise_vz = normal_dist(rng) * parameters_.gps_vz_noise_density * sqrt_dt_inv;
        const float random_walk_xy = normal_dist(rng) * parameters_.gps_xy_random_walk * sqrt_dt_inv;
        const float random_walk_z = normal_dist(rng) * parameters_.gps_z_random_walk * sqrt_dt_inv;

        parameters_.gps_bias[0] += random_walk_xy * dt - parameters_.gps_bias[0] / parameters_.gps_correlation_time;
        parameters_.gps_bias[1] += random_walk_xy * dt - parameters_.gps_bias[1] / parameters_.gps_correlation_time;
        parameters_.gps_bias[2] += random_walk_z * dt - parameters_.gps_bias[2] / parameters_.gps_correlation_time;

        const float pos_x_noisy = static_cast<float>(pos[0]) + noise_xy + parameters_.gps_bias[0];
        const float pos_y_noisy = static_cast<float>(pos[1]) + noise_xy + parameters_.gps_bias[1];
        const float pos_z_noisy = static_cast<float>(pos[2]) + noise_z + parameters_.gps_bias[2];
        usdrt::GfVec3f pos_noisy(pos_x_noisy, pos_y_noisy, pos_z_noisy);
        const auto gps_latlon = reprojectEarth(pos_noisy, home[0] * DEG2RAD, home[1] * DEG2RAD);

		// readings updation w/ noise model for the gps
        readings_.gps_latlonalt[0] = gps_latlon[0] * RAD2DEG;
        readings_.gps_latlonalt[1] = gps_latlon[1] * RAD2DEG;
        readings_.gps_latlonalt[2] = home[2] + pos_z_noisy;
        readings_.gps_vel_enu[0] = vel[0] + noise_vxy;
        readings_.gps_vel_enu[1] = vel[1] + noise_vxy;
        readings_.gps_vel_enu[2] = vel[2] + noise_vz;
	}

	const void reset(){
		parameters_.gps_bias *= 0;
		readings_ = GPSReadings();
	}

private:
    GPSParameters parameters_;
    GPSReadings readings_;
};


}
#endif