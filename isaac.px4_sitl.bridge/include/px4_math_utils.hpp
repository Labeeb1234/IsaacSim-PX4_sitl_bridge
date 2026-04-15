#ifndef PX4_MATH_UTILS_HPP
#define PX4_MATH_UTILS_HPP

#include <usdrt/gf/vec.h>
#include <usdrt/gf/quat.h>
#include <usdrt/gf/matrix.h>

namespace isaac::px4_sitl::bridge{

/**
 * @note T = double / float / half / int
 */

template <typename T>
using Vector2 = omni::math::linalg::vec2<T>;
template <typename T>
using Vector3 = omni::math::linalg::vec3<T>;
template <typename T>
using Quat = omni::math::linalg::quat<T>;

template <typename T>
static CUDA_CALLABLE Vector3<T> quatRotate(const Quat<T> &q, const Vector3<T> &v)
{
    const Quat<T> &qNorm = q.GetNormalized();

    const T &q_w = qNorm.GetReal();
    const Vector3<T> &q_v = Vector3<T>(qNorm.GetImaginary());

    // Apply the rotation
    const Vector3<T> a = v * (2.0 * q_w * q_w - 1.0);
    const Vector3<T> b = 2.0 * q_w * GfCross(q_v, v);
    const Vector3<T> c = 2.0 * q_v * q_v.Dot(v);

    return a + b + c;
}

template <typename T>
static CUDA_CALLABLE Vector3<T> quatRotateInverse(const Quat<T> &q, const Vector3<T> &v)
{
    const Quat<T> &qNorm = q.GetNormalized();
    const Quat<T> &qConj = qNorm.GetConjugate(); // Conjugate for inverse

    const T &q_w = qConj.GetReal();
    const Vector3<T> &q_v = Vector3<T>(qConj.GetImaginary());

    // Apply the inverse rotation
    const Vector3<T> a = v * (2.0 * q_w * q_w - 1.0);
    const Vector3<T> b = 2.0 * q_w * GfCross(q_v, v);
    const Vector3<T> c = 2.0 * q_v * q_v.Dot(v);

    return a + b + c;
}

/**
 * @brief Reprojects a set of 2D coordinates onto a sphere at a given origin (lat, lon) and reference (sin, cos) latitude.
 * @see https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/include/common.h#L260
 */
template <typename T>
static CUDA_CALLABLE Vector2<T> reprojectEarth(const Vector3<T> &pos,
                                               const T origin_lat,
                                               const T origin_lon)
{
    constexpr T EARTH_RADIUS = 6353000; // meters

    const T x_rad = pos[1] / EARTH_RADIUS;
    const T y_rad = pos[0] / EARTH_RADIUS;
    const T c = std::sqrt(x_rad * x_rad + y_rad * y_rad);
    if (c == 0)
        return Vector2<T>(origin_lat, origin_lon);

    const T sin_lat = std::sin(origin_lat);
    const T cos_lat = std::cos(origin_lat);
    const T sin_c = std::sin(c);
    const T cos_c = std::cos(c);

    const T lat_rad = std::asin(cos_c * sin_lat + (x_rad * sin_c * cos_lat) / c);
    const T lon_rad = origin_lon + std::atan2(y_rad * sin_c, c * cos_lat * cos_c - x_rad * sin_lat * sin_c);
    return Vector2<T>(lat_rad, lon_rad);
}

template <typename T>
static CUDA_CALLABLE T getYawfromQuat(const Quat<T> &q)
{
    // yaw=arctan2(2⋅(w⋅z+x⋅y),1−2⋅(y2+z2))
    const T angle1 = 2.0f * (q.GetReal() * q.GetImaginary()[2] + q.GetImaginary()[0] * q.GetImaginary()[1]);
    const T angle2 = 1.0f - 2.0f * (q.GetImaginary()[1] * q.GetImaginary()[1] + q.GetImaginary()[2] * q.GetImaginary()[2]);
    return std::atan2(angle1, angle2);
}

template <typename T>
static CUDA_CALLABLE Vector3<T> getQuatAxis(const Quat<T> &q, const int axis = 2)
{
    if (axis == 0)
    {
        // X (1-2(y^2+z^2), 2(x*y+w*z), 2(x*z-w*y))
        T x = 1 - 2 * (q.GetImaginary()[1] * q.GetImaginary()[1] + q.GetImaginary()[2] * q.GetImaginary()[2]);
        T y = 2 * (q.GetImaginary()[0] * q.GetImaginary()[1] + q.GetReal() * q.GetImaginary()[2]);
        T z = 2 * (q.GetImaginary()[0] * q.GetImaginary()[2] - q.GetReal() * q.GetImaginary()[2]);
        return Vector3<T>(x, y, z);
    }
    else if (axis == 1)
    {
        // Y (2(x*y-w*z), 1-2(x^2+z^2), 2(y*z+w*x))
        T x = 2 * (q.GetImaginary()[0] * q.GetImaginary()[1] - q.GetReal() * q.GetImaginary()[2]);
        T y = 1 - 2 * (q.GetImaginary()[0] * q.GetImaginary()[0] + q.GetImaginary()[2] * q.GetImaginary()[2]);
        T z = 2 * (q.GetImaginary()[1] * q.GetImaginary()[2] + q.GetReal() * q.GetImaginary()[0]);
        return Vector3<T>(x, y, z);
    }
    else if (axis == 2)
    {
        // Z (2(x*z+w*y), 2(y*z-w*x), 1-2(x^2+y^2))
        T x = 2 * (q.GetImaginary()[0] * q.GetImaginary()[2] + q.GetReal() * q.GetImaginary()[1]);
        T y = 2 * (q.GetImaginary()[1] * q.GetImaginary()[2] - q.GetReal() * q.GetImaginary()[0]);
        T z = 1 - 2 * (q.GetImaginary()[0] * q.GetImaginary()[0] + q.GetImaginary()[1] * q.GetImaginary()[1]);
        return Vector3<T>(x, y, z);
    }
    return Vector3<T>(0, 0, 0);
}

// Returns scalar value constrained by (min_val, max_val)
template <typename Scalar>
static inline constexpr const Scalar &constrain(const Scalar &val, const Scalar &min_val, const Scalar &max_val)
{
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

// Simple lerp implementation for < c++20
template <typename T>
static inline T lerp(T a, T b, T t)
{
    return a + t * (b - a);
}

static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;

}

#endif