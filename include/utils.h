#pragma once

#include <iostream>
#include <raylib.h>
#include <raymath.h>
#include <string>

namespace phys
{

constexpr Color ERROR{220, 0, 0, 0};
constexpr Color SUCCESS{0, 220, 0, 0};
constexpr Color INFO{200, 200, 100, 0};

inline auto operator-(const Vector3 vec) { return Vector3Negate(vec); }
/** @brief Returns true if vec1 is equal to either vec2 or its negative. */
inline auto Vector3Equivalent(const Vector3 vec1, const Vector3 vec2) -> bool
{
	return (vec1 == vec2) || (vec1 == -vec2);
}
/** @brief Returns true if the point 'c' is on the line segment described by
 *         points 'a' and 'b'.
 *   @note This function assumes that all input points lie on a shared line in
 *         3D space
 */
inline auto IsPointOnSegment(Vector3 a, Vector3 b, Vector3 c) -> bool
{
	return (Vector3DotProduct(a - b, c - b) >= 0.0f)
		   && (Vector3DotProduct(b - a, c - a) >= 0.0f);
}

/** @brief Clears all text styles applied to console output. */
constexpr void ClearStyles() { std::cout << "\033[0m"; }
/** @brief Applies Color 'col' to all console output going forward. */
constexpr void SetTextColor(Color col)
{
	std::cout
		<< "\033[38;2;"
		<< std::to_string(col.r)
		<< ';'
		<< std::to_string(col.g)
		<< ';'
		<< std::to_string(col.b)
		<< 'm';
}

} //namespace phys
