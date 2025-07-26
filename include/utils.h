#pragma once

#include <iostream>
#include <raylib.h>

namespace phys
{

const Color ERROR{220, 0, 0, 0};
const Color SUCCESS{0, 220, 0, 0};
const Color INFO{200, 200, 100, 0};

inline void ClearStyles() { std::cout << "\033[0m"; }
inline void SetTextColor(Color col)
{
	std::cout << "\033[38;2;" << std::to_string(col.r) << ';'
			  << std::to_string(col.g) << ';' << std::to_string(col.b) << 'm';
}

} //namespace phys
