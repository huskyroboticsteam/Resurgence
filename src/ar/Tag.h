#pragma once

#include <cstdint>

namespace AR
{
	enum TagOrientation
	{
		UP,
		RIGHT,
		DOWN,
		LEFT
	};

	struct Tag
	{
		const TagOrientation orientation;
		const uint16_t tag_data;
	};
}
