#pragma once

#include <memory>

#include "../colour/colour.hpp"
#include "../render/config.hpp"

namespace Render
{

	class Sensor final
	{

	private:

		// Data buffer
		std::shared_ptr<Colour[]> p_pixel{ nullptr };

		std::uint16_t const image_width{ 0 };
		std::uint16_t const image_height{ 0 };

		std::double_t const scalar{ 1. };

	public:

		Sensor() {};

		Sensor(
			Render::Config const& config
		)
			: image_width( config.image_width )
			, image_height( config.image_height )
			, scalar( 1. / std::max<std::uint16_t>( 1, config.max_samples ) )
		{
			if ( config.max_samples < 1 )
				throw std::invalid_argument( "Invalid config. Needs at least one (1) sample" );
			// Setup sensor data with zeroes (black)
			p_pixel = std::make_shared<Colour[]>( image_width * image_height, Colour::Black );
			// Nullptr if unable to construct
			if ( !p_pixel )
				throw std::invalid_argument( "Out of memory sensor data!" );
		};

		void pixel(
			std::uint16_t const px,
			std::uint16_t const py,
			Colour const& colour
		)
		{
			// Assumes that the method is used in a thread safe manner
			if ( ( px >= image_width ) || ( py >= image_height ) )
				return;
			p_pixel[px + py * image_width] = colour;
		};

		Colour get_colour(
			std::uint16_t const px,
			std::uint16_t const py
		) const
		{
			// Assumes that the method is used in a thread safe manner
			if ( ( px >= image_width ) || ( py >= image_height ) )
				return Colour::Black;
			return p_pixel[px + py * image_width] * scalar;
		};

	};

};
