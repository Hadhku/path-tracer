// Copyright (c) 2025 Thomas Klietsch, all rights reserved.
//
// Licensed under the GNU Lesser General Public License, version 3.0 or later
//
// This program is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or ( at your option ) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General
// Public License along with this program.If not, see < https://www.gnu.org/licenses/>. 

#pragma once

#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

#include "../bxdf/polymorphic.hpp"
#include "../colour/colour.hpp"
#include "../epsilon.hpp"
#include "../mathematics/double3.hpp"
#include "../random/mersenne.hpp"
#include "../ray/section.hpp"
#include "../render/camera.hpp"
#include "../render/config.hpp"
#include "../render/scene.hpp"
#include "../render/sensor.hpp"

namespace Integrator
{

	class PT
	{

	private:

		std::uint8_t const max_path_length{ 3 };
		std::uint16_t const max_samples{ 1 };

		Render::Camera const camera;
		Render::Scene const scene;
		Render::Sensor& sensor;

		Random::Mersenne prng;

		// Use light sampling on diffuse (non-dirac) surfaces.
		bool const f_NEE{ true };

	public:

		PT() = delete;

		PT(
			Render::Camera const& camera,
			Render::Sensor& sensor,
			Render::Scene const& scene,
			Render::Config const& config,
			bool const f_NEE = true
		)
			: camera( camera )
			, sensor( sensor )
			, scene( scene )
			, max_path_length( config.max_path_length )
			, max_samples( config.max_samples )
			, f_NEE( f_NEE )
		{
		};

		void process(
			std::uint16_t const x,
			std::uint16_t const y
		)
		{
			prng = Random::Mersenne( ( ( x + 1 ) * 0x1337 ) + ( ( y + 1 ) * 0xbeef ) );

			Colour accumulate( Colour::Black );

			for ( std::uint16_t sample{ 0 }; sample < max_samples; ++sample )
				accumulate += f_NEE
				? pt_nee( x, y )
				: pt_basic( x, y );

			sensor.pixel( x, y, accumulate );
		};

	private:

		// Unless an emitter is intersected, traced path is black.
		Colour pt_basic(
			std::uint16_t const x,
			std::uint16_t const y
		)
		{
			Ray::Section ray = camera.generate_ray( x, y, prng );

			std::uint8_t depth{ 1 };

			Colour throughput{ Colour::White };

			// Random emitter, and random point on it
			std::uint32_t const emitter_id = scene.random_emitter( prng );
			auto const [p_emitter, emitter_select_probability]
				= scene.emitter( emitter_id );
			auto const emitter_point = p_emitter->sample( prng );

			// Trace loop
			while ( 1 )
			{
				auto const [f_hit, hit_distance, idata]
					= scene.intersect( ray );
				if ( !f_hit )
					return Colour::Black;

				std::shared_ptr<BxDF::Polymorphic> const& p_material = scene.material( idata.material_id );
				auto const [bxdf_colour, bxdf_direction, bxdf_event, bxdf_pdf_W, bxdf_cos_theta]
					= p_material->sample( idata, prng );

				switch ( bxdf_event )
				{
					default:
					case BxDF::Event::None:
					{
						return Colour::Black;
					}
					case BxDF::Event::Emission:
					{
						auto const [p_emitter_hit, select_prb]
							= scene.emitter( p_material->emitter_id() );
						Colour const emission = p_emitter_hit->radiance( idata.point, -ray.direction );
						return throughput * emission;
					}
					case BxDF::Event::Diffuse:
					{
						throughput *= bxdf_colour / ( bxdf_pdf_W / bxdf_cos_theta );
						break;
					}
					case BxDF::Event::Reflect:
					{
						throughput *= bxdf_colour;
						break;
					}
				} // end switch

				if ( ++depth > max_path_length )
					return Colour::Black;

				ray = Ray::Section( idata.point, bxdf_direction, EPSILON_RAY );
			} // end trace loop
		};

		// Unless an emitter is intersected, or NEE samples are unobstructed (no occlusions),
		// traced path is black.
		Colour pt_nee(
			std::uint16_t const x,
			std::uint16_t const y
		)
		{
			Ray::Section ray = camera.generate_ray( x, y, prng );

			std::uint8_t depth{ 1 };

			Colour throughput{ Colour::White }; // Affected by each material on the traced path
			Colour accumulate( Colour::Black ); // Sum of all NEE contributions

			// Random emitter, and random point on it
			std::uint32_t const emitter_id = scene.random_emitter( prng );
			auto const [p_emitter, emitter_select_probability]
				= scene.emitter( emitter_id );
			auto const emitter_point = p_emitter->sample( prng );

			bool f_previous_dirac{ false }; // Dirac state of previous intersection material
			bool f_dirac_path{ true }; // Whether or not the whole path is dirac materials

			// Trace loop
			while ( 1 )
			{
				auto const [f_hit, hit_distance, idata]
					= scene.intersect( ray );
				if ( !f_hit )
					return accumulate;

				std::shared_ptr<BxDF::Polymorphic> const& p_material = scene.material( idata.material_id );
				auto const [bxdf_colour, bxdf_direction, bxdf_event, bxdf_pdf_W, bxdf_cos_theta]
					= p_material->sample( idata, prng );

				switch ( bxdf_event )
				{
					default:
					case BxDF::Event::None:
					{
						return accumulate;
					}
					case BxDF::Event::Emission:
					{
						if ( depth == 1 || f_dirac_path || f_previous_dirac )
						{
							// Evaluate if:
							//   * direct hit on emitter from initial camera ray
							//     ( no material or NEE has been done )
							//   * a complete dirac path (no NEE performed)
							//   * previous surface was dirac (e.g. no NEE)
							// Otherwise pixel is oversampled with NEE on diffuse surfaces
							auto const [p_emitter_hit, select_prb]
								= scene.emitter( p_material->emitter_id() );
							Colour const emission = p_emitter_hit->radiance( idata.point, -ray.direction );
							return accumulate + throughput * emission;
						}
						return accumulate;
					}
					case BxDF::Event::Diffuse:
					{
						// Next event estimator (NEE)
						Double3 const delta = emitter_point - idata.point;
						Double3 const evaluate_direction = delta.normalise();
						std::double_t const evaluate_distance = delta.magnitude();
						Ray::Section const evaluate_ray( idata.point, evaluate_direction, EPSILON_RAY );
						if ( !scene.occluded( evaluate_ray, evaluate_distance - 2. * EPSILON_RAY ) )
						{
							auto const [nee_factor, nee_pdf_W, nee_cos_theta]
								= p_material->evaluate( evaluate_direction, idata.from_direction, idata );
							auto const [emitter_pdf_A, emitter_cos_theta]
								= p_emitter->pdf_Le( emitter_point, -evaluate_direction );

							if ( ( nee_cos_theta > 0.f ) && ( emitter_cos_theta > 0.f ) )
							{
								std::double_t const emitter_pdf_A_to_W = emitter_pdf_A * emitter_select_probability * evaluate_distance * evaluate_distance / emitter_cos_theta;
								Colour const emission = p_emitter->radiance( emitter_point, -evaluate_direction );
								accumulate += throughput * emission * nee_factor * ( nee_cos_theta / emitter_pdf_A_to_W );
							}
						}

						// Updated after NEE
						throughput *= bxdf_colour / ( bxdf_pdf_W / bxdf_cos_theta );
						f_previous_dirac = false;
						f_dirac_path = false;
						break;
					}
					case BxDF::Event::Reflect:
					{
						throughput *= bxdf_colour;
						f_previous_dirac = true;
						break;
					}
				} // end switch

				if ( ++depth > max_path_length )
					return accumulate;

				ray = Ray::Section( idata.point, bxdf_direction, EPSILON_RAY );
			} // end trace loop
		};

	}; // end pt class

};
