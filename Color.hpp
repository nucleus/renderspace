/*
 * Color.hpp
 *
 *  Created on: 10.01.2013
 *      Author: michael
 */

#ifndef COLOR_HPP_
#define COLOR_HPP_

#include "glm.hpp"
#include <cmath>

#define GAMMAANDTONEMAP false

namespace Color {
	/*
	 * Gamma-corrective conversion from [0.0, 1.0] float colors to [0,255] uchar colors.
	 */
	inline glm::vec3 delinearize(glm::vec3 lrgb) {
		return glm::vec3(pow(lrgb.x, 1/2.2), pow(lrgb.y, 1/2.2), pow(lrgb.z, 1/2.2));
	}

	/*
	 * Linearization from sRGB to linear RGB.
	 */
	inline glm::vec3 linearize(glm::vec3 srgb) {
		return glm::vec3(pow(srgb.x, 2.2), pow(srgb.y, 2.2), pow(srgb.z, 2.2));
	}

	/*
	 * Tonemapping / dynamic compression according to Reinhard's model.
	 */
	inline glm::vec3 tonemap(glm::vec3 col) {
		return glm::vec3(col.x/(0.5f+col.x), col.y/(0.5f+col.y), col.z/(0.5f+col.z));
	}

}

#endif /* COLOR_HPP_ */
