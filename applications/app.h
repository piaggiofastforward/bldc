/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * app.h
 *
 *  Created on: 18 apr 2014
 *      Author: benjamin
 */

#ifndef APP_H_
#define APP_H_

#include "conf_general.h"

// Functions
void app_init(app_configuration *conf);
const app_configuration* app_get_configuration(void);
void app_set_configuration(app_configuration *conf);

// Custom apps
void app_i2cslave_init(void);
void app_i2cslave_configure(uint8_t controller_id);


#endif /* APP_H_ */
