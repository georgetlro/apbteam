#ifndef avrconfig_h
#define avrconfig_h
/* avrconfig.h - motor/output configuration template. */
/* motor - Motor control module. {{{
 *
 * Copyright (C) 2011 Nicolas Schodet
 *
 * APBTeam:
 *        Web: http://apbteam.org/
 *      Email: team AT apbteam DOT org
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * }}} */

/* motor/output - Output module. */
/** Use Output Compare PWM output module. */
#define AC_OUTPUT_USE_PWM_OCR 1
/** Use Motor Power PWM output module. */
#define AC_OUTPUT_USE_PWM_MP 0
/** Define module and module index for each output. */
#define AC_OUTPUT_LIST (pwm_ocr, 0), (pwm_ocr, 1)

#endif /* avrconfig_h */
