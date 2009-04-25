/*
             LUFA Library
     Copyright (C) Dean Camera, 2009.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2009  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Version constants for informational purposes and version-specific macro creation. This header file contains the
 *  current LUFA version number in several forms, for use in the user-application (for example, for printing out 
 *  whilst debugging, or for testing for version compatibility).
 */

#ifndef __LUFA_VERSION_H__
#define __LUFA_VERSION_H__

	/* Public Interface - May be used in end-application: */
		/* Macros: */
			/** Indicates the version number of the library, as an integer. */
			#define LUFA_VERSION_INTEGER     090209

			/** Indicates the version number of the library, as a string. */
			#define LUFA_VERSION_STRING      "090209"

#endif