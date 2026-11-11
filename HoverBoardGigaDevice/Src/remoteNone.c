/*
* This file is part of the Hoverboard-Firmware-Hack-Gen2.x-GD32 project.
*
* Copyright (C) 2026 Damian Krawczyk
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// remoteNone.c : no remote control at all, the motors are controlled by the pilot only (PILOT_ONEWHEEL, PILOT_USER, ..)

#include "../Inc/defines.h"
#include "../Inc/it.h"

#ifdef REMOTE_NONE

void RemoteUpdate(void)
{
	ResetTimeout();	// Reset the pwm timout to avoid stopping motors
}

void RemoteCallback(void){};

#endif
