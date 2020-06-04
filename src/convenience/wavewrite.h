/*
 * Copyright (C) 2014 by Kyle Keen <keenerd@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
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
#ifndef __WAVEWRITE_H
#define __WAVEWRITE_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif


void executeInBackground( char * file, char * args, char * searchStr[], char * replaceStr[] );


/*!
 * helper functions to write and finalize wave headers
 *   with compatibility to some SDR programs - showing frequency:
 * raw sample data still have to be written by caller to FILE*.
 * call waveWriteHeader() before writing anything to to file
 * and call waveFinalizeHeader() afterwards,
 * AND count/increment the written raw size in variable 'waveDataSize'.
 * stdout/stdout can't be used, because seek to begin isn't possible.
 *
 */

extern uint32_t	waveDataSize;
void waveWriteHeader(unsigned samplerate, unsigned freq, int bitsPerSample, int numChannels, FILE * f);
void waveFinalizeHeader();

#ifdef __cplusplus
}
#endif

#endif /*__WAVEWRITE_H*/
