///////////////////////////////////////////////////////////////////////////////
/// 
/// Authors: Trevor Sundberg
/// Copyright 2013, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

// Macro that we can use to report OpenGL errors
// This is slow, but it can be used for debugging
// Returns true if an error occurred, false otherwise
#define PrintOpenGLErrors() ReportOpenGLErrors(__FILE__, __LINE__)

// Reports OpenGL errors to the console
// Returns true if an error occurred, false otherwise
bool ReportOpenGLErrors(const char* file, int line);
