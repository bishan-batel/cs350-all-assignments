///////////////////////////////////////////////////////////////////////////////
/// 
/// Authors: Trevor Sundberg
/// Copyright 2013, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------
bool ReportOpenGLErrors(const char* file, int line)
{
  // Get the error code from OpenGL
  GLenum errorCode = glGetError();

  // If there actually was an error...
  if (errorCode != GL_NO_ERROR)
  {
    // Store the last slash (if we don't find one, just start at the beginning!)
    const char* fileLastSlash = file;

    // We only want to get the last slash in the file to just show the file name
    while (*file != NULL)
    {
      // Get the current character
      char c = *file;

      // If the character is a slash...
      if (c == '\\' || c == '/')
      {
        // Update the last slash we've seen (move ahead of the slash by 1)
        fileLastSlash = file + 1;
      }

      // Move forward in the string
      ++file;
    }

    // Print the error out to the console and return that an error occurred
    PrintError("OpenGL Error %s(%d): %s\n", fileLastSlash, line, gluErrorString(errorCode));
    return true;
  }

  // No error occurred!
  return false;
}
