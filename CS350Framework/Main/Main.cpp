///////////////////////////////////////////////////////////////////////////////
///
/// Feel free to reorganize your application's logic
/// This is nothing more than an incredibly simple framework to get you started
/// 
/// Authors: Trevor Sundberg
/// Copyright 2013, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

// Function called by AntTweakBar to copy the content of a std::string handled
// by the AntTweakBar library to a std::string handled by your application
void TW_CALL CopyStdStringToClient(std::string& destinationClientString, const std::string& sourceLibraryString)
{
  destinationClientString = sourceLibraryString;
}

//-----------------------------------------------------------------------------
// Idle function where we update
void Idle(Application* application, clock_t& lastTime)
{
  // Determine the new clock ticks
  clock_t newTime = clock();

  // Get the difference between the new clock tick and the last clock tick
  clock_t ticks = newTime - lastTime;

  // Divide out by tick frequency to get the frame time passed
  float frameTime = ticks / (float)CLOCKS_PER_SEC;

  // Update the last tick count
  lastTime = newTime;

  // Now update the application with the frame time
  application->Update(frameTime);
}

//-----------------------------------------------------------------------------
// Display function where we draw
void Display(Application* application, SDL_Window* window)
{
  // Let the application draw and present to the screen
  application->Draw();

  // Draw all the AntTweakBar windows / ui
  TwDraw();

  // Finally, present the back buffer to the screen
  SDL_GL_SwapWindow(window);
}

//-----------------------------------------------------------------------------
// Window resize/reshape function
void Reshape(Application* application, int width, int height)
{
  // Tell AntTweakBar that our window has been resized
  TwWindowSize(width, height);

  // Prevent a divide by zero, when window is too short
  // (you cant make a window of zero width).
  if (height <= 0)
  {
    height = 1;
  }
  
  // Compute the aspect ratio as w/h
  float aspectRatio = width / (float)height;

  // Set the viewport to be the entire window
  glViewport(0, 0, width, height);
  
  // Let the application handle it's own resizing
  application->Reshape(width, height, aspectRatio);
}

//-----------------------------------------------------------------------------
// Ant-tweakbar doesn't understand SDL2's events (made for sdl1) and the dll patch for
// SDL2 has some weird performance issues. So instead handle the event here by translating.
// Function returns true if handled by ant-tweakbar.
bool TwCustomEventSDL(SDL_Event& event)
{ 
// Helper macro to convert a key value from SDL to TW with the same name (eg. SDLK_LEFT to TW_KEY_LEFT)
#define SdlKeyToTwKey(keyName)\
  case SDLK_##keyName:\
    key = TW_KEY_##keyName; \
    break; \

// Helper macro to convert a keypad in sdl reprentation to a regular key in sdl
// representation (so we can turn KeyPad1 to Key1 which ant-tweakbar understands)
#define KeyPadToKey(num)\
  case SDLK_KP_##num:\
    key = SDLK_##num;\
    break;

  switch(event.type)
  {
    case SDL_MOUSEMOTION:
      return TwMouseMotion(event.motion.x, event.motion.y) != 0;

    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:
    {
      TwMouseAction action;
      // Convert to button up/down (could do on the outside level but a lot of code is shared)
      if(event.type == SDL_MOUSEBUTTONDOWN)
        action = TW_MOUSE_PRESSED;
      else
        action = TW_MOUSE_RELEASED;

      // Convert which mouse button it was
      TwMouseButtonID button;
      if(event.button.button == SDL_BUTTON_RIGHT)
        button = TW_MOUSE_RIGHT;
      else if(event.button.button == SDL_BUTTON_LEFT)
        button = TW_MOUSE_LEFT;
      else if(event.button.button == SDL_BUTTON_MIDDLE)
        button = TW_MOUSE_MIDDLE;
      else
        button = TW_MOUSE_RIGHT;

      return TwMouseButton(action, button) != 0;
    }

    case SDL_MOUSEWHEEL:
    {
      // Ant-tweakbar uses an absolute wheel position while sdl just returns a delta
      static int s_WheelPos = 0;
      s_WheelPos += event.wheel.y;
      return TwMouseWheel(s_WheelPos) != 0;
    }      

    case SDL_KEYDOWN:
    {
      // Convert relevant modifier codes
      int mod = event.key.keysym.mod;
      switch(mod)
      {
        case KMOD_SHIFT:
          mod = TW_KMOD_SHIFT;
          break;
        case KMOD_CTRL:
          mod = TW_KMOD_CTRL;
          break;
        case KMOD_ALT:
          mod = TW_KMOD_ALT;
          break;
      }

      // Convert special key codes (F keys, Num-pad, Arrows, etc...)
      int key = event.key.keysym.sym;
      switch(key)
      {
        SdlKeyToTwKey(F1);
        SdlKeyToTwKey(F2);
        SdlKeyToTwKey(F3);
        SdlKeyToTwKey(F4);
        SdlKeyToTwKey(F5);
        SdlKeyToTwKey(F6);
        SdlKeyToTwKey(F7);
        SdlKeyToTwKey(F8);
        SdlKeyToTwKey(F9);
        SdlKeyToTwKey(F10);
        SdlKeyToTwKey(F11);
        SdlKeyToTwKey(F12);
        
        SdlKeyToTwKey(LEFT);
        SdlKeyToTwKey(RIGHT);
        SdlKeyToTwKey(UP);
        SdlKeyToTwKey(DOWN);

        SdlKeyToTwKey(HOME);
        SdlKeyToTwKey(END);

        KeyPadToKey(0);
        KeyPadToKey(1);
        KeyPadToKey(2);
        KeyPadToKey(3);
        KeyPadToKey(4);
        KeyPadToKey(5);
        KeyPadToKey(6);
        KeyPadToKey(7);
        KeyPadToKey(8);
        KeyPadToKey(9);

        KeyPadToKey(PERIOD);
        KeyPadToKey(PLUS);
        KeyPadToKey(MINUS);
        case SDLK_KP_DIVIDE:
          key = SDLK_SLASH;
          break;
        case SDLK_KP_MULTIPLY:
          key = SDLK_ASTERISK;
          break;
        case SDLK_KP_ENTER:
          key = SDLK_RETURN;
          break;
      }
      
      return TwKeyPressed(key, mod) != 0;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
// Message pump for the application
void MainLoop(SDL_Window* window, Application* application)
{
  clock_t lastTime = clock();

  bool quit = false;
  while(!quit)
  {
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
      bool handled = TwCustomEventSDL(event);
      // If ant-tweakbar handled the event then don't do anything (don't want clicks to fall through)
      if(handled)
        continue;

      switch(event.type)
      {
        case SDL_QUIT:
        {
          quit = true;
          break;
        }

        case SDL_MOUSEMOTION:
        {
          application->OnMouseMove(event.motion.x, event.motion.y);
          break;
        }
        
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
        {
          application->OnMouseInput(event.button.button, event.button.state == SDL_PRESSED, event.button.x, event.button.y);
          break;
        }

        case SDL_MOUSEWHEEL:
        {
          application->OnMouseScroll(event.wheel.x, event.wheel.y);
          break;
        }

        case SDL_KEYDOWN:
        {
          application->OnKeyDown(event.key.keysym.sym, 0, 0);
          break;
        }

        case SDL_KEYUP:
        {
          application->OnKeyUp(event.key.keysym.sym, 0, 0);
          break;
        }

        case SDL_WINDOWEVENT:
        {
          switch(event.window.event)
          {
            // Handle the window being resized
            case SDL_WINDOWEVENT_RESIZED:
              Reshape(application, event.window.data1, event.window.data2);
              break;
          }
          break;
        }
      }
    }

    Idle(application, lastTime);
    Display(application, window);
  }
}

//-----------------------------------------------------------------------------
// Main startup function to initialize SDL, create the window, initialize GL and ant-tweakbar
bool Startup(SDL_Window*& window, SDL_GLContext& glContext, int WindowWidth, int WindowHeight)
{
  if (SDL_Init(SDL_INIT_VIDEO) != 0){
    printf("SDL_Init Error: %s:\n", SDL_GetError());
    return false;
  }

  // Get the 1st monitor's size
  SDL_DisplayMode current;
  int shouldBeZero = SDL_GetCurrentDisplayMode(0, &current);
  if(shouldBeZero != 0)
  {
    printf("SDL_GetCurrentDisplayMode Error: %s:\n", SDL_GetError());
    return false;
  }
  
  // Create our window centered in the screen
  const int WindowStartX = (current.w - WindowWidth) / 2;
  const int WindowStartY = (current.h - WindowHeight) / 2;
  window = SDL_CreateWindow("CS350 Framework", WindowStartX, WindowStartY, WindowWidth, WindowHeight, 
    SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  if(window == nullptr)
  {
    printf("SDL_CreateWindow Error: %s:\n", SDL_GetError());
    return false;
  }

  // Create the open gl context
  glContext = SDL_GL_CreateContext(window);
  
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,  16);
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE,     8);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE,   8);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE,    8);
  SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE,   8);
  SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);

  // Setup all the open-gl states we want to use (ones that don't change in the lifetime of the application)
  // Note: These can be changed anywhere, but generally we don't change the back buffer color
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glCullFace(GL_FRONT);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);

  // Initialize AntTweakBar
  TwInit(TW_OPENGL, NULL);
  TwCopyStdStringToClientFunc(CopyStdStringToClient);
  TwWindowSize(WindowWidth, WindowHeight);

  // Check to see if we had any errors
  PrintOpenGLErrors();
  
  // We leave the console up since it's a useful debugging tool
  printf("OpenGL Info: %s\n", glGetString(GL_VERSION));
  printf("    Version: %s\n", glGetString(GL_VERSION));
  printf("     Vendor: %s\n", glGetString(GL_VENDOR));
  printf("   Renderer: %s\n", glGetString(GL_RENDERER));

  return true;
}

//-----------------------------------------------------------------------------
// Check to see if any unit tests should be run
bool CheckForUnitTests(int argc, char *argv[])
{
  // Initialize all unit tests (basically fill out an array of function pointers)
  InitializeAssignment1Tests();
  InitializeAssignment2Tests();
  InitializeAssignment3Tests();
  InitializeAssignment4Tests();
  InitializeAssignment5Tests();

  // If we don't have enough arguments then bail
  if(argc < 3)
    return false;


  // Assume that the first 2 arguments are the assignment number and the test number. Test number 0 and -1 have special meanings.
  int assignmentNumber = atoi(argv[1]);
  int testNumber = atoi(argv[2]);

  std::string outFilePath = FormatString("Test%d_%d.txt", assignmentNumber, testNumber);
  // The 3rd argument is an optional one specifying what the output file name should be for the unit tests
  char* outFile = NULL;
  if(argc >= 4 && argv[3] != NULL)
    outFilePath = argv[3];

  // Open the file we're writing to
  FILE* file;
  fopen_s(&file, outFilePath.c_str(), "w");

  // Bounds check the assignment number
  if(assignmentNumber < 0 || (int)mTestFns.size() < assignmentNumber)
    return false;

  AssignmentUnitTestList& list = mTestFns[assignmentNumber - 1];
  // Test -1 is special, it prints how many tests their are (so the unit tests can be more easily automated)
  if(testNumber == -1)
  {
    // In this case the output is captured from stdout
    printf("%d", static_cast<int>(list.size()));
  }
  // Test 0 is also special, it runs all unit test for that assignment.
  // This is the one that you are given a diff file for.
  else if(testNumber == 0)
  {
    for(size_t i = 0; i < list.size(); ++i)
    {
      UnitTestWrapper& wrapper = list[i];
      wrapper.mFn(wrapper.mTestName, -1, file);
    }
  }
  else
  {
    // The argument of 1 runs the 1st test (index 0)
    int actualTestNumber = testNumber - 1;
    if(0 <= actualTestNumber && actualTestNumber < (int)list.size())
    {
      UnitTestWrapper& wrapper = list[actualTestNumber];
      wrapper.Run(-1, file);
    }
  }

  fclose(file);
  return true;
}

//-----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  //unsigned int currState;
  //_clearfp();
  //_controlfp_s(&currState, _EM_INEXACT | _EM_UNDERFLOW, _MCW_EM);

  // If we ran unit tests then just exit
  if(CheckForUnitTests(argc, argv))
    return 0;

  // Starting width / height of the window
  int WindowWidth = 800;
  int WindowHeight = 600;

  SDL_Window* window;
  SDL_GLContext glContext;
  bool success = Startup(window, glContext, WindowWidth, WindowHeight);

  // If we failed to initialize for some reason then shutdown
  // (I'm not sure what happens if we try to shutdown a system
  // that never initialized, but I don't really care right now...)
  if(!success)
  {
    TwTerminate();
    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  Application* application = new Application();
  // Communicate to GL/Tweakbar/etc... the window size
  Reshape(application, WindowWidth, WindowHeight);
  // Run the main message pump
  MainLoop(window, application);
  delete application;
  
  TwTerminate();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
