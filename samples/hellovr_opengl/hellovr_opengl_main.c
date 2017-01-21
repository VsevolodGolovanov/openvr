//========= Copyright Valve Corporation ============//

#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#include <GL/glu.h>
#include <stdio.h>
//#include <string>
#include <stdlib.h>

#include "openvr_capi_fixed.h"

#include "shared/lodepng.h"
#include "shared/Matrices.h"
#include "shared/pathtools.h"

#if defined(POSIX)
#include "unistd.h"
#endif

#ifndef _WIN32
#define APIENTRY
#endif

static void ThreadSleep(unsigned long nMilliseconds)
{
#if defined(_WIN32)
	Sleep(nMilliseconds);
#elif defined(POSIX)
	usleep(nMilliseconds * 1000);
#endif
}

typedef struct CGLRenderModel
{
	GLuint m_glVertBuffer;
	GLuint m_glIndexBuffer;
	GLuint m_glVertArray;
	GLuint m_glTexture;
	GLsizei m_unVertexCount;
	char* m_sModelName;
} CGLRenderModel;

static void CGLRenderModel_init(CGLRenderModel* self, char* sRenderModelName);
static void CGLRenderModel_destroy(CGLRenderModel* self);
static bool CGLRenderModel_BInit(CGLRenderModel* self, const RenderModel_t* vrModel, const RenderModel_TextureMap_t* vrDiffuseTexture);
static void CGLRenderModel_Draw(CGLRenderModel* self);

static bool g_bPrintf = true;

//k_unMaxTrackedDeviceCount
#define MAX_TRACKED_DEVICE_COUNT 16

typedef struct VertexDataScene
{
	Vector3 position;
	Vector2 texCoord;
} VertexDataScene;

typedef struct VertexDataWindow
{
	Vector2 position;
	Vector2 texCoord;
} VertexDataWindow;

typedef struct FramebufferDesc
{
	GLuint m_nDepthBufferId;
	GLuint m_nRenderTextureId;
	GLuint m_nRenderFramebufferId;
	GLuint m_nResolveTextureId;
	GLuint m_nResolveFramebufferId;
} FramebufferDesc;

//-----------------------------------------------------------------------------
// Purpose:
//------------------------------------------------------------------------------
typedef struct CMainApplication
{
	bool m_bDebugOpenGL;
	bool m_bVerbose;
	bool m_bPerf;
	bool m_bVblank;
	bool m_bGlFinishHack;

	VR_IVRSystem* m_pHMD;
	VR_IVRRenderModels* m_pRenderModels;
	VR_IVRCompositor* m_pCompositor;
	char* m_strDriver;
	char* m_strDisplay;
	TrackedDevicePose_t m_rTrackedDevicePose[MAX_TRACKED_DEVICE_COUNT];
	Matrix4 m_rmat4DevicePose[MAX_TRACKED_DEVICE_COUNT];
	bool m_rbShowTrackedDevice[MAX_TRACKED_DEVICE_COUNT];

	// SDL bookkeeping
	SDL_Window *m_pCompanionWindow;
	uint32_t m_nCompanionWindowWidth;
	uint32_t m_nCompanionWindowHeight;

	SDL_GLContext m_pContext;

	// OpenGL bookkeeping
	int m_iTrackedControllerCount;
	int m_iTrackedControllerCount_Last;
	int m_iValidPoseCount;
	int m_iValidPoseCount_Last;
	bool m_bShowCubes;

	char* m_strPoseClasses;                            // what classes we saw poses for this frame
	char m_rDevClassChar[MAX_TRACKED_DEVICE_COUNT];   // for each device, a character representing its class

	int m_iSceneVolumeWidth;
	int m_iSceneVolumeHeight;
	int m_iSceneVolumeDepth;
	float m_fScaleSpacing;
	float m_fScale;

	int m_iSceneVolumeInit;                                  // if you want something other than the default 20x20x20

	float m_fNearClip;
	float m_fFarClip;

	GLuint m_iTexture;

	unsigned int m_uiVertcount;

	GLuint m_glSceneVertBuffer;
	GLuint m_unSceneVAO;
	GLuint m_unCompanionWindowVAO;
	GLuint m_glCompanionWindowIDVertBuffer;
	GLuint m_glCompanionWindowIDIndexBuffer;
	unsigned int m_uiCompanionWindowIndexSize;

	GLuint m_glControllerVertBuffer;
	GLuint m_unControllerVAO;
	unsigned int m_uiControllerVertcount;

	Matrix4 m_mat4HMDPose;
	Matrix4 m_mat4eyePosLeft;
	Matrix4 m_mat4eyePosRight;

	Matrix4 m_mat4ProjectionCenter;
	Matrix4 m_mat4ProjectionLeft;
	Matrix4 m_mat4ProjectionRight;

	GLuint m_unSceneProgramID;
	GLuint m_unCompanionWindowProgramID;
	GLuint m_unControllerTransformProgramID;
	GLuint m_unRenderModelProgramID;

	GLint m_nSceneMatrixLocation;
	GLint m_nControllerMatrixLocation;
	GLint m_nRenderModelMatrixLocation;

	FramebufferDesc m_leftEyeDesc;
	FramebufferDesc m_rightEyeDesc;

	uint32_t m_nRenderWidth;
	uint32_t m_nRenderHeight;

	CGLRenderModel* m_vecRenderModels[MAX_TRACKED_DEVICE_COUNT]; // FIXME free models
	CGLRenderModel* m_rTrackedDeviceToRenderModel[MAX_TRACKED_DEVICE_COUNT];
} CMainApplication;

static void CMainApplication_init(CMainApplication* self, int argc, char *argv[]);

static bool CMainApplication_BInit(CMainApplication* self);
static bool CMainApplication_BInitGL(CMainApplication* self);
static bool CMainApplication_BInitCompositor(CMainApplication* self);

static void CMainApplication_SetupRenderModels(CMainApplication* self);

static void CMainApplication_Shutdown(CMainApplication* self);

static void CMainApplication_RunMainLoop(CMainApplication* self);
static bool CMainApplication_HandleInput(CMainApplication* self);
static void CMainApplication_ProcessVREvent(CMainApplication* self, const VREvent_t* event);
static void CMainApplication_RenderFrame(CMainApplication* self);

static bool CMainApplication_SetupTexturemaps(CMainApplication* self);

static void CMainApplication_SetupScene(CMainApplication* self);
static void CMainApplication_AddCubeToScene(CMainApplication* self, const Matrix4 * const mat, float** vertdata, int* vertdatasize);

static void CMainApplication_RenderControllerAxes(CMainApplication* self);

static bool CMainApplication_SetupStereoRenderTargets(CMainApplication* self);
static void CMainApplication_SetupCompanionWindow(CMainApplication* self);
static void CMainApplication_SetupCameras(CMainApplication* self);

static void CMainApplication_RenderStereoTargets(CMainApplication* self);
static void CMainApplication_RenderCompanionWindow(CMainApplication* self);
static void CMainApplication_RenderScene(CMainApplication* self, Hmd_Eye nEye);

static Matrix4 CMainApplication_GetHMDMatrixProjectionEye(CMainApplication* self, Hmd_Eye nEye);
static Matrix4 CMainApplication_GetHMDMatrixPoseEye(CMainApplication* self, Hmd_Eye nEye);
static Matrix4 CMainApplication_GetCurrentViewProjectionMatrix(CMainApplication* self, Hmd_Eye nEye);
static void CMainApplication_UpdateHMDMatrixPose(CMainApplication* self);

static Matrix4 CMainApplication_ConvertSteamVRMatrixToMatrix4(CMainApplication* self, const HmdMatrix34_t* matPose);

static GLuint CMainApplication_CompileGLShader(CMainApplication* self, const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader);
static bool CMainApplication_CreateAllShaders(CMainApplication* self);

static void CMainApplication_SetupRenderModelForTrackedDevice(CMainApplication* self, TrackedDeviceIndex_t unTrackedDeviceIndex);
static CGLRenderModel* CMainApplication_FindOrLoadRenderModel(CMainApplication* self, char *pchRenderModelName);

static bool CMainApplication_CreateFrameBuffer(CMainApplication* self, int nWidth, int nHeight, FramebufferDesc* framebufferDesc);


//-----------------------------------------------------------------------------
// Purpose: Outputs a set of optional arguments to debugging output, using
//          the printf format setting specified in fmt*.
//-----------------------------------------------------------------------------
static void dprintf(const char *fmt, ...)
{
	va_list args;
	char buffer[2048];

	va_start(args, fmt);
	vsprintf_s(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	if (g_bPrintf)
		printf("%s", buffer);

	OutputDebugStringA(buffer);
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void addToArrayFloat(const float vals[], const int valCount, float** data, int* const datasize)
{
	int newsize = *datasize + valCount;
	int newfloatsize = newsize * sizeof(float);

	if (*datasize == 0)
		*data = malloc(newfloatsize);
	else
		*data = realloc(*data, newfloatsize);
	for (int i = 0; i < valCount; ++i)
		(*data)[*datasize + i] = vals[i];

	*datasize = newsize;
}

void addToArrayFloat3(float** data, int* const datasize, float fl0, float fl1, float fl2)
{
	float arr[] = { fl0, fl1, fl2 };
	addToArrayFloat(arr, 3, data, datasize);
}

void addToArrayFloat5(float** data, int* const datasize, float fl0, float fl1, float fl2, float fl3, float fl4)
{
	float arr[] = { fl0, fl1, fl2, fl3, fl4 };
	addToArrayFloat(arr, 5, data, datasize);
}

//-----------------------------------------------------------------------------
// Purpose: Constructor
//-----------------------------------------------------------------------------
void CMainApplication_init(CMainApplication* self, int argc, char *argv[])
{
	self->m_pCompanionWindow = NULL;
	self->m_pContext = NULL;
	self->m_nCompanionWindowWidth = 640;
	self->m_nCompanionWindowHeight = 320;
	self->m_unSceneProgramID = 0;
	self->m_unCompanionWindowProgramID = 0;
	self->m_unControllerTransformProgramID = 0;
	self->m_unRenderModelProgramID = 0;
	self->m_pHMD = NULL;
	self->m_pRenderModels = NULL;
	self->m_bDebugOpenGL = false;
	self->m_bVerbose = false;
	self->m_bPerf = false;
	self->m_bVblank = false;
	self->m_bGlFinishHack = true;
	self->m_glControllerVertBuffer = 0;
	self->m_unControllerVAO = 0;
	self->m_unCompanionWindowVAO = 0;
	self->m_unSceneVAO = 0;
	self->m_nSceneMatrixLocation = -1;
	self->m_nControllerMatrixLocation = -1;
	self->m_nRenderModelMatrixLocation = -1;
	self->m_iTrackedControllerCount = 0;
	self->m_iTrackedControllerCount_Last = -1;
	self->m_iValidPoseCount = 0;
	self->m_iValidPoseCount_Last = -1;
	self->m_iSceneVolumeInit = 20;
	self->m_strPoseClasses = "";
	self->m_bShowCubes = true;
	self->m_strDriver = NULL;
	self->m_strDisplay = NULL;
	memset(self->m_vecRenderModels, 0, k_unMaxTrackedDeviceCount * sizeof(void*));

	for (int i = 1; i < argc; ++i)
	{
		if (!stricmp(argv[i], "-gldebug"))
		{
			self->m_bDebugOpenGL = true;
		}
		else if (!stricmp(argv[i], "-verbose"))
		{
			self->m_bVerbose = true;
		}
		else if (!stricmp(argv[i], "-novblank"))
		{
			self->m_bVblank = false;
		}
		else if (!stricmp(argv[i], "-noglfinishhack"))
		{
			self->m_bGlFinishHack = false;
		}
		else if (!stricmp(argv[i], "-noprintf"))
		{
			g_bPrintf = false;
		}
		else if (!stricmp(argv[i], "-cubevolume") && (argc > i + 1) && (*argv[i + 1] != '-'))
		{
			self->m_iSceneVolumeInit = atoi(argv[i + 1]);
			i++;
		}
	}
	// other initialization tasks are done in BInit
	memset(self->m_rDevClassChar, 0, sizeof(self->m_rDevClassChar));
};

//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property. Freeing the returned char* is on the caller.
//-----------------------------------------------------------------------------
char* GetTrackedDeviceString(VR_IVRSystem* pHmd, TrackedDeviceIndex_t unDevice, TrackedDeviceProperty prop, TrackedPropertyError *peError)
{
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
	if (unRequiredBufferLen == 0)
		return "";

	char *pchBuffer = malloc(unRequiredBufferLen);
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
	return pchBuffer;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
static bool CMainApplication_BInit(CMainApplication* self)
{
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
	{
		printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}

	// Loading the SteamVR Runtime
	EVRInitError eError = EVRInitError_VRInitError_None;
	// VR_Init has been inlined in openvr.h to help with versioning issues.
	// The capi does not currently have that implemented, but you can follow
	// what openvr.h does and do something similar.
	// https://steamcommunity.com/app/358720/discussions/0/405692758722144628/
	VR_InitInternal(&eError, EVRApplicationType_VRApplication_Scene);

	if (eError != EVRInitError_VRInitError_None)
	{
		self->m_pHMD = NULL;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", VR_GetVRInitErrorAsEnglishDescription(eError));
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
		return false;
	}

	// not gonna bother with VR_IsInterfaceVersionValid - VR_GetGenericInterface error checking&reporting will suffice

	// VR_GetGenericInterface returns a pointer to a class, and we could get its vtable easily (just cast the returned
	// instance to (VR_IVRSystem**), dereference once and use). But the problem is in passing "this" and arguments to
	// mirror the used C++ calling convention, which would be prone to breaking and non-portable.
	// Better use the undocumented method of passing version prepended by "FnTable:".
	char tableName[128];
	sprintf_s(tableName, sizeof(tableName), "%s%s", VR_IVRFnTable_Prefix, IVRSystem_Version);
	self->m_pHMD = (VR_IVRSystem*)VR_GetGenericInterface(tableName, &eError);
	if (eError != EVRInitError_VRInitError_None || !self->m_pHMD)
	{
		self->m_pHMD = NULL;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to get system interface: %s", VR_GetVRInitErrorAsEnglishDescription(eError));
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Init Failed", buf, NULL);
		return false;
	}

	sprintf_s(tableName, sizeof(tableName), "%s%s", VR_IVRFnTable_Prefix, IVRRenderModels_Version);
	self->m_pRenderModels = (VR_IVRRenderModels*)VR_GetGenericInterface(tableName, &eError);
	if (eError != EVRInitError_VRInitError_None || !self->m_pRenderModels)
	{
		self->m_pHMD = NULL;
		self->m_pRenderModels = NULL;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to get render model interface: %s", VR_GetVRInitErrorAsEnglishDescription(eError));
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Init Failed", buf, NULL);
		return false;
	}

	int nWindowPosX = 700;
	int nWindowPosY = 100;
	Uint32 unWindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
	//SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY );
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
	if (self->m_bDebugOpenGL)
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);

	self->m_pCompanionWindow = SDL_CreateWindow("hellovr", nWindowPosX, nWindowPosY, self->m_nCompanionWindowWidth, self->m_nCompanionWindowHeight, unWindowFlags);
	if (self->m_pCompanionWindow == NULL)
	{
		printf("%s - Window could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}

	self->m_pContext = SDL_GL_CreateContext(self->m_pCompanionWindow);
	if (self->m_pContext == NULL)
	{
		printf("%s - OpenGL context could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}

	glewExperimental = GL_TRUE;
	GLenum nGlewError = glewInit();
	if (nGlewError != GLEW_OK)
	{
		printf("%s - Error initializing GLEW! %s\n", __FUNCTION__, glewGetErrorString(nGlewError));
		return false;
	}
	glGetError(); // to clear the error caused deep in GLEW

	if (SDL_GL_SetSwapInterval(self->m_bVblank ? 1 : 0) < 0)
	{
		printf("%s - Warning: Unable to set VSync! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}

	TrackedPropertyError peError = 0;
	self->m_strDriver = GetTrackedDeviceString(self->m_pHMD, k_unTrackedDeviceIndex_Hmd, ETrackedDeviceProperty_Prop_TrackingSystemName_String, &peError);
	self->m_strDisplay = GetTrackedDeviceString(self->m_pHMD, k_unTrackedDeviceIndex_Hmd, ETrackedDeviceProperty_Prop_SerialNumber_String, &peError);

	char strWindowTitle[1024];
	sprintf_s(strWindowTitle, sizeof(strWindowTitle), "hellovr - %s %s", self->m_strDriver, self->m_strDisplay);
	SDL_SetWindowTitle(self->m_pCompanionWindow, strWindowTitle);

	// cube array
	self->m_iSceneVolumeWidth = self->m_iSceneVolumeInit;
	self->m_iSceneVolumeHeight = self->m_iSceneVolumeInit;
	self->m_iSceneVolumeDepth = self->m_iSceneVolumeInit;

	self->m_fScale = 0.3f;
	self->m_fScaleSpacing = 4.0f;

	self->m_fNearClip = 0.1f;
	self->m_fFarClip = 30.0f;

	self->m_iTexture = 0;
	self->m_uiVertcount = 0;

	if (!CMainApplication_BInitGL(self))
	{
		printf("%s - Unable to initialize OpenGL!\n", __FUNCTION__);
		return false;
	}

	if (!CMainApplication_BInitCompositor(self))
	{
		printf("%s - Failed to initialize VR Compositor!\n", __FUNCTION__);
		return false;
	}

	return true;
}


//-----------------------------------------------------------------------------
// Purpose: Outputs the string in message to debugging output.
//          All other parameters are ignored.
//          Does not return any meaningful value or reference.
//-----------------------------------------------------------------------------
void APIENTRY DebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const char* message, const void* userParam)
{
	dprintf("GL Error: %s\n", message);
}


//-----------------------------------------------------------------------------
// Purpose: Initialize OpenGL. Returns true if OpenGL has been successfully
//          initialized, false if shaders could not be created.
//          If failure occurred in a module other than shaders, the function
//          may return true or throw an error. 
//-----------------------------------------------------------------------------
bool CMainApplication_BInitGL(CMainApplication* self)
{
	if (self->m_bDebugOpenGL)
	{
		glDebugMessageCallback((GLDEBUGPROC)DebugCallback, NULL);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	}

	if (!CMainApplication_CreateAllShaders(self))
		return false;

	CMainApplication_SetupTexturemaps(self);
	CMainApplication_SetupScene(self);
	CMainApplication_SetupCameras(self);
	CMainApplication_SetupStereoRenderTargets(self);
	CMainApplication_SetupCompanionWindow(self);

	CMainApplication_SetupRenderModels(self);

	return true;
}


//-----------------------------------------------------------------------------
// Purpose: Initialize Compositor. Returns true if the compositor was
//          successfully initialized, false otherwise.
//-----------------------------------------------------------------------------
bool CMainApplication_BInitCompositor(CMainApplication* self)
{
	EVRInitError eError = EVRInitError_VRInitError_None;
	char tableName[128];
	sprintf_s(tableName, sizeof(tableName), "%s%s", VR_IVRFnTable_Prefix, IVRCompositor_Version);
	self->m_pCompositor = (VR_IVRCompositor*)VR_GetGenericInterface(tableName, &eError);
	if (eError != EVRInitError_VRInitError_None || !self->m_pCompositor)
	{
		self->m_pCompositor = NULL;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to get compositor interface: %s", VR_GetVRInitErrorAsEnglishDescription(eError));
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Init Failed", buf, NULL);
		return false;
	}
	return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_Shutdown(CMainApplication* self)
{
	if (self->m_pHMD)
	{
		VR_ShutdownInternal();
		self->m_pHMD = NULL;
	}

	if (self->m_strDriver)
		free(self->m_strDriver);
	if (self->m_strDisplay)
		free(self->m_strDisplay);

	for (unsigned int i = 0; i < k_unMaxTrackedDeviceCount; ++i)
	{
		free(self->m_vecRenderModels[i]);
	}

	if (self->m_pContext)
	{
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_FALSE);
		glDebugMessageCallback(NULL, NULL);
		glDeleteBuffers(1, &self->m_glSceneVertBuffer);

		if (self->m_unSceneProgramID)
		{
			glDeleteProgram(self->m_unSceneProgramID);
		}
		if (self->m_unControllerTransformProgramID)
		{
			glDeleteProgram(self->m_unControllerTransformProgramID);
		}
		if (self->m_unRenderModelProgramID)
		{
			glDeleteProgram(self->m_unRenderModelProgramID);
		}
		if (self->m_unCompanionWindowProgramID)
		{
			glDeleteProgram(self->m_unCompanionWindowProgramID);
		}

		glDeleteRenderbuffers(1, &self->m_leftEyeDesc.m_nDepthBufferId);
		glDeleteTextures(1, &self->m_leftEyeDesc.m_nRenderTextureId);
		glDeleteFramebuffers(1, &self->m_leftEyeDesc.m_nRenderFramebufferId);
		glDeleteTextures(1, &self->m_leftEyeDesc.m_nResolveTextureId);
		glDeleteFramebuffers(1, &self->m_leftEyeDesc.m_nResolveFramebufferId);

		glDeleteRenderbuffers(1, &self->m_rightEyeDesc.m_nDepthBufferId);
		glDeleteTextures(1, &self->m_rightEyeDesc.m_nRenderTextureId);
		glDeleteFramebuffers(1, &self->m_rightEyeDesc.m_nRenderFramebufferId);
		glDeleteTextures(1, &self->m_rightEyeDesc.m_nResolveTextureId);
		glDeleteFramebuffers(1, &self->m_rightEyeDesc.m_nResolveFramebufferId);

		if (self->m_unCompanionWindowVAO != 0)
		{
			glDeleteVertexArrays(1, &self->m_unCompanionWindowVAO);
		}
		if (self->m_unSceneVAO != 0)
		{
			glDeleteVertexArrays(1, &self->m_unSceneVAO);
		}
		if (self->m_unControllerVAO != 0)
		{
			glDeleteVertexArrays(1, &self->m_unControllerVAO);
		}
	}

	if (self->m_pCompanionWindow)
	{
		SDL_DestroyWindow(self->m_pCompanionWindow);
		self->m_pCompanionWindow = NULL;
	}

	SDL_Quit();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication_HandleInput(CMainApplication* self)
{
	SDL_Event sdlEvent;
	bool bRet = false;

	while (SDL_PollEvent(&sdlEvent) != 0)
	{
		if (sdlEvent.type == SDL_QUIT)
		{
			bRet = true;
		}
		else if (sdlEvent.type == SDL_KEYDOWN)
		{
			if (sdlEvent.key.keysym.sym == SDLK_ESCAPE
				|| sdlEvent.key.keysym.sym == SDLK_q)
			{
				bRet = true;
			}
			if (sdlEvent.key.keysym.sym == SDLK_c)
			{
				self->m_bShowCubes = !self->m_bShowCubes;
			}
		}
	}

	// Process SteamVR events
	VREvent_t event;
	while (self->m_pHMD->PollNextEvent(&event, sizeof(event)))
	{
		CMainApplication_ProcessVREvent(self, &event);
	}

	// Process SteamVR controller state
	for (TrackedDeviceIndex_t unDevice = 0; unDevice < k_unMaxTrackedDeviceCount; unDevice++)
	{
		VRControllerState_t state;
		if (self->m_pHMD->GetControllerState(unDevice, &state, sizeof(state)))
		{
			self->m_rbShowTrackedDevice[unDevice] = state.ulButtonPressed == 0;
		}
	}

	return bRet;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_RunMainLoop(CMainApplication* self)
{
	bool bQuit = false;

	SDL_StartTextInput();
	SDL_ShowCursor(SDL_DISABLE);

	while (!bQuit)
	{
		bQuit = CMainApplication_HandleInput(self);

		CMainApplication_RenderFrame(self);
	}

	SDL_StopTextInput();
}


//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
void CMainApplication_ProcessVREvent(CMainApplication* self, const VREvent_t* event)
{
	switch (event->eventType)
	{
	case EVREventType_VREvent_TrackedDeviceActivated:
	{
		CMainApplication_SetupRenderModelForTrackedDevice(self, event->trackedDeviceIndex);
		dprintf("Device %u attached. Setting up render model.\n", event->trackedDeviceIndex);
	}
	break;
	case EVREventType_VREvent_TrackedDeviceDeactivated:
	{
		dprintf("Device %u detached.\n", event->trackedDeviceIndex);
	}
	break;
	case EVREventType_VREvent_TrackedDeviceUpdated:
	{
		dprintf("Device %u updated.\n", event->trackedDeviceIndex);
	}
	break;
	}
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_RenderFrame(CMainApplication* self)
{
	// for now as fast as possible
	if (self->m_pHMD)
	{
		CMainApplication_RenderControllerAxes(self);
		CMainApplication_RenderStereoTargets(self);
		CMainApplication_RenderCompanionWindow(self);

		Texture_t leftEyeTexture = { (void*)self->m_leftEyeDesc.m_nResolveTextureId, ETextureType_TextureType_OpenGL, EColorSpace_ColorSpace_Gamma };
		self->m_pCompositor->Submit(EVREye_Eye_Left, &leftEyeTexture, NULL, EVRSubmitFlags_Submit_Default);
		Texture_t rightEyeTexture = { (void*)self->m_rightEyeDesc.m_nResolveTextureId, ETextureType_TextureType_OpenGL, EColorSpace_ColorSpace_Gamma };
		self->m_pCompositor->Submit(EVREye_Eye_Right, &rightEyeTexture, NULL, EVRSubmitFlags_Submit_Default);
	}

	if (self->m_bVblank && self->m_bGlFinishHack)
	{
		//$ HACKHACK. From gpuview profiling, it looks like there is a bug where two renders and a present
		// happen right before and after the vsync causing all kinds of jittering issues. This glFinish()
		// appears to clear that up. Temporary fix while I try to get nvidia to investigate this problem.
		// 1/29/2014 mikesart
		glFinish();
	}

	// SwapWindow
	{
		SDL_GL_SwapWindow(self->m_pCompanionWindow);
	}

	// Clear
	{
		// We want to make sure the glFinish waits for the entire present to complete, not just the submission
		// of the command. So, we do a clear here right here so the glFinish will wait fully for the swap.
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	// Flush and wait for swap.
	if (self->m_bVblank)
	{
		glFlush();
		glFinish();
	}

	// Spew out the controller and pose count whenever they change.
	if (self->m_iTrackedControllerCount != self->m_iTrackedControllerCount_Last || self->m_iValidPoseCount != self->m_iValidPoseCount_Last)
	{
		self->m_iValidPoseCount_Last = self->m_iValidPoseCount;
		self->m_iTrackedControllerCount_Last = self->m_iTrackedControllerCount;

		dprintf("PoseCount:%d(%s) Controllers:%d\n", self->m_iValidPoseCount, self->m_strPoseClasses, self->m_iTrackedControllerCount);
	}

	CMainApplication_UpdateHMDMatrixPose(self);
}


//-----------------------------------------------------------------------------
// Purpose: Compiles a GL shader program and returns the handle. Returns 0 if
//			the shader couldn't be compiled for some reason.
//-----------------------------------------------------------------------------
GLuint CMainApplication_CompileGLShader(CMainApplication* self, const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader)
{
	GLuint unProgramID = glCreateProgram();

	GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(nSceneVertexShader, 1, &pchVertexShader, NULL);
	glCompileShader(nSceneVertexShader);

	GLint vShaderCompiled = GL_FALSE;
	glGetShaderiv(nSceneVertexShader, GL_COMPILE_STATUS, &vShaderCompiled);
	if (vShaderCompiled != GL_TRUE)
	{
		dprintf("%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneVertexShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneVertexShader);
	glDeleteShader(nSceneVertexShader); // the program hangs onto this once it's attached

	GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(nSceneFragmentShader, 1, &pchFragmentShader, NULL);
	glCompileShader(nSceneFragmentShader);

	GLint fShaderCompiled = GL_FALSE;
	glGetShaderiv(nSceneFragmentShader, GL_COMPILE_STATUS, &fShaderCompiled);
	if (fShaderCompiled != GL_TRUE)
	{
		dprintf("%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader);
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneFragmentShader);
		return 0;
	}

	glAttachShader(unProgramID, nSceneFragmentShader);
	glDeleteShader(nSceneFragmentShader); // the program hangs onto this once it's attached

	glLinkProgram(unProgramID);

	GLint programSuccess = GL_TRUE;
	glGetProgramiv(unProgramID, GL_LINK_STATUS, &programSuccess);
	if (programSuccess != GL_TRUE)
	{
		dprintf("%s - Error linking program %d!\n", pchShaderName, unProgramID);
		glDeleteProgram(unProgramID);
		return 0;
	}

	glUseProgram(unProgramID);
	glUseProgram(0);

	return unProgramID;
}


//-----------------------------------------------------------------------------
// Purpose: Creates all the shaders used by HelloVR SDL
//-----------------------------------------------------------------------------
bool CMainApplication_CreateAllShaders(CMainApplication* self)
{
	self->m_unSceneProgramID = CMainApplication_CompileGLShader(self,
		"Scene",

		// Vertex Shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec2 v2UVcoordsIn;\n"
		"layout(location = 2) in vec3 v3NormalIn;\n"
		"out vec2 v2UVcoords;\n"
		"void main()\n"
		"{\n"
		"	v2UVcoords = v2UVcoordsIn;\n"
		"	gl_Position = matrix * position;\n"
		"}\n",

		// Fragment Shader
		"#version 410 core\n"
		"uniform sampler2D mytexture;\n"
		"in vec2 v2UVcoords;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = texture(mytexture, v2UVcoords);\n"
		"}\n"
		);
	self->m_nSceneMatrixLocation = glGetUniformLocation(self->m_unSceneProgramID, "matrix");
	if (self->m_nSceneMatrixLocation == -1)
	{
		dprintf("Unable to find matrix uniform in scene shader\n");
		return false;
	}

	self->m_unControllerTransformProgramID = CMainApplication_CompileGLShader(self,
		"Controller",

		// vertex shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec3 v3ColorIn;\n"
		"out vec4 v4Color;\n"
		"void main()\n"
		"{\n"
		"	v4Color.xyz = v3ColorIn; v4Color.a = 1.0;\n"
		"	gl_Position = matrix * position;\n"
		"}\n",

		// fragment shader
		"#version 410\n"
		"in vec4 v4Color;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = v4Color;\n"
		"}\n"
		);
	self->m_nControllerMatrixLocation = glGetUniformLocation(self->m_unControllerTransformProgramID, "matrix");
	if (self->m_nControllerMatrixLocation == -1)
	{
		dprintf("Unable to find matrix uniform in controller shader\n");
		return false;
	}

	self->m_unRenderModelProgramID = CMainApplication_CompileGLShader(self,
		"render model",

		// vertex shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec3 v3NormalIn;\n"
		"layout(location = 2) in vec2 v2TexCoordsIn;\n"
		"out vec2 v2TexCoord;\n"
		"void main()\n"
		"{\n"
		"	v2TexCoord = v2TexCoordsIn;\n"
		"	gl_Position = matrix * vec4(position.xyz, 1);\n"
		"}\n",

		//fragment shader
		"#version 410 core\n"
		"uniform sampler2D diffuse;\n"
		"in vec2 v2TexCoord;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = texture( diffuse, v2TexCoord);\n"
		"}\n"

		);
	self->m_nRenderModelMatrixLocation = glGetUniformLocation(self->m_unRenderModelProgramID, "matrix");
	if (self->m_nRenderModelMatrixLocation == -1)
	{
		dprintf("Unable to find matrix uniform in render model shader\n");
		return false;
	}

	self->m_unCompanionWindowProgramID = CMainApplication_CompileGLShader(self,
		"CompanionWindow",

		// vertex shader
		"#version 410 core\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec2 v2UVIn;\n"
		"noperspective out vec2 v2UV;\n"
		"void main()\n"
		"{\n"
		"	v2UV = v2UVIn;\n"
		"	gl_Position = position;\n"
		"}\n",

		// fragment shader
		"#version 410 core\n"
		"uniform sampler2D mytexture;\n"
		"noperspective in vec2 v2UV;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"		outputColor = texture(mytexture, v2UV);\n"
		"}\n"
		);


	return self->m_unSceneProgramID != 0
		&& self->m_unControllerTransformProgramID != 0
		&& self->m_unRenderModelProgramID != 0
		&& self->m_unCompanionWindowProgramID != 0;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
static bool CMainApplication_SetupTexturemaps(CMainApplication* self)
{
	char exePath[1024];
	Path_GetExecutablePath(exePath, sizeof(exePath));
	char sExecutableDirectory[1024];
	Path_StripFilename(exePath, sExecutableDirectory, sizeof(sExecutableDirectory));
	char strFullPath[1024];
	Path_MakeAbsolute("../cube_texture.png", sExecutableDirectory, strFullPath, sizeof(strFullPath));

	unsigned char* imageRGBA;
	unsigned nImageWidth, nImageHeight;

	unsigned nError = lodepng_decode_file(&imageRGBA, &nImageWidth, &nImageHeight, strFullPath, LCT_RGBA, 8);

	if (nError != 0)
		return false;

	glGenTextures(1, &self->m_iTexture);
	glBindTexture(GL_TEXTURE_2D, self->m_iTexture);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nImageWidth, nImageHeight,
		0, GL_RGBA, GL_UNSIGNED_BYTE, &imageRGBA[0]);

	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	GLfloat fLargest;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

	glBindTexture(GL_TEXTURE_2D, 0);

	return (self->m_iTexture != 0);
}


//-----------------------------------------------------------------------------
// Purpose: create a sea of cubes
//-----------------------------------------------------------------------------
void CMainApplication_SetupScene(CMainApplication* self)
{
	if (!self->m_pHMD)
		return;

	float* vertdata = NULL;
	int vertdatasize = 0;

	Matrix4 matScale = Matrix4_identity();
	Matrix4_scale_xyz(&matScale, self->m_fScale, self->m_fScale, self->m_fScale);
	Matrix4 matTransform = Matrix4_identity();
	Matrix4_translate_xyz(&matTransform,
		-((float)self->m_iSceneVolumeWidth * self->m_fScaleSpacing) / 2.f,
		-((float)self->m_iSceneVolumeHeight * self->m_fScaleSpacing) / 2.f,
		-((float)self->m_iSceneVolumeDepth * self->m_fScaleSpacing) / 2.f);

	Matrix4 mat = Matrix4_multiply_Matrix4(&matScale, &matTransform);

	for (int z = 0; z < self->m_iSceneVolumeDepth; z++)
	{
		for (int y = 0; y < self->m_iSceneVolumeHeight; y++)
		{
			for (int x = 0; x < self->m_iSceneVolumeWidth; x++)
			{
				CMainApplication_AddCubeToScene(self, &mat, &vertdata, &vertdatasize);
				Matrix4 scsp = Matrix4_identity();
				Matrix4_translate_xyz(&scsp, self->m_fScaleSpacing, 0, 0);
				mat = Matrix4_multiply_Matrix4(&mat, &scsp);
			}
			Matrix4 scsp = Matrix4_identity();
			Matrix4_translate_xyz(&scsp, -((float)self->m_iSceneVolumeWidth) * self->m_fScaleSpacing, self->m_fScaleSpacing, 0);
			mat = Matrix4_multiply_Matrix4(&mat, &scsp);
		}
		Matrix4 scsp = Matrix4_identity();
		Matrix4_translate_xyz(&scsp, 0, -((float)self->m_iSceneVolumeHeight) * self->m_fScaleSpacing, self->m_fScaleSpacing);
		mat = Matrix4_multiply_Matrix4(&mat, &scsp);
	}
	self->m_uiVertcount = vertdatasize / 5;

	glGenVertexArrays(1, &self->m_unSceneVAO);
	glBindVertexArray(self->m_unSceneVAO);

	glGenBuffers(1, &self->m_glSceneVertBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, self->m_glSceneVertBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdatasize, vertdata, GL_STATIC_DRAW);
	free(vertdata);

	GLsizei stride = sizeof(VertexDataScene);
	uintptr_t offset = 0;

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

	offset += sizeof(Vector3);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

	glBindVertexArray(0);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_AddCubeToScene(CMainApplication* self, const Matrix4 * const mat, float** vertdata, int* vertdatasize)
{
	// Matrix4 mat( outermat.data() );

	Vector4 va = { 0, 0, 0, 1 };
	Vector4 A = Matrix4_multiply_Vector4(mat, &va);
	Vector4 vb = { 1, 0, 0, 1 };
	Vector4 B = Matrix4_multiply_Vector4(mat, &vb);
	Vector4 vc = { 1, 1, 0, 1 };
	Vector4 C = Matrix4_multiply_Vector4(mat, &vc);
	Vector4 vd = { 0, 1, 0, 1 };
	Vector4 D = Matrix4_multiply_Vector4(mat, &vd);
	Vector4 ve = { 0, 0, 1, 1 };
	Vector4 E = Matrix4_multiply_Vector4(mat, &ve);
	Vector4 vf = { 1, 0, 1, 1 };
	Vector4 F = Matrix4_multiply_Vector4(mat, &vf);
	Vector4 vg = { 1, 1, 1, 1 };
	Vector4 G = Matrix4_multiply_Vector4(mat, &vg);
	Vector4 vh = { 0, 1, 1, 1 };
	Vector4 H = Matrix4_multiply_Vector4(mat, &vh);

	// triangles instead of quads
	addToArrayFloat5(vertdata, vertdatasize, E.x, E.y, E.z, 0, 1); //Front
	addToArrayFloat5(vertdata, vertdatasize, F.x, F.y, F.z, 1, 1);
	addToArrayFloat5(vertdata, vertdatasize, G.x, G.y, G.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, G.x, G.y, G.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, H.x, H.y, H.z, 0, 0);
	addToArrayFloat5(vertdata, vertdatasize, E.x, E.y, E.z, 0, 1);

	addToArrayFloat5(vertdata, vertdatasize, B.x, B.y, B.z, 0, 1); //Back
	addToArrayFloat5(vertdata, vertdatasize, A.x, A.y, A.z, 1, 1);
	addToArrayFloat5(vertdata, vertdatasize, D.x, D.y, D.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, D.x, D.y, D.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, C.x, C.y, C.z, 0, 0);
	addToArrayFloat5(vertdata, vertdatasize, B.x, B.y, B.z, 0, 1);

	addToArrayFloat5(vertdata, vertdatasize, H.x, H.y, H.z, 0, 1); //Top
	addToArrayFloat5(vertdata, vertdatasize, G.x, G.y, G.z, 1, 1);
	addToArrayFloat5(vertdata, vertdatasize, C.x, C.y, C.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, C.x, C.y, C.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, D.x, D.y, D.z, 0, 0);
	addToArrayFloat5(vertdata, vertdatasize, H.x, H.y, H.z, 0, 1);

	addToArrayFloat5(vertdata, vertdatasize, A.x, A.y, A.z, 0, 1); //Bottom
	addToArrayFloat5(vertdata, vertdatasize, B.x, B.y, B.z, 1, 1);
	addToArrayFloat5(vertdata, vertdatasize, F.x, F.y, F.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, F.x, F.y, F.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, E.x, E.y, E.z, 0, 0);
	addToArrayFloat5(vertdata, vertdatasize, A.x, A.y, A.z, 0, 1);

	addToArrayFloat5(vertdata, vertdatasize, A.x, A.y, A.z, 0, 1); //Left
	addToArrayFloat5(vertdata, vertdatasize, E.x, E.y, E.z, 1, 1);
	addToArrayFloat5(vertdata, vertdatasize, H.x, H.y, H.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, H.x, H.y, H.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, D.x, D.y, D.z, 0, 0);
	addToArrayFloat5(vertdata, vertdatasize, A.x, A.y, A.z, 0, 1);

	addToArrayFloat5(vertdata, vertdatasize, F.x, F.y, F.z, 0, 1); //Right
	addToArrayFloat5(vertdata, vertdatasize, B.x, B.y, B.z, 1, 1);
	addToArrayFloat5(vertdata, vertdatasize, C.x, C.y, C.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, C.x, C.y, C.z, 1, 0);
	addToArrayFloat5(vertdata, vertdatasize, G.x, G.y, G.z, 0, 0);
	addToArrayFloat5(vertdata, vertdatasize, F.x, F.y, F.z, 0, 1);
}


//-----------------------------------------------------------------------------
// Purpose: Draw all of the controllers as X/Y/Z lines
//-----------------------------------------------------------------------------
void CMainApplication_RenderControllerAxes(CMainApplication* self)
{
	// don't draw controllers if somebody else has input focus
	if (self->m_pHMD->IsInputFocusCapturedByAnotherProcess())
		return;

	float* vertdata = NULL;
	int vertdatasize = 0;

	self->m_uiControllerVertcount = 0;
	self->m_iTrackedControllerCount = 0;

	for (TrackedDeviceIndex_t unTrackedDevice = k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < k_unMaxTrackedDeviceCount; ++unTrackedDevice)
	{
		if (!self->m_pHMD->IsTrackedDeviceConnected(unTrackedDevice))
			continue;

		if (self->m_pHMD->GetTrackedDeviceClass(unTrackedDevice) != ETrackedDeviceClass_TrackedDeviceClass_Controller)
			continue;

		self->m_iTrackedControllerCount += 1;

		if (!self->m_rTrackedDevicePose[unTrackedDevice].bPoseIsValid)
			continue;

		const Matrix4* const mat = &self->m_rmat4DevicePose[unTrackedDevice];

		Vector4 vc = { 0, 0, 0, 1 };
		Vector4 center = Matrix4_multiply_Vector4(mat, &vc);

		for (int i = 0; i < 3; ++i)
		{
			Vector3 color = { 0, 0, 0 };
			Vector4 point = { 0, 0, 0, 1 };
			(&point.x)[i] += 0.05f;  // offset in X, Y, Z
			(&color.x)[i] = 1.0;  // R, G, B
			point = Matrix4_multiply_Vector4(mat, &point);
			addToArrayFloat3(&vertdata, &vertdatasize, center.x, center.y, center.z);

			addToArrayFloat3(&vertdata, &vertdatasize, color.x, color.y, color.z);

			addToArrayFloat3(&vertdata, &vertdatasize, point.x, point.y, point.z);

			addToArrayFloat3(&vertdata, &vertdatasize, color.x, color.y, color.z);

			self->m_uiControllerVertcount += 2;
		}

		Vector4 vs = { 0, 0, -0.02f, 1 };
		Vector4 start = Matrix4_multiply_Vector4(mat, &vs);
		Vector4 ve = { 0, 0, -39.f, 1 };
		Vector4 end = Matrix4_multiply_Vector4(mat, &ve);
		Vector3 color = { .92f, .92f, .71f };

		addToArrayFloat3(&vertdata, &vertdatasize, start.x, start.y, start.z);
		addToArrayFloat3(&vertdata, &vertdatasize, color.x, color.y, color.z);

		addToArrayFloat3(&vertdata, &vertdatasize, end.x, end.y, end.z);
		addToArrayFloat3(&vertdata, &vertdatasize, color.x, color.y, color.z);
		self->m_uiControllerVertcount += 2;
	}

	// Setup the VAO the first time through.
	if (self->m_unControllerVAO == 0)
	{
		glGenVertexArrays(1, &self->m_unControllerVAO);
		glBindVertexArray(self->m_unControllerVAO);

		glGenBuffers(1, &self->m_glControllerVertBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, self->m_glControllerVertBuffer);

		GLuint stride = 2 * 3 * sizeof(float);
		GLuint offset = 0;

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

		offset += sizeof(Vector3);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

		glBindVertexArray(0);
	}

	glBindBuffer(GL_ARRAY_BUFFER, self->m_glControllerVertBuffer);

	// set vertex data if we have some
	if (vertdatasize > 0)
	{
		//$ TODO: Use glBufferSubData for this...
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdatasize, &vertdata[0], GL_STREAM_DRAW);

		free(vertdata);
	}
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_SetupCameras(CMainApplication* self)
{
	self->m_mat4ProjectionLeft = CMainApplication_GetHMDMatrixProjectionEye(self, EVREye_Eye_Left);
	self->m_mat4ProjectionRight = CMainApplication_GetHMDMatrixProjectionEye(self, EVREye_Eye_Right);
	self->m_mat4eyePosLeft = CMainApplication_GetHMDMatrixPoseEye(self, EVREye_Eye_Left);
	self->m_mat4eyePosRight = CMainApplication_GetHMDMatrixPoseEye(self, EVREye_Eye_Right);
}


//-----------------------------------------------------------------------------
// Purpose: Creates a frame buffer. Returns true if the buffer was set up.
//          Returns false if the setup failed.
//-----------------------------------------------------------------------------
bool CMainApplication_CreateFrameBuffer(CMainApplication* self, int nWidth, int nHeight, FramebufferDesc* framebufferDesc)
{
	glGenFramebuffers(1, &framebufferDesc->m_nRenderFramebufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc->m_nRenderFramebufferId);

	glGenRenderbuffers(1, &framebufferDesc->m_nDepthBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc->m_nDepthBufferId);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, nWidth, nHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, framebufferDesc->m_nDepthBufferId);

	glGenTextures(1, &framebufferDesc->m_nRenderTextureId);
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc->m_nRenderTextureId);
	glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, nWidth, nHeight, true);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc->m_nRenderTextureId, 0);

	glGenFramebuffers(1, &framebufferDesc->m_nResolveFramebufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc->m_nResolveFramebufferId);

	glGenTextures(1, &framebufferDesc->m_nResolveTextureId);
	glBindTexture(GL_TEXTURE_2D, framebufferDesc->m_nResolveTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc->m_nResolveTextureId, 0);

	// check FBO status
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		return false;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication_SetupStereoRenderTargets(CMainApplication* self)
{
	if (!self->m_pHMD)
		return false;

	self->m_pHMD->GetRecommendedRenderTargetSize(&self->m_nRenderWidth, &self->m_nRenderHeight);

	CMainApplication_CreateFrameBuffer(self, self->m_nRenderWidth, self->m_nRenderHeight, &self->m_leftEyeDesc);
	CMainApplication_CreateFrameBuffer(self, self->m_nRenderWidth, self->m_nRenderHeight, &self->m_rightEyeDesc);

	return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_SetupCompanionWindow(CMainApplication* self)
{
	if (!self->m_pHMD)
		return;

	VertexDataWindow vVerts[8] = {
		// left eye verts
		{ { -1, -1 }, { 0, 1 } },
		{ { 0, -1 }, { 1 , 1 } },
		{ { -1, 1 }, { 0, 0 } },
		{ { 0, 1 }, { 1, 0 } },

		// right eye verts
		{ { 0, -1 }, { 0, 1 } },
		{ { 1, -1 }, { 1, 1 } },
		{ { 0, 1 }, { 0, 0 } },
		{ { 1, 1 }, { 1, 0 } },
	};

	GLushort vIndices[] = { 0, 1, 3,   0, 3, 2,   4, 5, 7,   4, 7, 6 };
	self->m_uiCompanionWindowIndexSize = _countof(vIndices);

	glGenVertexArrays(1, &self->m_unCompanionWindowVAO);
	glBindVertexArray(self->m_unCompanionWindowVAO);

	glGenBuffers(1, &self->m_glCompanionWindowIDVertBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, self->m_glCompanionWindowIDVertBuffer);
	glBufferData(GL_ARRAY_BUFFER, _countof(vVerts)*sizeof(VertexDataWindow), &vVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &self->m_glCompanionWindowIDIndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_glCompanionWindowIDIndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, self->m_uiCompanionWindowIndexSize*sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, position));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, texCoord));

	glBindVertexArray(0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_RenderStereoTargets(CMainApplication* self)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_MULTISAMPLE);

	// Left Eye
	glBindFramebuffer(GL_FRAMEBUFFER, self->m_leftEyeDesc.m_nRenderFramebufferId);
	glViewport(0, 0, self->m_nRenderWidth, self->m_nRenderHeight);
	CMainApplication_RenderScene(self, EVREye_Eye_Left);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glDisable(GL_MULTISAMPLE);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, self->m_leftEyeDesc.m_nRenderFramebufferId);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, self->m_leftEyeDesc.m_nResolveFramebufferId);

	glBlitFramebuffer(0, 0, self->m_nRenderWidth, self->m_nRenderHeight, 0, 0, self->m_nRenderWidth, self->m_nRenderHeight,
		GL_COLOR_BUFFER_BIT,
		GL_LINEAR);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

	glEnable(GL_MULTISAMPLE);

	// Right Eye
	glBindFramebuffer(GL_FRAMEBUFFER, self->m_rightEyeDesc.m_nRenderFramebufferId);
	glViewport(0, 0, self->m_nRenderWidth, self->m_nRenderHeight);
	CMainApplication_RenderScene(self, EVREye_Eye_Right);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glDisable(GL_MULTISAMPLE);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, self->m_rightEyeDesc.m_nRenderFramebufferId);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, self->m_rightEyeDesc.m_nResolveFramebufferId);

	glBlitFramebuffer(0, 0, self->m_nRenderWidth, self->m_nRenderHeight, 0, 0, self->m_nRenderWidth, self->m_nRenderHeight,
		GL_COLOR_BUFFER_BIT,
		GL_LINEAR);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}


//-----------------------------------------------------------------------------
// Purpose: Renders a scene with respect to nEye.
//-----------------------------------------------------------------------------
void CMainApplication_RenderScene(CMainApplication* self, Hmd_Eye nEye)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	if (self->m_bShowCubes)
	{
		glUseProgram(self->m_unSceneProgramID);
		glUniformMatrix4fv(self->m_nSceneMatrixLocation, 1, GL_FALSE, CMainApplication_GetCurrentViewProjectionMatrix(self, nEye).m);
		glBindVertexArray(self->m_unSceneVAO);
		glBindTexture(GL_TEXTURE_2D, self->m_iTexture);
		glDrawArrays(GL_TRIANGLES, 0, self->m_uiVertcount);
		glBindVertexArray(0);
	}

	bool bIsInputCapturedByAnotherProcess = self->m_pHMD->IsInputFocusCapturedByAnotherProcess();

	if (!bIsInputCapturedByAnotherProcess)
	{
		// draw the controller axis lines
		glUseProgram(self->m_unControllerTransformProgramID);
		glUniformMatrix4fv(self->m_nControllerMatrixLocation, 1, GL_FALSE, CMainApplication_GetCurrentViewProjectionMatrix(self, nEye).m);
		glBindVertexArray(self->m_unControllerVAO);
		glDrawArrays(GL_LINES, 0, self->m_uiControllerVertcount);
		glBindVertexArray(0);
	}

	// ----- Render Model rendering -----
	glUseProgram(self->m_unRenderModelProgramID);

	for (uint32_t unTrackedDevice = 0; unTrackedDevice < k_unMaxTrackedDeviceCount; unTrackedDevice++)
	{
		if (!self->m_rTrackedDeviceToRenderModel[unTrackedDevice] || !self->m_rbShowTrackedDevice[unTrackedDevice])
			continue;

		const TrackedDevicePose_t* pose = &self->m_rTrackedDevicePose[unTrackedDevice];
		if (!pose->bPoseIsValid)
			continue;

		if (bIsInputCapturedByAnotherProcess && self->m_pHMD->GetTrackedDeviceClass(unTrackedDevice) == ETrackedDeviceClass_TrackedDeviceClass_Controller)
			continue;

		const Matrix4* matDeviceToTracking = &self->m_rmat4DevicePose[unTrackedDevice];
		Matrix4 proj = CMainApplication_GetCurrentViewProjectionMatrix(self, nEye);
		Matrix4 matMVP = Matrix4_multiply_Matrix4(&proj, matDeviceToTracking);
		glUniformMatrix4fv(self->m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.m);

		CGLRenderModel_Draw(self->m_rTrackedDeviceToRenderModel[unTrackedDevice]);
	}

	glUseProgram(0);
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_RenderCompanionWindow(CMainApplication* self)
{
	glDisable(GL_DEPTH_TEST);
	glViewport(0, 0, self->m_nCompanionWindowWidth, self->m_nCompanionWindowHeight);

	glBindVertexArray(self->m_unCompanionWindowVAO);
	glUseProgram(self->m_unCompanionWindowProgramID);

	// render left eye (first half of index array )
	glBindTexture(GL_TEXTURE_2D, self->m_leftEyeDesc.m_nResolveTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glDrawElements(GL_TRIANGLES, self->m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, 0);

	// render right eye (second half of index array )
	glBindTexture(GL_TEXTURE_2D, self->m_rightEyeDesc.m_nResolveTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glDrawElements(GL_TRIANGLES, self->m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, (const void *)(self->m_uiCompanionWindowIndexSize));

	glBindVertexArray(0);
	glUseProgram(0);
}


//-----------------------------------------------------------------------------
// Purpose: Gets a Matrix Projection Eye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication_GetHMDMatrixProjectionEye(CMainApplication* self, Hmd_Eye nEye)
{
	if (!self->m_pHMD)
		return Matrix4_identity();

	HmdMatrix44_t mat = self->m_pHMD->GetProjectionMatrix(nEye, self->m_fNearClip, self->m_fFarClip);

	Matrix4 mx = {
		.m = {
			mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
			mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
			mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
			mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
		}
	};
	return mx;
}


//-----------------------------------------------------------------------------
// Purpose: Gets an HMDMatrixPoseEye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication_GetHMDMatrixPoseEye(CMainApplication* self, Hmd_Eye nEye)
{
	if (!self->m_pHMD)
		return Matrix4_identity();

	HmdMatrix34_t matEyeRight = self->m_pHMD->GetEyeToHeadTransform(nEye);
	Matrix4 mx = {
		.m = {
			matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0,
			matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
			matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
			matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f
		}
	};

	Matrix4_invert(&mx);
	return mx;
}


//-----------------------------------------------------------------------------
// Purpose: Gets a Current View Projection Matrix with respect to nEye,
//          which may be an Eye_Left or an Eye_Right.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication_GetCurrentViewProjectionMatrix(CMainApplication* self, Hmd_Eye nEye)
{
	Matrix4 matMVP, matProjEyePos;
	if (nEye == EVREye_Eye_Left)
	{
		matProjEyePos = Matrix4_multiply_Matrix4(&self->m_mat4ProjectionLeft, &self->m_mat4eyePosLeft);
	}
	else if (nEye == EVREye_Eye_Right)
	{
		matProjEyePos = Matrix4_multiply_Matrix4(&self->m_mat4ProjectionRight, &self->m_mat4eyePosRight);
	}
	matMVP = Matrix4_multiply_Matrix4(&matProjEyePos, &self->m_mat4HMDPose);

	return matMVP;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication_UpdateHMDMatrixPose(CMainApplication* self)
{
	if (!self->m_pHMD)
		return;

	self->m_pCompositor->WaitGetPoses(self->m_rTrackedDevicePose, k_unMaxTrackedDeviceCount, NULL, 0);

	self->m_iValidPoseCount = 0;
	self->m_strPoseClasses = "";
	for (unsigned int nDevice = 0; nDevice < k_unMaxTrackedDeviceCount; ++nDevice)
	{
		if (self->m_rTrackedDevicePose[nDevice].bPoseIsValid)
		{
			self->m_iValidPoseCount++;
			self->m_rmat4DevicePose[nDevice] = CMainApplication_ConvertSteamVRMatrixToMatrix4(self, &self->m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
			if (self->m_rDevClassChar[nDevice] == 0)
			{
				switch (self->m_pHMD->GetTrackedDeviceClass(nDevice))
				{
				case ETrackedDeviceClass_TrackedDeviceClass_Controller:        self->m_rDevClassChar[nDevice] = 'C'; break;
				case ETrackedDeviceClass_TrackedDeviceClass_HMD:               self->m_rDevClassChar[nDevice] = 'H'; break;
				case ETrackedDeviceClass_TrackedDeviceClass_Invalid:           self->m_rDevClassChar[nDevice] = 'I'; break;
				case ETrackedDeviceClass_TrackedDeviceClass_GenericTracker:    self->m_rDevClassChar[nDevice] = 'G'; break;
				case ETrackedDeviceClass_TrackedDeviceClass_TrackingReference: self->m_rDevClassChar[nDevice] = 'T'; break;
				default:                                                       self->m_rDevClassChar[nDevice] = '?'; break;
				}
			}
			self->m_strPoseClasses += self->m_rDevClassChar[nDevice];
		}
	}

	if (self->m_rTrackedDevicePose[k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
	{
		Matrix4_invert(&self->m_rmat4DevicePose[k_unTrackedDeviceIndex_Hmd]);
		self->m_mat4HMDPose = self->m_rmat4DevicePose[k_unTrackedDeviceIndex_Hmd];
	}
}


//-----------------------------------------------------------------------------
// Purpose: Finds a render model we've already loaded or loads a new one.
// Allocates new CGLRenderModel instance.
//-----------------------------------------------------------------------------
CGLRenderModel* CMainApplication_FindOrLoadRenderModel(CMainApplication* self, char *pchRenderModelName)
{
	CGLRenderModel *pRenderModel = NULL;
	for (unsigned int i = 0; i < k_unMaxTrackedDeviceCount; ++i)
	{
		CGLRenderModel* model = self->m_vecRenderModels[i];
		if (!model)
			break;
		if (!stricmp(model->m_sModelName, pchRenderModelName))
		{
			pRenderModel = model;
			break;
		}
	}

	// load the model if we didn't find one
	if (!pRenderModel)
	{
		RenderModel_t *pModel;
		EVRRenderModelError error;
		while (1)
		{
			error = self->m_pRenderModels->LoadRenderModel_Async(pchRenderModelName, &pModel);
			if (error != EVRRenderModelError_VRRenderModelError_Loading)
				break;

			ThreadSleep(1);
		}

		if (error != EVRRenderModelError_VRRenderModelError_None)
		{
			dprintf("Unable to load render model %s - %s\n", pchRenderModelName, self->m_pRenderModels->GetRenderModelErrorNameFromEnum(error));
			return NULL; // move on to the next tracked device
		}

		RenderModel_TextureMap_t *pTexture;
		while (1)
		{
			error = self->m_pRenderModels->LoadTexture_Async(pModel->diffuseTextureId, &pTexture);
			if (error != EVRRenderModelError_VRRenderModelError_Loading)
				break;

			ThreadSleep(1);
		}

		if (error != EVRRenderModelError_VRRenderModelError_None)
		{
			dprintf("Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, pchRenderModelName);
			self->m_pRenderModels->FreeRenderModel(pModel);
			return NULL; // move on to the next tracked device
		}

		pRenderModel = malloc(sizeof(CGLRenderModel));
		CGLRenderModel_init(pRenderModel, pchRenderModelName);
		if (!CGLRenderModel_BInit(pRenderModel, pModel, pTexture))
		{
			dprintf("Unable to create GL model from render model %s\n", pchRenderModelName);
			free(pRenderModel);
			pRenderModel = NULL;
		}
		else
		{
			for (unsigned int i = 0; i < k_unMaxTrackedDeviceCount; ++i) {
				if (!self->m_vecRenderModels[i])
				{
					self->m_vecRenderModels[i] = pRenderModel;
					break;
				}
				else if (i == k_unMaxTrackedDeviceCount - 1)
				{
					dprintf("No place in array to fit render model %s\n", pchRenderModelName);
					exit(1);
				}
			}
		}
		self->m_pRenderModels->FreeRenderModel(pModel);
		self->m_pRenderModels->FreeTexture(pTexture);
	}
	return pRenderModel;
}


//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL a Render Model for a single tracked device
//-----------------------------------------------------------------------------
void CMainApplication_SetupRenderModelForTrackedDevice(CMainApplication* self, TrackedDeviceIndex_t unTrackedDeviceIndex)
{
	if (unTrackedDeviceIndex >= k_unMaxTrackedDeviceCount)
		return;

	TrackedPropertyError peError;
	// try to find a model we've already set up
	char* sRenderModelName = GetTrackedDeviceString(self->m_pHMD, unTrackedDeviceIndex, ETrackedDeviceProperty_Prop_RenderModelName_String, &peError);
	CGLRenderModel *pRenderModel = CMainApplication_FindOrLoadRenderModel(self, sRenderModelName);
	if (!pRenderModel)
	{
		free(sRenderModelName);
		char* sTrackingSystemName = GetTrackedDeviceString(self->m_pHMD, unTrackedDeviceIndex, ETrackedDeviceProperty_Prop_TrackingSystemName_String, &peError);
		dprintf("Unable to load render model for tracked device %d (%s.%s)", unTrackedDeviceIndex, sTrackingSystemName, sRenderModelName);
	}
	else
	{
		// the allocated sRenderModelName lifetime is determinted by CGLRenderModel
		self->m_rTrackedDeviceToRenderModel[unTrackedDeviceIndex] = pRenderModel;
		self->m_rbShowTrackedDevice[unTrackedDeviceIndex] = true;
	}
}


//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL Render Models
//-----------------------------------------------------------------------------
void CMainApplication_SetupRenderModels(CMainApplication* self)
{
	memset(self->m_rTrackedDeviceToRenderModel, 0, sizeof(self->m_rTrackedDeviceToRenderModel));

	if (!self->m_pHMD)
		return;

	for (uint32_t unTrackedDevice = k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < k_unMaxTrackedDeviceCount; unTrackedDevice++)
	{
		if (!self->m_pHMD->IsTrackedDeviceConnected(unTrackedDevice))
			continue;

		CMainApplication_SetupRenderModelForTrackedDevice(self, unTrackedDevice);
	}
}


//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
Matrix4 CMainApplication_ConvertSteamVRMatrixToMatrix4(CMainApplication* self, const HmdMatrix34_t* matPose)
{
	Matrix4 mx = {
		{
			matPose->m[0][0], matPose->m[1][0], matPose->m[2][0], 0.0,
			matPose->m[0][1], matPose->m[1][1], matPose->m[2][1], 0.0,
			matPose->m[0][2], matPose->m[1][2], matPose->m[2][2], 0.0,
			matPose->m[0][3], matPose->m[1][3], matPose->m[2][3], 1.0f
		}
	};
	return mx;
}


//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL Render Models
//-----------------------------------------------------------------------------
void CGLRenderModel_init(CGLRenderModel* self, char* sRenderModelName)
{
	self->m_glIndexBuffer = 0;
	self->m_glVertArray = 0;
	self->m_glVertBuffer = 0;
	self->m_glTexture = 0;
	self->m_sModelName = sRenderModelName;
}


void CGLRenderModel_destroy(CGLRenderModel* self) // FIXME actually call
{
	free(self->m_sModelName);

	if (self->m_glVertBuffer)
	{
		glDeleteBuffers(1, &self->m_glIndexBuffer);
		glDeleteVertexArrays(1, &self->m_glVertArray);
		glDeleteBuffers(1, &self->m_glVertBuffer);
		self->m_glIndexBuffer = 0;
		self->m_glVertArray = 0;
		self->m_glVertBuffer = 0;
	}
}


//-----------------------------------------------------------------------------
// Purpose: Allocates and populates the GL resources for a render model
//-----------------------------------------------------------------------------
bool CGLRenderModel_BInit(CGLRenderModel* self, const RenderModel_t* vrModel, const RenderModel_TextureMap_t* vrDiffuseTexture)
{
	// create and bind a VAO to hold state for this model
	glGenVertexArrays(1, &self->m_glVertArray);
	glBindVertexArray(self->m_glVertArray);

	// Populate a vertex buffer
	glGenBuffers(1, &self->m_glVertBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, self->m_glVertBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(RenderModel_Vertex_t) * vrModel->unVertexCount, vrModel->rVertexData, GL_STATIC_DRAW);

	// Identify the components in the vertex buffer
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RenderModel_Vertex_t), (void *)offsetof(RenderModel_Vertex_t, vPosition));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(RenderModel_Vertex_t), (void *)offsetof(RenderModel_Vertex_t, vNormal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(RenderModel_Vertex_t), (void *)offsetof(RenderModel_Vertex_t, rfTextureCoord));

	// Create and populate the index buffer
	glGenBuffers(1, &self->m_glIndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_glIndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint16_t) * vrModel->unTriangleCount * 3, vrModel->rIndexData, GL_STATIC_DRAW);

	glBindVertexArray(0);

	// create and populate the texture
	glGenTextures(1, &self->m_glTexture);
	glBindTexture(GL_TEXTURE_2D, self->m_glTexture);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTexture->unWidth, vrDiffuseTexture->unHeight,
		0, GL_RGBA, GL_UNSIGNED_BYTE, vrDiffuseTexture->rubTextureMapData);

	// If this renders black ask McJohn what's wrong.
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	GLfloat fLargest;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

	glBindTexture(GL_TEXTURE_2D, 0);

	self->m_unVertexCount = vrModel->unTriangleCount * 3;

	return true;
}


//-----------------------------------------------------------------------------
// Purpose: Draws the render model
//-----------------------------------------------------------------------------
void CGLRenderModel_Draw(CGLRenderModel* self)
{
	glBindVertexArray(self->m_glVertArray);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, self->m_glTexture);

	glDrawElements(GL_TRIANGLES, self->m_unVertexCount, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	CMainApplication app;
	CMainApplication_init(&app, argc, argv);

	if (!CMainApplication_BInit(&app))
	{
		CMainApplication_Shutdown(&app);
		return 1;
	}

	CMainApplication_RunMainLoop(&app);

	CMainApplication_Shutdown(&app);

	return 0;
}
