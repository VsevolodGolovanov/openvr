#ifndef OPENVR_CAPI_FIXED_H
#define OPENVR_CAPI_FIXED_H
#if defined(_WIN32) || defined(__clang__)
#pragma once
#endif

#ifdef EXTERN_C
#undef EXTERN_C
#endif // EXTERN_C

#ifdef S_API
#undef S_API
#endif // S_API

#include <openvr_capi.h>

#undef S_API
#define S_API __declspec(dllimport) 

// expose the '#if 0'ed functions
S_API intptr_t VR_InitInternal(EVRInitError *peError, EVRApplicationType eType);
S_API void VR_ShutdownInternal();
S_API bool VR_IsHmdPresent();
S_API intptr_t VR_GetGenericInterface(const char *pchInterfaceVersion, EVRInitError *peError);
S_API bool VR_IsRuntimeInstalled();
S_API const char * VR_GetVRInitErrorAsSymbol(EVRInitError error);
S_API const char * VR_GetVRInitErrorAsEnglishDescription(EVRInitError error);

// not exposed in capi, but exposed in others:
S_API bool VR_IsInterfaceVersionValid(const char *pchInterfaceVersion);

// types for structs
typedef struct VR_IVRSystem_FnTable VR_IVRSystem;
typedef struct VR_IVRExtendedDisplay_FnTable VR_IVRExtendedDisplay;
typedef struct VR_IVRTrackedCamera_FnTable VR_IVRTrackedCamera;
typedef struct VR_IVRApplications_FnTable VR_IVRApplications;
typedef struct VR_IVRChaperone_FnTable VR_IVRChaperone;
typedef struct VR_IVRChaperoneSetup_FnTable VR_IVRChaperoneSetup;
typedef struct VR_IVRCompositor_FnTable VR_IVRCompositor;
typedef struct VR_IVROverlay_FnTable VR_IVROverlay;
typedef struct VR_IVRRenderModels_FnTable VR_IVRRenderModels;
typedef struct VR_IVRNotifications_FnTable VR_IVRNotifications;
typedef struct VR_IVRSettings_FnTable VR_IVRSettings;
typedef struct VR_IVRScreenshots_FnTable VR_IVRScreenshots;
typedef struct VR_IVRResources_FnTable VR_IVRResources;
typedef struct VREvent_t VREvent_t;

// VR_GetGenericInterface FnTable interface prefix
static const char* VR_IVRFnTable_Prefix = "FnTable:";

#endif // OPENVR_CAPI_FIXED_H