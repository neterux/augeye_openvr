//========= Copyright Valve Corporation ============//

#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#if defined( OSX )
#include <Foundation/Foundation.h>
#include <AppKit/AppKit.h>
#include <OpenGL/glu.h>
// Apple's version of glut.h #undef's APIENTRY, redefine it
#define APIENTRY
#else
#include <GL/glu.h>
#endif
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <vector>

#include <openvr.h>

#include "shared/Matrices.h"
#include "shared/pathtools.h"

#if defined(POSIX)
#include "unistd.h"
#endif

// Additonal codes
#include <opencv2/opencv.hpp>
#include "uEyeCamera.h"
#include "pupil.h"
#include "EyeTrack.h"

#ifndef _WIN32
#define APIENTRY
#endif

#ifndef _countof
#define _countof(x) (sizeof(x)/sizeof((x)[0]))
#endif

#define USE_EYETRACKER

#ifdef USE_EYETRACKER
// for threading
struct thread_aborted {};
std::atomic<bool> exit_flag;
void check_threadexit() {
    if (exit_flag)
        throw thread_aborted{};
}

void GetPupil(std::mutex& mtx, int nEye, cv::Point2f& pupilCenterPt)
{
    std::ostringstream msg;
    msg << "Waiting Pupil Service... (" << nEye << ")";
    std::cout << msg.str() << std::endl;

    Pupil eyecam;

    msg.str("");
    msg.clear(std::ostringstream::goodbit);
    msg << "Pupil Service connected (" << nEye << ")";
    std::cout << msg.str() << std::endl;

    try
    {
        while (true)
        {
            {
                //std::lock_guard<std::mutex> lock(mtx);
                eyecam.Get(&pupilCenterPt.x, &pupilCenterPt.y, nEye);
            }
            //EyeTrack::pupilPt pt;
            //eyecam.Get(&pt.x, &pt.y, nEye);
            //pupilCenterPt.store(pt);
            check_threadexit();
        }
    }
    catch (thread_aborted & e)
    {
        return;
    }
    return;
}

void GetPupil_exit() {
    exit_flag = true;
}
#endif // USE_EYETRACKER

void ThreadSleep( unsigned long nMilliseconds )
{
#if defined(_WIN32)
    ::Sleep( nMilliseconds );
#elif defined(POSIX)
    usleep( nMilliseconds * 1000 );
#endif
}

class CGLRenderModel
{
public:
    CGLRenderModel( const std::string & sRenderModelName );
    ~CGLRenderModel();

    bool BInit( const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture );
    void Cleanup();
    void Draw();
    const std::string & GetName() const { return m_sModelName; }

private:
    GLuint m_glVertBuffer;
    GLuint m_glIndexBuffer;
    GLuint m_glVertArray;
    GLuint m_glTexture;
    GLsizei m_unVertexCount;
    std::string m_sModelName;
};

static bool g_bPrintf = true;

//-----------------------------------------------------------------------------
// Purpose:
//------------------------------------------------------------------------------
class CMainApplication
{
public:
    CMainApplication( int argc, char *argv[] );
    virtual ~CMainApplication();

    bool BInit();
    bool BInitGL();
    bool BInitCompositor();

    void Shutdown();

    void RunMainLoop();
    bool HandleInput();
    void ProcessVREvent( const vr::VREvent_t & event );
    void RenderFrame();

    bool SetupTexturemaps();

    void SetupScene();
    void AddCubeToScene( Matrix4 mat, std::vector<float> &vertdata );
    void AddVertex( float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata );
    void AddPlaneToScene(Matrix4 mat, std::vector<float>& vertdata);  // Add
    void AddPlaneMeshToScene(std::vector<float>& vertdata);  // Add

    void RenderControllerAxes();

    bool SetupStereoRenderTargets();
    void SetupCompanionWindow();
    void SetupCameras();

    void RenderStereoTargets();
    void RenderCompanionWindow();
    void RenderScene( vr::Hmd_Eye nEye );
    void UpdateTexture( vr::Hmd_Eye nEye );  // Add

    Matrix4 GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye );
    Matrix3 IntrinsicsFromProjection(vr::Hmd_Eye nEye, bool ydown = true);
    Matrix4 GetHMDMatrixPoseEye( vr::Hmd_Eye nEye );
    Matrix4 GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );
    void UpdateHMDMatrixPose();

    Matrix4 ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose );

    GLuint CompileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader );
    bool CreateAllShaders();

    CGLRenderModel *FindOrLoadRenderModel( const char *pchRenderModelName );

private: 
    bool m_bDebugOpenGL;
    bool m_bVerbose;
    bool m_bPerf;
    bool m_bVblank;
    bool m_bGlFinishHack;

    vr::IVRSystem *m_pHMD;
    std::string m_strDriver;
    std::string m_strDisplay;
    vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
    Matrix4 m_rmat4DevicePose[ vr::k_unMaxTrackedDeviceCount ];
    
    struct ControllerInfo_t
    {
        vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
        vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
        vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
        Matrix4 m_rmat4Pose;
        CGLRenderModel *m_pRenderModel = nullptr;
        std::string m_sRenderModelName;
        bool m_bShowController = false;
    };

    enum EHand
    {
        Left = 0,
        Right = 1,
    };
    ControllerInfo_t m_rHand[2];

private: // SDL bookkeeping
    SDL_Window *m_pCompanionWindow;
    uint32_t m_nCompanionWindowWidth;
    uint32_t m_nCompanionWindowHeight;

    SDL_GLContext m_pContext;

private: // OpenGL bookkeeping
    int m_iTrackedControllerCount;
    int m_iTrackedControllerCount_Last;
    int m_iValidPoseCount;
    int m_iValidPoseCount_Last;
    bool m_bShowCubes;
    Vector2 m_vAnalogValue;

    std::string m_strPoseClasses;                            // what classes we saw poses for this frame
    char m_rDevClassChar[ vr::k_unMaxTrackedDeviceCount ];   // for each device, a character representing its class

    float m_fScale;
    
    int m_iSceneVolumeInit;                                  // if you want something other than the default 20x20x20
    
    float m_fNearClip;
    float m_fFarClip;

    GLuint m_iTexture[2];                           // Texture for m_vision image correspond to L/R eye
    unsigned int nImageWidth, nImageHeight;         // Add
    unsigned int nTextureWidth, nTextureHeight;     // Add
    inline unsigned texture_size(unsigned s) { return pow(2.0, unsigned(ceil(log(double(s)) / log(2.0)))); }

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
    Matrix4 m_mat4eyePose[2];

    Matrix4 m_mat4ProjectionCenter;
    Matrix4 m_mat4Projection[2];

    struct VertexDataScene
    {
        Vector3 position;
        Vector2 texCoord;
    };

    struct VertexDataWindow
    {
        Vector2 position;
        Vector2 texCoord;

        VertexDataWindow( const Vector2 & pos, const Vector2 tex ) :  position(pos), texCoord(tex) {    }
    };

    GLuint m_unSceneProgramID;
    GLuint m_unCompanionWindowProgramID;
    GLuint m_unControllerTransformProgramID;
    GLuint m_unRenderModelProgramID;

    GLint m_nSceneMatrixLocation;
    GLint m_nCameraTextureLocation;
    GLint m_nCalibBooleanLocation;
    GLint m_nControllerMatrixLocation;
    GLint m_nRenderModelMatrixLocation;

    struct FramebufferDesc
    {
        GLuint m_nDepthBufferId;
        GLuint m_nRenderTextureId;
        GLuint m_nRenderFramebufferId;
        GLuint m_nResolveTextureId;
        GLuint m_nResolveFramebufferId;
    };
    FramebufferDesc leftEyeDesc;
    FramebufferDesc rightEyeDesc;

    bool CreateFrameBuffer( int nWidth, int nHeight, FramebufferDesc &framebufferDesc );
    
    uint32_t m_nRenderWidth;
    uint32_t m_nRenderHeight;

    std::vector< CGLRenderModel * > m_vecRenderModels;

    vr::VRActionHandle_t m_actionHideCubes = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t m_actionHideThisController = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t m_actionTriggerHaptic = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t m_actionAnalongInput = vr::k_ulInvalidActionHandle;

    vr::VRActionSetHandle_t m_actionsetDemo = vr::k_ulInvalidActionSetHandle;

    /// for capturing vision image
    uEyeCamera m_vision[2];  // Handle uEyeCamera
    int m_eye;
    bool m_bChangeFocus = false;

#ifdef USE_EYETRACKER
    // for eye tracking
    EyeTrack tracker;
#endif // USE_EYETRACKER

    cv::Mat cam_mat = (cv::Mat_<float>(3, 3) << 1172.5f, 0.f, 1024.f, 0.f, 1172.5f, 1024.f, 0.f, 0.f, 1.f);
    cv::Mat dist_coeffs = (cv::Mat_<float>(8, 1) << -0.02612325714, -0.2002757143, 0.0000088126, 0.00001278283571,
            -0.006831787357, 0.3176116643, -0.28692775, -0.04330581357);

    //Matrix4 m_mat4StereoProjection[2] =
    //{
    //    Matrix4(
    //        1172.5f, 0.f, 1.02309077e+03f, 0.f,
    //        0.f, 1172.5f, 1.25093422e+03f, 0.f,
    //        0.f, 0.f, 1.f, 0.f,
    //        0.f, 0.f, 0.f, 1.f
    //    ),
    //    Matrix4(
    //        1172.5f, 0.f, 1.02309077e+03f, 0.f,
    //        0.f, 1172.5f, 1.25093422e+03f, -3.26792056e+03f,
    //        0.f, 0.f, 1.f, 0.f,
    //        0.f, 0.f, 0.f, 1.f
    //    )
    //};
    Matrix4 m_mat4StereoProjection[2] =
    {
        Matrix4(
            1172.5f, 0.f, 7.94994858e+02f, 0.f,
            0.f, 1172.5f, 1.02461522e+03f, 0.f,
            0.f, 0.f, 1.f, 0.f,
            0.f, 0.f, 0.f, 1.f
        ),
        Matrix4(
            1172.5f, 0.f, 7.94994858e+02f, -3.28434263e+03f,
            0.f, 1172.5f, 1.02461522e+03f, 0.f,
            0.f, 0.f, 1.f, 0.f,
            0.f, 0.f, 0.f, 1.f
        )
    };

    float stereo_cam_mat[2][9] =
    {
        { 1172.5f, 0.f, 1.02309077e+03f, 0.f, 1172.5f, 1.25093422e+03f, 0.f, 0.f, 1.f },
        { 1172.5f, 0.f, 1.02309077e+03f, 0.f, 1172.5f, 1.25093422e+03f, 0.f, 0.f, 1.f }
    };

    float stereo_rotate_mat[2][9] =
    {
        { 0.99911598f, 0.04200094f, -0.00178599f,
          0.04198325f, -0.99470763f, 0.09377711f,
          0.00216219f, -0.09376919f, -0.99559162f },
        { 0.99920284f, 0.03988358f, 0.00172761f,
          0.03990643f, -0.99907119f, -0.01625526f,
          0.00107769f, 0.01631125f, -0.99986638f }
    };
};


//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a rising edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionRisingEdge(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr )
{
    vr::InputDigitalActionData_t actionData;
    vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle );
    if (pDevicePath)
    {
        *pDevicePath = vr::k_ulInvalidInputValueHandle;
        if (actionData.bActive)
        {
            vr::InputOriginInfo_t originInfo;
            if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
            {
                *pDevicePath = originInfo.devicePath;
            }
        }
    }
    return actionData.bActive && actionData.bChanged && actionData.bState;
}


//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a falling edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionFallingEdge(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr )
{
    vr::InputDigitalActionData_t actionData;
    vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle );
    if (pDevicePath)
    {
        *pDevicePath = vr::k_ulInvalidInputValueHandle;
        if (actionData.bActive)
        {
            vr::InputOriginInfo_t originInfo;
            if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
            {
                *pDevicePath = originInfo.devicePath;
            }
        }
    }
    return actionData.bActive && actionData.bChanged && !actionData.bState;
}


//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and its state is true
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionState(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr )
{
    vr::InputDigitalActionData_t actionData;
    vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle );
    if (pDevicePath)
    {
        *pDevicePath = vr::k_ulInvalidInputValueHandle;
        if (actionData.bActive)
        {
            vr::InputOriginInfo_t originInfo;
            if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
            {
                *pDevicePath = originInfo.devicePath;
            }
        }
    }
    return actionData.bActive && actionData.bState;
}

//-----------------------------------------------------------------------------
// Purpose: Outputs a set of optional arguments to debugging output, using
//          the printf format setting specified in fmt*.
//-----------------------------------------------------------------------------
void dprintf( const char *fmt, ... )
{
    va_list args;
    char buffer[ 2048 ];

    va_start( args, fmt );
    vsprintf_s( buffer, fmt, args );
    va_end( args );

    if ( g_bPrintf )
        printf( "%s", buffer );

    OutputDebugStringA( buffer );
}

//-----------------------------------------------------------------------------
// Purpose: Constructor
//-----------------------------------------------------------------------------
CMainApplication::CMainApplication( int argc, char *argv[] )
    : m_pCompanionWindow(NULL)
    , m_pContext(NULL)
    , m_nCompanionWindowWidth( 640 )
    , m_nCompanionWindowHeight( 320 )
    , m_unSceneProgramID( 0 )
    , m_unCompanionWindowProgramID( 0 )
    , m_unControllerTransformProgramID( 0 )
    , m_unRenderModelProgramID( 0 )
    , m_pHMD( NULL )
    , m_bDebugOpenGL( false )
    , m_bVerbose( false )
    , m_bPerf( false )
    , m_bVblank( false )
    , m_bGlFinishHack( true )
    , m_glControllerVertBuffer( 0 )
    , m_unControllerVAO( 0 )
    , m_unSceneVAO( 0 )
    , m_nSceneMatrixLocation(-1)
    , m_nCameraTextureLocation(-1)
    , m_nControllerMatrixLocation( -1 )
    , m_nRenderModelMatrixLocation( -1 )
    , m_iTrackedControllerCount( 0 )
    , m_iTrackedControllerCount_Last( -1 )
    , m_iValidPoseCount( 0 )
    , m_iValidPoseCount_Last( -1 )
    , m_iSceneVolumeInit( 20 )
    , m_strPoseClasses("")
    , m_bShowCubes( true )
    , m_eye( 0 )
{

    for( int i = 1; i < argc; i++ )
    {
        if( !stricmp( argv[i], "-gldebug" ) )
        {
            m_bDebugOpenGL = true;
        }
        else if( !stricmp( argv[i], "-verbose" ) )
        {
            m_bVerbose = true;
        }
        else if( !stricmp( argv[i], "-novblank" ) )
        {
            m_bVblank = false;
        }
        else if( !stricmp( argv[i], "-noglfinishhack" ) )
        {
            m_bGlFinishHack = false;
        }
        else if( !stricmp( argv[i], "-noprintf" ) )
        {
            g_bPrintf = false;
        }
        else if ( !stricmp( argv[i], "-cubevolume" ) && ( argc > i + 1 ) && ( *argv[ i + 1 ] != '-' ) )
        {
            m_iSceneVolumeInit = atoi( argv[ i + 1 ] );
            i++;
        }
    }
    // other initialization tasks are done in BInit
    memset(m_rDevClassChar, 0, sizeof(m_rDevClassChar));
};


//-----------------------------------------------------------------------------
// Purpose: Destructor
//-----------------------------------------------------------------------------
CMainApplication::~CMainApplication()
{
    // work is done in Shutdown
    dprintf( "Shutdown" );
}


//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//            into a std::string
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString( vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
    uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
    if( unRequiredBufferLen == 0 )
        return "";

    char *pchBuffer = new char[ unRequiredBufferLen ];
    unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
    std::string sResult = pchBuffer;
    delete [] pchBuffer;
    return sResult;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::BInit()
{
    if ( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_TIMER ) < 0 )
    {
        printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
        return false;
    }

    // Loading the SteamVR Runtime
    vr::EVRInitError eError = vr::VRInitError_None;
    m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

    if ( eError != vr::VRInitError_None )
    {
        m_pHMD = NULL;
        char buf[1024];
        sprintf_s( buf, sizeof( buf ), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError ) );
        SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
        return false;
    }


    int nWindowPosX = 700;
    int nWindowPosY = 100;
    Uint32 unWindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;

    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 4 );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 1 );
    //SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );

    SDL_GL_SetAttribute( SDL_GL_MULTISAMPLEBUFFERS, 0 );
    SDL_GL_SetAttribute( SDL_GL_MULTISAMPLESAMPLES, 0 );
    if( m_bDebugOpenGL )
        SDL_GL_SetAttribute( SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG );

    m_pCompanionWindow = SDL_CreateWindow( "hellovr", nWindowPosX, nWindowPosY, m_nCompanionWindowWidth, m_nCompanionWindowHeight, unWindowFlags );
    if (m_pCompanionWindow == NULL)
    {
        printf( "%s - Window could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
        return false;
    }

    m_pContext = SDL_GL_CreateContext(m_pCompanionWindow);
    if (m_pContext == NULL)
    {
        printf( "%s - OpenGL context could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
        return false;
    }

    glewExperimental = GL_TRUE;
    GLenum nGlewError = glewInit();
    if (nGlewError != GLEW_OK)
    {
        printf( "%s - Error initializing GLEW! %s\n", __FUNCTION__, glewGetErrorString( nGlewError ) );
        return false;
    }
    glGetError(); // to clear the error caused deep in GLEW

    if ( SDL_GL_SetSwapInterval( m_bVblank ? 1 : 0 ) < 0 )
    {
        printf( "%s - Warning: Unable to set VSync! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
        return false;
    }


    m_strDriver = "No Driver";
    m_strDisplay = "No Display";

    m_strDriver = GetTrackedDeviceString( vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String );
    m_strDisplay = GetTrackedDeviceString( vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String );

    std::string strWindowTitle = "hellovr - " + m_strDriver + " " + m_strDisplay;
    SDL_SetWindowTitle( m_pCompanionWindow, strWindowTitle.c_str() );

    // Object parameter
     m_fScale = 1.0f;
 
     m_fNearClip = 0.1f;
     m_fFarClip = 30.0f;
 
    m_iTexture[vr::Eye_Left] = 0;
    m_iTexture[vr::Eye_Right] = 0;
     m_uiVertcount = 0;

    // uEyeCamera Init
    m_vision[vr::Eye_Left].Init(vr::Eye_Left + 1);
    m_vision[vr::Eye_Right].Init(vr::Eye_Right + 1);
 
    UpdateHMDMatrixPose();
    
    if (!BInitGL())
    {
        printf("%s - Unable to initialize OpenGL!\n", __FUNCTION__);
        return false;
    }

    if (!BInitCompositor())
    {
        printf("%s - Failed to initialize VR Compositor!\n", __FUNCTION__);
        return false;
    }

#ifdef USE_EYETRACKER
    // init EyeTrack
    tracker.Init(nImageWidth, nImageHeight);
#endif // USE_EYETRACKER

    vr::VRInput()->SetActionManifestPath( Path_MakeAbsolute( "../hellovr_actions.json", Path_StripFilename( Path_GetExecutablePath() ) ).c_str() );

    vr::VRInput()->GetActionHandle( "/actions/demo/in/HideCubes", &m_actionHideCubes );
    vr::VRInput()->GetActionHandle( "/actions/demo/in/HideThisController", &m_actionHideThisController);
    vr::VRInput()->GetActionHandle( "/actions/demo/in/TriggerHaptic", &m_actionTriggerHaptic );
    vr::VRInput()->GetActionHandle( "/actions/demo/in/AnalogInput", &m_actionAnalongInput );

    vr::VRInput()->GetActionSetHandle( "/actions/demo", &m_actionsetDemo );

    vr::VRInput()->GetActionHandle( "/actions/demo/out/Haptic_Left", &m_rHand[Left].m_actionHaptic );
    vr::VRInput()->GetInputSourceHandle( "/user/hand/left", &m_rHand[Left].m_source );
    vr::VRInput()->GetActionHandle( "/actions/demo/in/Hand_Left", &m_rHand[Left].m_actionPose );

    vr::VRInput()->GetActionHandle( "/actions/demo/out/Haptic_Right", &m_rHand[Right].m_actionHaptic );
    vr::VRInput()->GetInputSourceHandle( "/user/hand/right", &m_rHand[Right].m_source );
    vr::VRInput()->GetActionHandle( "/actions/demo/in/Hand_Right", &m_rHand[Right].m_actionPose );

    return true;
}


//-----------------------------------------------------------------------------
// Purpose: Outputs the string in message to debugging output.
//          All other parameters are ignored.
//          Does not return any meaningful value or reference.
//-----------------------------------------------------------------------------
void APIENTRY DebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const char* message, const void* userParam)
{
    dprintf( "GL Error: %s\n", message );
}


//-----------------------------------------------------------------------------
// Purpose: Initialize OpenGL. Returns true if OpenGL has been successfully
//          initialized, false if shaders could not be created.
//          If failure occurred in a module other than shaders, the function
//          may return true or throw an error. 
//-----------------------------------------------------------------------------
bool CMainApplication::BInitGL()
{
    if( m_bDebugOpenGL )
    {
        glDebugMessageCallback( (GLDEBUGPROC)DebugCallback, nullptr);
        glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE );
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    }

    if( !CreateAllShaders() )
        return false;

    SetupTexturemaps();
    SetupScene();
    SetupCameras();
    SetupStereoRenderTargets();
    SetupCompanionWindow();

    return true;
}


//-----------------------------------------------------------------------------
// Purpose: Initialize Compositor. Returns true if the compositor was
//          successfully initialized, false otherwise.
//-----------------------------------------------------------------------------
bool CMainApplication::BInitCompositor()
{
    vr::EVRInitError peError = vr::VRInitError_None;

    if ( !vr::VRCompositor() )
    {
        printf( "Compositor initialization failed. See log file for details\n" );
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::Shutdown()
{
    if( m_pHMD )
    {
        vr::VR_Shutdown();
        m_pHMD = NULL;
    }

    for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )
    {
        delete (*i);
    }
    m_vecRenderModels.clear();
    
    if( m_pContext )
    {
        if( m_bDebugOpenGL )
        {
            glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE );
            glDebugMessageCallback(nullptr, nullptr);
        }
        glDeleteBuffers(1, &m_glSceneVertBuffer);

        if ( m_unSceneProgramID )
        {
            glDeleteProgram( m_unSceneProgramID );
        }
        if ( m_unControllerTransformProgramID )
        {
            glDeleteProgram( m_unControllerTransformProgramID );
        }
        if ( m_unRenderModelProgramID )
        {
            glDeleteProgram( m_unRenderModelProgramID );
        }
        if ( m_unCompanionWindowProgramID )
        {
            glDeleteProgram( m_unCompanionWindowProgramID );
        }

        glDeleteRenderbuffers( 1, &leftEyeDesc.m_nDepthBufferId );
        glDeleteTextures( 1, &leftEyeDesc.m_nRenderTextureId );
        glDeleteFramebuffers( 1, &leftEyeDesc.m_nRenderFramebufferId );
        glDeleteTextures( 1, &leftEyeDesc.m_nResolveTextureId );
        glDeleteFramebuffers( 1, &leftEyeDesc.m_nResolveFramebufferId );

        glDeleteRenderbuffers( 1, &rightEyeDesc.m_nDepthBufferId );
        glDeleteTextures( 1, &rightEyeDesc.m_nRenderTextureId );
        glDeleteFramebuffers( 1, &rightEyeDesc.m_nRenderFramebufferId );
        glDeleteTextures( 1, &rightEyeDesc.m_nResolveTextureId );
        glDeleteFramebuffers( 1, &rightEyeDesc.m_nResolveFramebufferId );

        if( m_unCompanionWindowVAO != 0 )
        {
            glDeleteVertexArrays( 1, &m_unCompanionWindowVAO );
        }
        if( m_unSceneVAO != 0 )
        {
            glDeleteVertexArrays( 1, &m_unSceneVAO );
        }
        if( m_unControllerVAO != 0 )
        {
            glDeleteVertexArrays( 1, &m_unControllerVAO );
        }
    }

    m_vision[vr::Eye_Left].~uEyeCamera();
    m_vision[vr::Eye_Right].~uEyeCamera();

    if( m_pCompanionWindow )
    {
        SDL_DestroyWindow(m_pCompanionWindow);
        m_pCompanionWindow = NULL;
    }

    SDL_Quit();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::HandleInput()
{
    SDL_Event sdlEvent;
    bool bRet = false;

    while ( SDL_PollEvent( &sdlEvent ) != 0 )
    {
        if ( sdlEvent.type == SDL_QUIT || sdlEvent.type == SDL_WINDOWEVENT_CLOSE )
        {
            bRet = true;
        }
        else if ( sdlEvent.type == SDL_KEYDOWN )
        {
            if (sdlEvent.key.keysym.sym == SDLK_ESCAPE || sdlEvent.key.keysym.sym == SDLK_q)
            {
                bRet = true;
            }
            if (sdlEvent.key.keysym.sym == SDLK_d)
            {
                m_bShowCubes = !m_bShowCubes;
            }
#ifdef USE_EYETRACKER
            if (sdlEvent.key.keysym.sym == SDLK_r)
            {
                tracker.calibrated = false;
            }
            if (sdlEvent.key.keysym.sym == SDLK_c)
            {
                tracker.calibKeyPressed = true;
            }
#endif // USE_EYETRACKER
            if (sdlEvent.key.keysym.sym == SDLK_RIGHT)
            {
                m_eye = vr::Eye_Left;
            }
            if (sdlEvent.key.keysym.sym == SDLK_LEFT)
            {
                m_eye = vr::Eye_Right;
            }
            if (sdlEvent.key.keysym.sym == SDLK_UP)
            {
                m_vision[m_eye].SetFocus(5, true);
            }
            if (sdlEvent.key.keysym.sym == SDLK_DOWN)
            {
                m_vision[m_eye].SetFocus(-5, true);
            }
            if (sdlEvent.key.keysym.sym == SDLK_o)
            {
                m_vision[vr::Eye_Left].SetFocus(169, false);
                m_vision[vr::Eye_Right].SetFocus(150, false);
            }
            if (sdlEvent.key.keysym.sym == SDLK_p)
            {
                m_vision[vr::Eye_Left].SetFocus(200, false);
                m_vision[vr::Eye_Right].SetFocus(181, false);
            }
        }
    }

    // Process SteamVR events
    vr::VREvent_t event;
    while( m_pHMD->PollNextEvent( &event, sizeof( event ) ) )
    {
        ProcessVREvent( event );
    }

    // Process SteamVR action state
    // UpdateActionState is called each frame to update the state of the actions themselves. The application
    // controls which action sets are active with the provided array of VRActiveActionSet_t structs.
    vr::VRActiveActionSet_t actionSet = { 0 };
    actionSet.ulActionSet = m_actionsetDemo;
    vr::VRInput()->UpdateActionState( &actionSet, sizeof(actionSet), 1 );

    m_bShowCubes = !GetDigitalActionState( m_actionHideCubes );

    vr::VRInputValueHandle_t ulHapticDevice;
    if ( GetDigitalActionRisingEdge( m_actionTriggerHaptic, &ulHapticDevice ) )
    {
        if ( ulHapticDevice == m_rHand[Left].m_source )
        {
            vr::VRInput()->TriggerHapticVibrationAction( m_rHand[Left].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle );
        }
        if ( ulHapticDevice == m_rHand[Right].m_source )
        {
            vr::VRInput()->TriggerHapticVibrationAction( m_rHand[Right].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle );
        }
    }

    vr::InputAnalogActionData_t analogData;
    if ( vr::VRInput()->GetAnalogActionData( m_actionAnalongInput, &analogData, sizeof( analogData ), vr::k_ulInvalidInputValueHandle ) == vr::VRInputError_None && analogData.bActive )
    {
        m_vAnalogValue[0] = analogData.x;
        m_vAnalogValue[1] = analogData.y;
    }

    m_rHand[Left].m_bShowController = true;
    m_rHand[Right].m_bShowController = true;

    vr::VRInputValueHandle_t ulHideDevice;
    if ( GetDigitalActionState( m_actionHideThisController, &ulHideDevice ) )
    {
        if ( ulHideDevice == m_rHand[Left].m_source )
        {
            m_rHand[Left].m_bShowController = false;
        }
        if ( ulHideDevice == m_rHand[Right].m_source )
        {
            m_rHand[Right].m_bShowController = false;
        }
    }

    for ( EHand eHand = Left; eHand <= Right; ((int&)eHand)++ )
    {
        vr::InputPoseActionData_t poseData;
        if ( vr::VRInput()->GetPoseActionDataForNextFrame( m_rHand[eHand].m_actionPose, vr::TrackingUniverseStanding, &poseData, sizeof( poseData ), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None
            || !poseData.bActive || !poseData.pose.bPoseIsValid )
        {
            m_rHand[eHand].m_bShowController = false;
        }
        else
        {
            m_rHand[eHand].m_rmat4Pose = ConvertSteamVRMatrixToMatrix4( poseData.pose.mDeviceToAbsoluteTracking );

            vr::InputOriginInfo_t originInfo;
            if ( vr::VRInput()->GetOriginTrackedDeviceInfo( poseData.activeOrigin, &originInfo, sizeof( originInfo ) ) == vr::VRInputError_None 
                && originInfo.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid )
            {
                std::string sRenderModelName = GetTrackedDeviceString( originInfo.trackedDeviceIndex, vr::Prop_RenderModelName_String );
                if ( sRenderModelName != m_rHand[eHand].m_sRenderModelName )
                {
                    m_rHand[eHand].m_pRenderModel = FindOrLoadRenderModel( sRenderModelName.c_str() );
                    m_rHand[eHand].m_sRenderModelName = sRenderModelName;
                }
            }
        }
    }

    return bRet;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::RunMainLoop()
{
    bool bQuit = false;

    SDL_StartTextInput();
    SDL_ShowCursor( SDL_DISABLE );

#ifdef USE_EYETRACKER
    std::thread t_eyetrack_L(GetPupil, std::ref(tracker.mtx), vr::Eye_Left, std::ref(tracker.pupilCenterPt[vr::Eye_Left]));
    std::thread t_eyetrack_R(GetPupil, std::ref(tracker.mtx), vr::Eye_Right, std::ref(tracker.pupilCenterPt[vr::Eye_Right]));
#endif // USE_EYETRACKER

    //// focus
    //bool bGazePtChangeL = false;
    //bool bGazePtChangeR = false;
    //int focus[2][2] = { {289, 234}, {270, 215} };

    // fps
    unsigned int baseTime = SDL_GetTicks();
    int frameCount = 0;

    while ( !bQuit )  // Main loop
    {
        // fps
        frameCount++;
        unsigned int currentTime = SDL_GetTicks();
        if (currentTime - baseTime >= 1000)
        {
            float fps = float(frameCount) / (currentTime - baseTime) * 1000;
            printf("\rfps: %f", fps);
            baseTime = SDL_GetTicks();
            frameCount = 0;
        }

        bQuit = HandleInput();
        
#ifdef USE_EYETRACKER
        if (tracker.calibrated)
        {
            tracker.CalcurateGaze();

            //tracker.GetGazeDepth();
            //// Change focus
            //if (tracker.gazePt.y < 980 && !bGazePtChangeL) // 1024
            //{
            //    m_vision[vr::Eye_Left].SetFocus(focus[vr::Eye_Left][0], false);
            //    m_vision[vr::Eye_Right].SetFocus(focus[vr::Eye_Right][0], false);
            //    bGazePtChangeL = true;
            //    bGazePtChangeR = false;
            //}
            //if (tracker.gazePt.y > 980 && !bGazePtChangeR)
            //{
            //    m_vision[vr::Eye_Left].SetFocus(focus[vr::Eye_Left][1], false);
            //    m_vision[vr::Eye_Right].SetFocus(focus[vr::Eye_Right][1], false);
            //    bGazePtChangeL = false;
            //    bGazePtChangeR = true;
            //}
        }
        else
        {
            tracker.Calibrate();
        }
#endif // USE_EYETRACKER

        RenderFrame();
    }

#ifdef USE_EYETRACKER
    t_eyetrack_L.detach();
    t_eyetrack_R.detach();
    GetPupil_exit();
#endif // USE_EYETRACKER

    SDL_StopTextInput();
}

//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
void CMainApplication::ProcessVREvent( const vr::VREvent_t & event )
{
    switch( event.eventType )
    {
    case vr::VREvent_TrackedDeviceDeactivated:
        {
            dprintf( "Device %u detached.\n", event.trackedDeviceIndex );
        }
        break;
    case vr::VREvent_TrackedDeviceUpdated:
        {
            dprintf( "Device %u updated.\n", event.trackedDeviceIndex );
        }
        break;
    }
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::RenderFrame()
{
    // for now as fast as possible
    if ( m_pHMD )
    {
        RenderControllerAxes();
        RenderStereoTargets();
        RenderCompanionWindow();

        vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
        vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
    }

    if ( m_bVblank && m_bGlFinishHack )
    {
        //$ HACKHACK. From gpuview profiling, it looks like there is a bug where two renders and a present
        // happen right before and after the vsync causing all kinds of jittering issues. This glFinish()
        // appears to clear that up. Temporary fix while I try to get nvidia to investigate this problem.
        // 1/29/2014 mikesart
        glFinish();
    }

    // SwapWindow
    {
        SDL_GL_SwapWindow( m_pCompanionWindow );
    }

    // Clear
    {
        // We want to make sure the glFinish waits for the entire present to complete, not just the submission
        // of the command. So, we do a clear here right here so the glFinish will wait fully for the swap.
        glClearColor( 0, 0, 0, 1 );
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    }

    // Flush and wait for swap.
    if ( m_bVblank )
    {
        glFlush();
        glFinish();
    }

    // Spew out the controller and pose count whenever they change.
    if ( m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last )
    {
        m_iValidPoseCount_Last = m_iValidPoseCount;
        m_iTrackedControllerCount_Last = m_iTrackedControllerCount;
        
        dprintf( "PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount );
    }

    UpdateHMDMatrixPose();
}


//-----------------------------------------------------------------------------
// Purpose: Compiles a GL shader program and returns the handle. Returns 0 if
//            the shader couldn't be compiled for some reason.
//-----------------------------------------------------------------------------
GLuint CMainApplication::CompileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader )
{
    GLuint unProgramID = glCreateProgram();

    GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource( nSceneVertexShader, 1, &pchVertexShader, NULL);
    glCompileShader( nSceneVertexShader );

    GLint vShaderCompiled = GL_FALSE;
    glGetShaderiv( nSceneVertexShader, GL_COMPILE_STATUS, &vShaderCompiled);
    if ( vShaderCompiled != GL_TRUE)
    {
        dprintf("%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
        GLsizei bufSize;
        glGetShaderiv(nSceneVertexShader, GL_INFO_LOG_LENGTH, &bufSize);
        {
            std::vector<GLchar> infoLog(bufSize);
            GLsizei length;
            glGetShaderInfoLog(nSceneVertexShader, bufSize, &length, &infoLog[0]);
            std::cerr << &infoLog[0] << std::endl;
        }
        glDeleteProgram( unProgramID );
        glDeleteShader( nSceneVertexShader );
        return 0;
    }
    glAttachShader( unProgramID, nSceneVertexShader);
    glDeleteShader( nSceneVertexShader ); // the program hangs onto this once it's attached

    GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource( nSceneFragmentShader, 1, &pchFragmentShader, NULL);
    glCompileShader( nSceneFragmentShader );

    GLint fShaderCompiled = GL_FALSE;
    glGetShaderiv( nSceneFragmentShader, GL_COMPILE_STATUS, &fShaderCompiled);
    if (fShaderCompiled != GL_TRUE)
    {
        dprintf("%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader );
        GLsizei bufSize;
        glGetShaderiv(nSceneFragmentShader, GL_INFO_LOG_LENGTH, &bufSize);
        {
            std::vector<GLchar> infoLog(bufSize);
            GLsizei length;
            glGetShaderInfoLog(nSceneFragmentShader, bufSize, &length, &infoLog[0]);
            std::cerr << &infoLog[0] << std::endl;
        }
        glDeleteProgram( unProgramID );
        glDeleteShader( nSceneFragmentShader );
        return 0;    
    }
    glAttachShader( unProgramID, nSceneFragmentShader );
    glDeleteShader( nSceneFragmentShader ); // the program hangs onto this once it's attached

    glLinkProgram( unProgramID );

    GLint programSuccess = GL_TRUE;
    glGetProgramiv( unProgramID, GL_LINK_STATUS, &programSuccess);
    if ( programSuccess != GL_TRUE )
    {
        dprintf("%s - Error linking program %d!\n", pchShaderName, unProgramID);
        glDeleteProgram( unProgramID );
        return 0;
    }

    glUseProgram( unProgramID );
    glUseProgram( 0 );

    return unProgramID;
}


//-----------------------------------------------------------------------------
// Purpose: Creates all the shaders used by HelloVR SDL
//-----------------------------------------------------------------------------
bool CMainApplication::CreateAllShaders()
{
    m_unSceneProgramID = CompileGLShader(
        "Scene",

        // vertex shader
        "#version 410\n"
        "uniform mat4 matrix;\n"
        "layout(location = 0) in vec3 position;\n"
        "layout(location = 1) in vec2 v2UVcoordsIn;\n"
        "layout(location = 2) in vec3 v3NormalIn;\n"
        "out vec2 posUV;\n"
        "out vec2 v2UVcoords;\n"
        "void main()\n"
        "{\n"
        "    posUV = vec2(position.xy);\n"
        "    v2UVcoords = v2UVcoordsIn;\n"
        "    gl_Position = matrix * vec4(position, 1.f);\n"
        "}\n",

        // Fragment Shader
        "#version 410 core\n"
        "uniform sampler2D camera;\n"
        "uniform bool calibrated;\n"
        "in vec2 posUV;\n"
        "in vec2 v2UVcoords;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "   float a = radians(-180.f);\n"
        "   mat2 rotate = mat2(cos(a), -sin(a), sin(a), cos(a));\n"
        "   vec2 center = vec2(0.5f, 0.5f);\n"
        "   vec2 rot_uv = rotate * (v2UVcoords - center) + center;\n"
        "   if (calibrated) {\n"
        "       outputColor = texture(camera, rot_uv);\n"
        "   } else {\n"
        "       outputColor = texture(camera, posUV);\n"
        "   }\n"
        "}\n"
    );
    m_nSceneMatrixLocation = glGetUniformLocation( m_unSceneProgramID, "matrix" );
    if( m_nSceneMatrixLocation == -1 )
    {
        dprintf( "Unable to find matrix uniform in scene shader\n" );
        return false;
    }
    m_nCameraTextureLocation = glGetUniformLocation(m_unSceneProgramID, "camera");
    if (m_nCameraTextureLocation == -1)
    {
        dprintf("Unable to find camera texture uniform in scene shader\n");
        return false;
    }
    m_nCalibBooleanLocation = glGetUniformLocation(m_unSceneProgramID, "calibrated");
    if (m_nCalibBooleanLocation == -1)
    {
        dprintf("Unable to find bool uniform in scene shader\n");
        return false;
    }

    m_unControllerTransformProgramID = CompileGLShader(
        "Controller",

        // vertex shader
        "#version 410\n"
        "uniform mat4 matrix;\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec3 v3ColorIn;\n"
        "out vec4 v4Color;\n"
        "void main()\n"
        "{\n"
        "    v4Color.xyz = v3ColorIn; v4Color.a = 1.0;\n"
        "    gl_Position = matrix * position;\n"
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
    m_nControllerMatrixLocation = glGetUniformLocation( m_unControllerTransformProgramID, "matrix" );
    if( m_nControllerMatrixLocation == -1 )
    {
        dprintf( "Unable to find matrix uniform in controller shader\n" );
        return false;
    }

    m_unRenderModelProgramID = CompileGLShader(
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
        "    v2TexCoord = v2TexCoordsIn;\n"
        "    gl_Position = matrix * vec4(position.xyz, 1);\n"
        "}\n",

        //fragment shader
        "#version 410 core\n"
        "uniform sampler2D diffuse;\n"
        "in vec2 v2TexCoord;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "   outputColor = texture(diffuse, v2TexCoord);\n"
        "}\n"
    );
    m_nRenderModelMatrixLocation = glGetUniformLocation( m_unRenderModelProgramID, "matrix" );
    if( m_nRenderModelMatrixLocation == -1 )
    {
        dprintf( "Unable to find matrix uniform in render model shader\n" );
        return false;
    }

    m_unCompanionWindowProgramID = CompileGLShader(
        "CompanionWindow",

        // vertex shader
        "#version 410 core\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec2 v2UVIn;\n"
        "noperspective out vec2 v2UV;\n"
        "void main()\n"
        "{\n"
        "    v2UV = v2UVIn;\n"
        "    gl_Position = position;\n"
        "}\n",

        // fragment shader
        "#version 410 core\n"
        "uniform sampler2D mytexture;\n"
        "noperspective in vec2 v2UV;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "   outputColor = texture(mytexture, v2UV);\n"
        "}\n"
    );

    return m_unSceneProgramID != 0 
        && m_unControllerTransformProgramID != 0
        && m_unRenderModelProgramID != 0
        && m_unCompanionWindowProgramID != 0;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::SetupTexturemaps()
{
    nImageWidth = m_vision[vr::Eye_Left].GetWidth();
    nImageHeight = m_vision[vr::Eye_Left].GetHeight();
    nTextureWidth = texture_size(nImageWidth);
    nTextureHeight = texture_size(nImageHeight);

    glGenTextures(2, m_iTexture);

    // Left
    glBindTexture(GL_TEXTURE_2D, m_iTexture[vr::Eye_Left]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nTextureWidth, nTextureHeight,
        0, GL_RGBA, GL_UNSIGNED_BYTE, m_vision[vr::Eye_Left].GetImagePtr());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // Right
    glBindTexture(GL_TEXTURE_2D, m_iTexture[vr::Eye_Right]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nTextureWidth, nTextureHeight,
        0, GL_RGBA, GL_UNSIGNED_BYTE, m_vision[vr::Eye_Right].GetImagePtr());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    
    // null
    glBindTexture( GL_TEXTURE_2D, 0 );

    return ( m_iTexture != 0 );
}


//-----------------------------------------------------------------------------
// Purpose: create a sea of cubes
//-----------------------------------------------------------------------------
void CMainApplication::SetupScene()
{
    if ( !m_pHMD )
        return;

    std::vector<float> vertdataarray;

    //// Change object scale
    //Matrix4 matScale;
    //matScale.scale( m_fScale, m_fScale, m_fScale );
    //Matrix4 mat = matScale;
    //AddPlaneToScene(mat, vertdataarray);

    AddPlaneMeshToScene(vertdataarray);
    m_uiVertcount = vertdataarray.size() / 5;
    
    glGenVertexArrays( 1, &m_unSceneVAO );
    glBindVertexArray( m_unSceneVAO );

    glGenBuffers( 1, &m_glSceneVertBuffer );
    glBindBuffer( GL_ARRAY_BUFFER, m_glSceneVertBuffer );
    glBufferData( GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STATIC_DRAW);

    GLsizei stride = sizeof(VertexDataScene);
    uintptr_t offset = 0;

    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride , (const void *)offset);

    offset += sizeof(Vector3);
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

    glBindVertexArray( 0 );
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    std::vector<float>().swap(vertdataarray);
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::AddVertex( float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata )
{
    // (x, y, z) = (f10, f11, f12)
    vertdata.push_back( fl0 );
    vertdata.push_back( fl1 );
    vertdata.push_back( fl2 );
    // (u, v) = (f13, f14)
    vertdata.push_back( fl3 );
    vertdata.push_back( fl4 );
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::AddCubeToScene( Matrix4 mat, std::vector<float> &vertdata )
{
    // Matrix4 mat( outermat.data() );

    Vector4 A = mat * Vector4( 0, 0, 0, 1 );
    Vector4 B = mat * Vector4( 1, 0, 0, 1 );
    Vector4 C = mat * Vector4( 1, 1, 0, 1 );
    Vector4 D = mat * Vector4( 0, 1, 0, 1 );
    Vector4 E = mat * Vector4( 0, 0, 1, 1 );
    Vector4 F = mat * Vector4( 1, 0, 1, 1 );
    Vector4 G = mat * Vector4( 1, 1, 1, 1 );
    Vector4 H = mat * Vector4( 0, 1, 1, 1 );

    // triangles instead of quads
    AddVertex( E.x, E.y, E.z, 0, 1, vertdata ); //Front
    AddVertex( F.x, F.y, F.z, 1, 1, vertdata );
    AddVertex( G.x, G.y, G.z, 1, 0, vertdata );
    AddVertex( G.x, G.y, G.z, 1, 0, vertdata );
    AddVertex( H.x, H.y, H.z, 0, 0, vertdata );
    AddVertex( E.x, E.y, E.z, 0, 1, vertdata );
                     
    AddVertex( B.x, B.y, B.z, 0, 1, vertdata ); //Back
    AddVertex( A.x, A.y, A.z, 1, 1, vertdata );
    AddVertex( D.x, D.y, D.z, 1, 0, vertdata );
    AddVertex( D.x, D.y, D.z, 1, 0, vertdata );
    AddVertex( C.x, C.y, C.z, 0, 0, vertdata );
    AddVertex( B.x, B.y, B.z, 0, 1, vertdata );
                    
    AddVertex( H.x, H.y, H.z, 0, 1, vertdata ); //Top
    AddVertex( G.x, G.y, G.z, 1, 1, vertdata );
    AddVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddVertex( D.x, D.y, D.z, 0, 0, vertdata );
    AddVertex( H.x, H.y, H.z, 0, 1, vertdata );
                
    AddVertex( A.x, A.y, A.z, 0, 1, vertdata ); //Bottom
    AddVertex( B.x, B.y, B.z, 1, 1, vertdata );
    AddVertex( F.x, F.y, F.z, 1, 0, vertdata );
    AddVertex( F.x, F.y, F.z, 1, 0, vertdata );
    AddVertex( E.x, E.y, E.z, 0, 0, vertdata );
    AddVertex( A.x, A.y, A.z, 0, 1, vertdata );
                    
    AddVertex( A.x, A.y, A.z, 0, 1, vertdata ); //Left
    AddVertex( E.x, E.y, E.z, 1, 1, vertdata );
    AddVertex( H.x, H.y, H.z, 1, 0, vertdata );
    AddVertex( H.x, H.y, H.z, 1, 0, vertdata );
    AddVertex( D.x, D.y, D.z, 0, 0, vertdata );
    AddVertex( A.x, A.y, A.z, 0, 1, vertdata );

    AddVertex( F.x, F.y, F.z, 0, 1, vertdata ); //Right
    AddVertex( B.x, B.y, B.z, 1, 1, vertdata );
    AddVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddVertex( G.x, G.y, G.z, 0, 0, vertdata );
    AddVertex( F.x, F.y, F.z, 0, 1, vertdata );
}

void CMainApplication::AddPlaneToScene(Matrix4 mat, std::vector<float>& vertdata)
{
    Vector4 A = mat * Vector4(0, 0, 0, 1);
    Vector4 B = mat * Vector4(1, 0, 0, 1);
    Vector4 C = mat * Vector4(1, 1, 0, 1);
    Vector4 D = mat * Vector4(0, 1, 0, 1);

    AddVertex(B.x, B.y, B.z, 1, 0, vertdata);
    AddVertex(A.x, A.y, A.z, 1, 1, vertdata);
    AddVertex(D.x, D.y, D.z, 0, 1, vertdata);
    AddVertex(D.x, D.y, D.z, 0, 1, vertdata);
    AddVertex(C.x, C.y, C.z, 0, 0, vertdata);
    AddVertex(B.x, B.y, B.z, 1, 0, vertdata);
}

void CMainApplication::AddPlaneMeshToScene(std::vector<float>& vertdata)
{
    std::cout << "Prepareing Image Plane Mesh..." << std::endl;
    const int DIV = 256;

    // gen map matrix
    cv::Mat new_cmat, mapx, mapy;
    new_cmat = cv::getOptimalNewCameraMatrix(cam_mat, dist_coeffs, cv::Size(2048, 2048), 1);
    cv::initUndistortRectifyMap(cam_mat, dist_coeffs, cv::Mat::eye(3, 3, CV_32FC1), new_cmat,
        cv::Size(2048, 2048), CV_32FC1, mapx, mapy);

    cv::Mat uv;
    cv::resize(mapx, uv, cv::Size(DIV + 1, DIV + 1));
    mapx = uv / 2048;
    cv::resize(mapy, uv, cv::Size(DIV + 1, DIV + 1));
    mapy = uv / 2048;

    // square 4 points on uv coordinate
    Vector2 a = Vector2(0, 0);
    Vector2 b = Vector2(1, 0);
    Vector2 c = Vector2(1, 1);
    Vector2 d = Vector2(0, 1);
    Vector2 A = (1.0f / DIV) * a;
    Vector2 B = (1.0f / DIV) * b;
    Vector2 C = (1.0f / DIV) * c;
    Vector2 D = (1.0f / DIV) * d;
    
    for (int i = 0; i < DIV; i++)
    {
        for (int j = 0; j < DIV; j++)
        {
            Vector2 ofs = Vector2(1.0f / DIV * j, 1.0f / DIV * i);
            AddVertex(A.x + ofs.x, A.y + ofs.y, 0, mapx.at<float>(a.x + j, a.y + i), mapy.at<float>(a.x + j, a.y + i), vertdata);
            AddVertex(B.x + ofs.x, B.y + ofs.y, 0, mapx.at<float>(b.x + j, b.y + i), mapy.at<float>(b.x + j, b.y + i), vertdata);
            AddVertex(D.x + ofs.x, D.y + ofs.y, 0, mapx.at<float>(d.x + j, d.y + i), mapy.at<float>(d.x + j, d.y + i), vertdata);
            AddVertex(D.x + ofs.x, D.y + ofs.y, 0, mapx.at<float>(d.x + j, d.y + i), mapy.at<float>(d.x + j, d.y + i), vertdata);
            AddVertex(B.x + ofs.x, B.y + ofs.y, 0, mapx.at<float>(b.x + j, b.y + i), mapy.at<float>(b.x + j, b.y + i), vertdata);
            AddVertex(C.x + ofs.x, C.y + ofs.y, 0, mapx.at<float>(c.x + j, c.y + i), mapy.at<float>(c.x + j, c.y + i), vertdata);
        }
    }
    std::cout << "Done" << std::endl;
}


//-----------------------------------------------------------------------------
// Purpose: Draw all of the controllers as X/Y/Z lines
//-----------------------------------------------------------------------------
void CMainApplication::RenderControllerAxes()
{
    // Don't attempt to update controllers if input is not available
    if( !m_pHMD->IsInputAvailable() )
        return;

    std::vector<float> vertdataarray;

    m_uiControllerVertcount = 0;
    m_iTrackedControllerCount = 0;

    for ( EHand eHand = Left; eHand <= Right; ((int&)eHand)++ )
    {
        if ( !m_rHand[eHand].m_bShowController )
            continue;

        const Matrix4 & mat = m_rHand[eHand].m_rmat4Pose;

        Vector4 center = mat * Vector4( 0, 0, 0, 1 );

        for ( int i = 0; i < 3; ++i )
        {
            Vector3 color( 0, 0, 0 );
            Vector4 point( 0, 0, 0, 1 );
            point[i] += 0.05f;  // offset in X, Y, Z
            color[i] = 1.0;  // R, G, B
            point = mat * point;
            vertdataarray.push_back( center.x );
            vertdataarray.push_back( center.y );
            vertdataarray.push_back( center.z );

            vertdataarray.push_back( color.x );
            vertdataarray.push_back( color.y );
            vertdataarray.push_back( color.z );
        
            vertdataarray.push_back( point.x );
            vertdataarray.push_back( point.y );
            vertdataarray.push_back( point.z );
        
            vertdataarray.push_back( color.x );
            vertdataarray.push_back( color.y );
            vertdataarray.push_back( color.z );
        
            m_uiControllerVertcount += 2;
        }

        Vector4 start = mat * Vector4( 0, 0, -0.02f, 1 );
        Vector4 end = mat * Vector4( 0, 0, -39.f, 1 );
        Vector3 color( .92f, .92f, .71f );

        vertdataarray.push_back( start.x );vertdataarray.push_back( start.y );vertdataarray.push_back( start.z );
        vertdataarray.push_back( color.x );vertdataarray.push_back( color.y );vertdataarray.push_back( color.z );

        vertdataarray.push_back( end.x );vertdataarray.push_back( end.y );vertdataarray.push_back( end.z );
        vertdataarray.push_back( color.x );vertdataarray.push_back( color.y );vertdataarray.push_back( color.z );
        m_uiControllerVertcount += 2;
    }

    // Setup the VAO the first time through.
    if ( m_unControllerVAO == 0 )
    {
        glGenVertexArrays( 1, &m_unControllerVAO );
        glBindVertexArray( m_unControllerVAO );

        glGenBuffers( 1, &m_glControllerVertBuffer );
        glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );

        GLuint stride = 2 * 3 * sizeof( float );
        uintptr_t offset = 0;

        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        offset += sizeof( Vector3 );
        glEnableVertexAttribArray( 1 );
        glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        glBindVertexArray( 0 );
    }

    glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );

    // set vertex data if we have some
    if( vertdataarray.size() > 0 )
    {
        //$ TODO: Use glBufferSubData for this...
        glBufferData( GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW );
    }
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::SetupCameras()
{
    m_mat4Projection[vr::Eye_Left] = GetHMDMatrixProjectionEye(vr::Eye_Left);
    m_mat4Projection[vr::Eye_Right] = GetHMDMatrixProjectionEye(vr::Eye_Right);
    m_mat4eyePose[vr::Eye_Left] = GetHMDMatrixPoseEye(vr::Eye_Left);
    m_mat4eyePose[vr::Eye_Right] = GetHMDMatrixPoseEye(vr::Eye_Right);
}


//-----------------------------------------------------------------------------
// Purpose: Creates a frame buffer. Returns true if the buffer was set up.
//          Returns false if the setup failed.
//-----------------------------------------------------------------------------
bool CMainApplication::CreateFrameBuffer( int nWidth, int nHeight, FramebufferDesc &framebufferDesc )
{
    glGenFramebuffers(1, &framebufferDesc.m_nRenderFramebufferId );
    glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nRenderFramebufferId);

    glGenRenderbuffers(1, &framebufferDesc.m_nDepthBufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, nWidth, nHeight );
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,    framebufferDesc.m_nDepthBufferId );

    glGenTextures(1, &framebufferDesc.m_nRenderTextureId );
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId );
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, nWidth, nHeight, true);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId, 0);

    glGenFramebuffers(1, &framebufferDesc.m_nResolveFramebufferId );
    glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nResolveFramebufferId);

    glGenTextures(1, &framebufferDesc.m_nResolveTextureId );
    glBindTexture(GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId, 0);

    // check FBO status
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        return false;
    }

    glBindFramebuffer( GL_FRAMEBUFFER, 0 );

    return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::SetupStereoRenderTargets()
{
    if ( !m_pHMD )
        return false;

    m_pHMD->GetRecommendedRenderTargetSize( &m_nRenderWidth, &m_nRenderHeight );

    CreateFrameBuffer( m_nRenderWidth, m_nRenderHeight, leftEyeDesc );
    CreateFrameBuffer( m_nRenderWidth, m_nRenderHeight, rightEyeDesc );
    
    return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::SetupCompanionWindow()
{
    if ( !m_pHMD )
        return;

    std::vector<VertexDataWindow> vVerts;

    // left eye verts
    vVerts.push_back( VertexDataWindow( Vector2(-1, -1), Vector2(0, 1)) );
    vVerts.push_back( VertexDataWindow( Vector2(0, -1), Vector2(1, 1)) );
    vVerts.push_back( VertexDataWindow( Vector2(-1, 1), Vector2(0, 0)) );
    vVerts.push_back( VertexDataWindow( Vector2(0, 1), Vector2(1, 0)) );

    // right eye verts
    vVerts.push_back( VertexDataWindow( Vector2(0, -1), Vector2(0, 1)) );
    vVerts.push_back( VertexDataWindow( Vector2(1, -1), Vector2(1, 1)) );
    vVerts.push_back( VertexDataWindow( Vector2(0, 1), Vector2(0, 0)) );
    vVerts.push_back( VertexDataWindow( Vector2(1, 1), Vector2(1, 0)) );

    GLushort vIndices[] = { 0, 1, 3,   0, 3, 2,   4, 5, 7,   4, 7, 6};
    m_uiCompanionWindowIndexSize = _countof(vIndices);

    glGenVertexArrays( 1, &m_unCompanionWindowVAO );
    glBindVertexArray( m_unCompanionWindowVAO );

    glGenBuffers( 1, &m_glCompanionWindowIDVertBuffer );
    glBindBuffer( GL_ARRAY_BUFFER, m_glCompanionWindowIDVertBuffer );
    glBufferData( GL_ARRAY_BUFFER, vVerts.size()*sizeof(VertexDataWindow), &vVerts[0], GL_STATIC_DRAW );

    glGenBuffers( 1, &m_glCompanionWindowIDIndexBuffer );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_glCompanionWindowIDIndexBuffer );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, m_uiCompanionWindowIndexSize*sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW );

    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof( VertexDataWindow, position ) );

    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof( VertexDataWindow, texCoord ) );

    glBindVertexArray( 0 );

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::RenderStereoTargets()
{
    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
    glEnable( GL_MULTISAMPLE );

    // Left Eye
    glBindFramebuffer( GL_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId );
     glViewport(0, 0, m_nRenderWidth, m_nRenderHeight );
     RenderScene( vr::Eye_Left );
     glBindFramebuffer( GL_FRAMEBUFFER, 0 );
    
    glDisable( GL_MULTISAMPLE );
         
     glBindFramebuffer(GL_READ_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.m_nResolveFramebufferId );

    glBlitFramebuffer( 0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight, 
        GL_COLOR_BUFFER_BIT,
         GL_LINEAR );

     glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0 );    

    glEnable( GL_MULTISAMPLE );

    // Right Eye
    glBindFramebuffer( GL_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId );
     glViewport(0, 0, m_nRenderWidth, m_nRenderHeight );
     RenderScene( vr::Eye_Right );
     glBindFramebuffer( GL_FRAMEBUFFER, 0 );
     
    glDisable( GL_MULTISAMPLE );

     glBindFramebuffer(GL_READ_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId );
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.m_nResolveFramebufferId );
    
    glBlitFramebuffer( 0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight, 
        GL_COLOR_BUFFER_BIT,
         GL_LINEAR  );

     glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0 );
}

//-----------------------------------------------------------------------------
// Purpose: Renders a scene with respect to nEye.
//-----------------------------------------------------------------------------
void CMainApplication::RenderScene( vr::Hmd_Eye nEye )
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    if( m_bShowCubes )
    {
        glUseProgram( m_unSceneProgramID );

        Matrix4 matTransform;
        matTransform.translate(-0.5f, -0.5f, -0.3f);
        //Matrix3 mat4stCam = Matrix3(stereo_cam_mat[nEye]);
        //Matrix3 mat4stRot = Matrix3(stereo_rotate_mat[nEye]);
        //Matrix3 intrinsic = IntrinsicsFromProjection(nEye);
        //Matrix4 hmd_proj = m_mat4Projection[nEye] * m_mat4eyePose[nEye];
        float ipd = std::fabs(m_pHMD->GetEyeToHeadTransform(vr::Eye_Left).m[0][3])
            + std::fabs(m_pHMD->GetEyeToHeadTransform(vr::Eye_Right).m[0][3]); // scale: m
        glUniformMatrix4fv(m_nSceneMatrixLocation, 1, GL_FALSE,
            (m_mat4Projection[nEye] * m_mat4StereoProjection[nEye].invert() * matTransform).get());
        //(m_mat4Projection[nEye] * m_mat4eyePose[nEye] * m_mat4StereoProjection[nEye].invert() * matTransform).get());
        glUniform1i(m_nCameraTextureLocation, 0);
#ifdef USE_EYETRACKER
        glUniform1i(m_nCalibBooleanLocation, tracker.calibrated);
#else
        glUniform1i(m_nCalibBooleanLocation, true);
#endif // USE_EYETRACKER

        glBindVertexArray( m_unSceneVAO );

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_iTexture[nEye]);
        UpdateTexture( nEye );

        glDrawArrays( GL_TRIANGLES, 0, m_uiVertcount );

        glBindVertexArray( 0 );
    }

    bool bIsInputAvailable = m_pHMD->IsInputAvailable();

    if( bIsInputAvailable )
    {
        // draw the controller axis lines
        glUseProgram( m_unControllerTransformProgramID );
        glUniformMatrix4fv( m_nControllerMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix( nEye ).get() );
        glBindVertexArray( m_unControllerVAO );
        glDrawArrays( GL_LINES, 0, m_uiControllerVertcount );
        glBindVertexArray( 0 );
    }

    // ----- Render Model rendering -----
    glUseProgram( m_unRenderModelProgramID );

    for ( EHand eHand = Left; eHand <= Right; ((int&)eHand)++ )
    {
        if ( !m_rHand[eHand].m_bShowController || !m_rHand[eHand].m_pRenderModel )
            continue;

        const Matrix4 & matDeviceToTracking = m_rHand[eHand].m_rmat4Pose;
        Matrix4 matMVP = GetCurrentViewProjectionMatrix( nEye ) * matDeviceToTracking;
        glUniformMatrix4fv( m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.get() );

        m_rHand[eHand].m_pRenderModel->Draw();
    }

    glUseProgram( 0 );
}

void CMainApplication::UpdateTexture(vr::Hmd_Eye nEye)
{
#ifdef USE_EYETRACKER
    if (tracker.calibrated)
    {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, nImageWidth, nImageHeight,
            GL_RGBA, GL_UNSIGNED_BYTE, m_vision[nEye].GetImagePtr());

        //要マルチテクスチャ
        //glTexSubImage2D(GL_TEXTURE_2D, 0, tracker.gazePt[nEye].x, tracker.gazePt[nEye].y, tracker.gazePtImg.cols, tracker.gazePtImg.rows,
        //    GL_BGR, GL_UNSIGNED_BYTE, tracker.gazePtImg.data);
    }
    else
    {
        glTexSubImage2D(GL_TEXTURE_2D, 0, tracker.gazePtCalib.x, tracker.gazePtCalib.y, tracker.gazePtImg.cols, tracker.gazePtImg.rows,
            GL_BGR, GL_UNSIGNED_BYTE, tracker.gazePtImg.data);
    }
#else
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, nImageWidth, nImageHeight,
        GL_RGBA, GL_UNSIGNED_BYTE, m_vision[nEye].GetImagePtr());
#endif // USE_EYETRACKER
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::RenderCompanionWindow()
{
    glDisable(GL_DEPTH_TEST);
    glViewport( 0, 0, m_nCompanionWindowWidth, m_nCompanionWindowHeight );

    glBindVertexArray( m_unCompanionWindowVAO );
    glUseProgram( m_unCompanionWindowProgramID );

    // render left eye (first half of index array )
    glBindTexture(GL_TEXTURE_2D, leftEyeDesc.m_nResolveTextureId );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glDrawElements( GL_TRIANGLES, m_uiCompanionWindowIndexSize/2, GL_UNSIGNED_SHORT, 0 );

    // render right eye (second half of index array )
    glBindTexture(GL_TEXTURE_2D, rightEyeDesc.m_nResolveTextureId  );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glDrawElements( GL_TRIANGLES, m_uiCompanionWindowIndexSize/2, GL_UNSIGNED_SHORT, (const void *)(uintptr_t)(m_uiCompanionWindowIndexSize) );

    glBindVertexArray( 0 );
    glUseProgram( 0 );
}


//-----------------------------------------------------------------------------
// Purpose: Gets a Matrix Projection Eye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye )
{
    if ( !m_pHMD )
        return Matrix4();

    vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix( nEye, m_fNearClip, m_fFarClip );

    return Matrix4(
        mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
        mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1], 
        mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2], 
        mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
    );
}

Matrix3 CMainApplication::IntrinsicsFromProjection(vr::Hmd_Eye nEye, bool ydown)
{
    if (!m_pHMD)
        return Matrix3();

    vr::HmdMatrix44_t proj = m_pHMD->GetProjectionMatrix(nEye, m_fNearClip, m_fFarClip);

    int height = 1600;
    int width = 1440;

    Vector2 focal, center;
    focal.x = proj.m[0][0] * width / 2.f;
    focal.y = proj.m[1][1] * height / 2.f;
    if (!ydown) focal.y *= -1.f;
    center.x = -(proj.m[2][0] * width - width) / 2.f;
    if (ydown) center.y = (proj.m[2][1] * height + height) / 2.f;
    else center.y = (height - proj.m[2][1] * height) / 2.f;

    return Matrix3(
        focal.x, 0.f, center.x,
        0.f, focal.y, center.y,
        0.f, 0.f, 1.f
    );
}

//-----------------------------------------------------------------------------
// Purpose: Gets an HMDMatrixPoseEye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetHMDMatrixPoseEye( vr::Hmd_Eye nEye )
{
    if ( !m_pHMD )
        return Matrix4();

    vr::HmdMatrix34_t matEyeRight = m_pHMD->GetEyeToHeadTransform( nEye );
    Matrix4 matrixObj(
        matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.f, 
        matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.f,
        matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.f,
        matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.f
        );

    return matrixObj.invert();
}


//-----------------------------------------------------------------------------
// Purpose: Gets a Current View Projection Matrix with respect to nEye,
//          which may be an Eye_Left or an Eye_Right.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye )
{
    Matrix4 matMVP;
    matMVP = m_mat4Projection[nEye] * m_mat4eyePose[nEye] * m_mat4HMDPose;
    return matMVP;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::UpdateHMDMatrixPose()
{
    if ( !m_pHMD )
        return;

    vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

    m_iValidPoseCount = 0;
    m_strPoseClasses = "";
    for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )
    {
        if ( m_rTrackedDevicePose[nDevice].bPoseIsValid )
        {
            m_iValidPoseCount++;
            m_rmat4DevicePose[nDevice] = ConvertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
            if (m_rDevClassChar[nDevice]==0)
            {
                switch (m_pHMD->GetTrackedDeviceClass(nDevice))
                {
                case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
                case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
                case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
                case vr::TrackedDeviceClass_GenericTracker:    m_rDevClassChar[nDevice] = 'G'; break;
                case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
                default:                                       m_rDevClassChar[nDevice] = '?'; break;
                }
            }
            m_strPoseClasses += m_rDevClassChar[nDevice];
        }
    }

    if ( m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
    {
        m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
        m_mat4HMDPose.invert();
    }
}


//-----------------------------------------------------------------------------
// Purpose: Finds a render model we've already loaded or loads a new one
//-----------------------------------------------------------------------------
CGLRenderModel *CMainApplication::FindOrLoadRenderModel( const char *pchRenderModelName )
{
    CGLRenderModel *pRenderModel = NULL;
    for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )
    {
        if( !stricmp( (*i)->GetName().c_str(), pchRenderModelName ) )
        {
            pRenderModel = *i;
            break;
        }
    }

    // load the model if we didn't find one
    if( !pRenderModel )
    {
        vr::RenderModel_t *pModel;
        vr::EVRRenderModelError error;
        while ( 1 )
        {
            error = vr::VRRenderModels()->LoadRenderModel_Async( pchRenderModelName, &pModel );
            if ( error != vr::VRRenderModelError_Loading )
                break;

            ThreadSleep( 1 );
        }

        if ( error != vr::VRRenderModelError_None )
        {
            dprintf( "Unable to load render model %s - %s\n", pchRenderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum( error ) );
            return NULL; // move on to the next tracked device
        }

        vr::RenderModel_TextureMap_t *pTexture;
        while ( 1 )
        {
            error = vr::VRRenderModels()->LoadTexture_Async( pModel->diffuseTextureId, &pTexture );
            if ( error != vr::VRRenderModelError_Loading )
                break;

            ThreadSleep( 1 );
        }

        if ( error != vr::VRRenderModelError_None )
        {
            dprintf( "Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, pchRenderModelName );
            vr::VRRenderModels()->FreeRenderModel( pModel );
            return NULL; // move on to the next tracked device
        }

        pRenderModel = new CGLRenderModel( pchRenderModelName );
        if ( !pRenderModel->BInit( *pModel, *pTexture ) )
        {
            dprintf( "Unable to create GL model from render model %s\n", pchRenderModelName );
            delete pRenderModel;
            pRenderModel = NULL;
        }
        else
        {
            m_vecRenderModels.push_back( pRenderModel );
        }
        vr::VRRenderModels()->FreeRenderModel( pModel );
        vr::VRRenderModels()->FreeTexture( pTexture );
    }
    return pRenderModel;
}


//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose )
{
    Matrix4 matrixObj(
        matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
        matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
        matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
        matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
        );
    return matrixObj;
}


//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL Render Models
//-----------------------------------------------------------------------------
CGLRenderModel::CGLRenderModel( const std::string & sRenderModelName )
    : m_sModelName( sRenderModelName )
{
    m_glIndexBuffer = 0;
    m_glVertArray = 0;
    m_glVertBuffer = 0;
    m_glTexture = 0;
    m_unVertexCount = 0;
}


CGLRenderModel::~CGLRenderModel()
{
    Cleanup();
}


//-----------------------------------------------------------------------------
// Purpose: Allocates and populates the GL resources for a render model
//-----------------------------------------------------------------------------
bool CGLRenderModel::BInit( const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture )
{
    // create and bind a VAO to hold state for this model
    glGenVertexArrays( 1, &m_glVertArray );
    glBindVertexArray( m_glVertArray );

    // Populate a vertex buffer
    glGenBuffers( 1, &m_glVertBuffer );
    glBindBuffer( GL_ARRAY_BUFFER, m_glVertBuffer );
    glBufferData( GL_ARRAY_BUFFER, sizeof( vr::RenderModel_Vertex_t ) * vrModel.unVertexCount, vrModel.rVertexData, GL_STATIC_DRAW );

    // Identify the components in the vertex buffer
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vPosition ) );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vNormal ) );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, rfTextureCoord ) );

    // Create and populate the index buffer
    glGenBuffers( 1, &m_glIndexBuffer );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_glIndexBuffer );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( uint16_t ) * vrModel.unTriangleCount * 3, vrModel.rIndexData, GL_STATIC_DRAW );

    glBindVertexArray( 0 );

    // create and populate the texture
    glGenTextures(1, &m_glTexture );
    glBindTexture( GL_TEXTURE_2D, m_glTexture );

    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTexture.unWidth, vrDiffuseTexture.unHeight,
        0, GL_RGBA, GL_UNSIGNED_BYTE, vrDiffuseTexture.rubTextureMapData );

    // If this renders black ask McJohn what's wrong.
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );

    GLfloat fLargest;
    glGetFloatv( GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest );

    glBindTexture( GL_TEXTURE_2D, 0 );

    m_unVertexCount = vrModel.unTriangleCount * 3;

    return true;
}


//-----------------------------------------------------------------------------
// Purpose: Frees the GL resources for a render model
//-----------------------------------------------------------------------------
void CGLRenderModel::Cleanup()
{
    if( m_glVertBuffer )
    {
        glDeleteBuffers(1, &m_glIndexBuffer);
        glDeleteVertexArrays( 1, &m_glVertArray );
        glDeleteBuffers(1, &m_glVertBuffer);
        m_glIndexBuffer = 0;
        m_glVertArray = 0;
        m_glVertBuffer = 0;
    }
}


//-----------------------------------------------------------------------------
// Purpose: Draws the render model
//-----------------------------------------------------------------------------
void CGLRenderModel::Draw()
{
    glBindVertexArray( m_glVertArray );

    glActiveTexture( GL_TEXTURE0 );
    glBindTexture( GL_TEXTURE_2D, m_glTexture );

    glDrawElements( GL_TRIANGLES, m_unVertexCount, GL_UNSIGNED_SHORT, 0 );

    glBindVertexArray( 0 );
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    CMainApplication *pMainApplication = new CMainApplication( argc, argv );

    if (!pMainApplication->BInit())
    {
        pMainApplication->Shutdown();
        return 1;
    }

    pMainApplication->RunMainLoop();

    pMainApplication->Shutdown();

    return 0;
}
