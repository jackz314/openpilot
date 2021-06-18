/* This copyright notice applies to this file only:
 *
 * \copyright
 * Copyright (c) 2020, Jack Zhang. All rights reserved.
 * Copyright (c) 2013-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <dlfcn.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "inc/NvFBC.h"
// If you have the CUDA Toolkit Installed, you can include the CUDA headers
// #include "cuda.h"
#include "inc/cuda_drvapi_dynlink_cuda.h"

#include "cereal/messaging/messaging.h"

#define LIB_NVFBC_NAME "libnvidia-fbc.so.1"
#define LIB_CUDA_NAME  "libcuda.so.1"

#define N_FRAMES 100

#include <X11/Xlib.h>

/*
 * CUDA entry points
 */
typedef CUresult (* CUINITPROC) (unsigned int Flags);
typedef CUresult (* CUDEVICEGETPROC) (CUdevice *device, int ordinal);
typedef CUresult (* CUCTXCREATEV2PROC) (CUcontext *pctx, unsigned int flags, CUdevice dev);
typedef CUresult (* CUMEMCPYDTOHV2PROC) (void *dstHost, CUdeviceptr srcDevice, size_t ByteCount);

static CUINITPROC cuInit_ptr = NULL;
static CUDEVICEGETPROC cuDeviceGet_ptr = NULL;
static CUCTXCREATEV2PROC cuCtxCreate_v2_ptr = NULL;
static CUMEMCPYDTOHV2PROC cuMemcpyDtoH_v2_ptr = NULL;

/**
 * Dynamically opens the CUDA library and resolves the symbols that are
 * needed for this application.
 *
 * \param [out] libCUDA
 *   A pointer to the opened CUDA library.
 *
 * \return
 *   NVFBC_TRUE in case of success, NVFBC_FALSE otherwise.
 */
static NVFBC_BOOL cuda_load_library(void *libCUDA)
{
    libCUDA = dlopen(LIB_CUDA_NAME, RTLD_NOW);
    if (libCUDA == NULL) {
        fprintf(stderr, "Unable to open '%s'\n", LIB_CUDA_NAME);
        return NVFBC_FALSE;
    }

    cuInit_ptr = (CUINITPROC) dlsym(libCUDA, "cuInit");
    if (cuInit_ptr == NULL) {
        fprintf(stderr, "Unable to resolve symbol 'cuInit'\n");
        return NVFBC_FALSE;
    }

    cuDeviceGet_ptr = (CUDEVICEGETPROC) dlsym(libCUDA, "cuDeviceGet");
    if (cuDeviceGet_ptr == NULL) {
        fprintf(stderr, "Unable to resolve symbol 'cuDeviceGet'\n");
        return NVFBC_FALSE;
    }

    cuCtxCreate_v2_ptr = (CUCTXCREATEV2PROC) dlsym(libCUDA, "cuCtxCreate_v2");
    if (cuCtxCreate_v2_ptr == NULL) {
        fprintf(stderr, "Unable to resolve symbol 'cuCtxCreate_v2'\n");
        return NVFBC_FALSE;
    }

    cuMemcpyDtoH_v2_ptr = (CUMEMCPYDTOHV2PROC) dlsym(libCUDA, "cuMemcpyDtoH_v2");
    if (cuMemcpyDtoH_v2_ptr == NULL) {
        fprintf(stderr, "Unable to resolve symbol 'cuMemcpyDtoH_v2'\n");
        return NVFBC_FALSE;
    }

    return NVFBC_TRUE;
}

/**
 * Initializes CUDA and creates a CUDA context.
 *
 * \param [in] cuCtx
 *   A pointer to the created CUDA context.
 *
 * \return
 *   NVFBC_TRUE in case of success, NVFBC_FALSE otherwise.
 */
static NVFBC_BOOL cuda_init(CUcontext *cuCtx)
{
    CUresult cuRes;
    CUdevice cuDev;

    cuRes = cuInit_ptr(0);
    if (cuRes != CUDA_SUCCESS) {
        fprintf(stderr, "Unable to initialize CUDA (result: %d)\n", cuRes);
        return NVFBC_FALSE;
    }

    cuRes = cuDeviceGet_ptr(&cuDev, 0);
    if (cuRes != CUDA_SUCCESS) {
        fprintf(stderr, "Unable to get CUDA device (result: %d)\n", cuRes);
        return NVFBC_FALSE;
    }

    cuRes = cuCtxCreate_v2_ptr(cuCtx, CU_CTX_SCHED_AUTO, cuDev);
    if (cuRes != CUDA_SUCCESS) {
        fprintf(stderr, "Unable to create CUDA context (result: %d)\n", cuRes);
        return NVFBC_FALSE;
    }

    return NVFBC_TRUE;
}

// Bytes per pixel
static int Bpp = sizeof(uint8_t) * 3;
#define BITMAP_ROW_SIZE(width) (((width * Bpp) + 3) & ~3)
static PubSocket *pubSocket;

// static int fid = 0;
inline void send_frame(const unsigned char *frame, const uint32_t frameId, const int width, const int height){
    int size = BITMAP_ROW_SIZE(width) * height;
    // FILE * fd = fopen("myfile.bin", "wb");
    // fwrite(frame, size, 1, fd);
    MessageBuilder msg;
    auto framed = msg.initEvent().initRoadCameraState();
    framed.setFrameId(frameId);
    framed.setImage(kj::arrayPtr((const uint8_t *)frame, size));
    framed.setTransform({
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    });
    auto bytes = msg.toBytes();
    
    pubSocket->send((char *)bytes.begin(), bytes.size());
}

uint64_t NvFBCUtilsGetTimeInMillis()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return ((uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec) / 1000;
}

/**
 * Initializes the NvFBC and CUDA libraries and creates an NvFBC instance.
 *
 * Creates and sets up a capture session to video memory using the CUDA interop.
 *
 * Captures a bunch of frames every second, converts them to BMP and saves them
 * to the disk.
 */
int main(int argc, char *argv[])
{
    unsigned int i;
    NVFBC_SIZE frameSize = { 0, 0 };
    NVFBC_BOX captureBox = { 0, 0, 0, 0 };

    NVFBC_TRACKING_TYPE trackingType = NVFBC_TRACKING_DEFAULT;
    uint32_t outputId = 0;

    void *libNVFBC = NULL, *libCUDA = NULL;
    PNVFBCCREATEINSTANCE NvFBCCreateInstance_ptr = NULL;
    NVFBC_API_FUNCTION_LIST pFn;

    CUcontext cuCtx;

    NVFBCSTATUS fbcStatus;
    NVFBC_BOOL fbcBool;

    NVFBC_SESSION_HANDLE fbcHandle;
    NVFBC_CREATE_HANDLE_PARAMS createHandleParams;
    NVFBC_GET_STATUS_PARAMS statusParams;
    NVFBC_CREATE_CAPTURE_SESSION_PARAMS createCaptureParams;
    NVFBC_DESTROY_CAPTURE_SESSION_PARAMS destroyCaptureParams;
    NVFBC_DESTROY_HANDLE_PARAMS destroyHandleParams;
    NVFBC_TOCUDA_SETUP_PARAMS setupParams;

    NVFBC_BUFFER_FORMAT bufferFormat = NVFBC_BUFFER_FORMAT_RGB;

    // crop & scale to 1164x874
    frameSize.w = 1164;
    frameSize.h = 874;

    // frameSize.w = 1920;
    // frameSize.h = 1080;

    /*
     * Open X connection to retrieve the size of the framebuffer.
     */
    Display *dpy = XOpenDisplay(NULL);
    if (dpy == NULL) {
        fprintf(stderr, "Unable to open display\n");
        return EXIT_FAILURE;
    }

    unsigned int framebufferWidth  = DisplayWidth(dpy, XDefaultScreen(dpy));
    unsigned int framebufferHeight = DisplayHeight(dpy, XDefaultScreen(dpy));

    double croppedWidth = (double) frameSize.w / frameSize.h * framebufferHeight;
    int cropLen = (framebufferWidth - croppedWidth)/2;

    // printf("Cropped len = %d\n", cropLen);

    //crop off left and right side that doesn't match the target aspect ratio
    captureBox.h = framebufferHeight;
    captureBox.w = (int) croppedWidth;
    captureBox.x = cropLen;
    captureBox.y = 0;

    /*
     * Dynamically load the NvFBC library.
     */
    libNVFBC = dlopen(LIB_NVFBC_NAME, RTLD_NOW);
    if (libNVFBC == NULL) {
        fprintf(stderr, "Unable to open '%s'\n", LIB_NVFBC_NAME);
        return EXIT_FAILURE;
    }

    fbcBool = cuda_load_library(libCUDA);
    if (fbcBool != NVFBC_TRUE) {
        return EXIT_FAILURE;
    }

    fbcBool = cuda_init(&cuCtx);
    if (fbcBool != NVFBC_TRUE) {
        return EXIT_FAILURE;
    }

    /*
     * Resolve the 'NvFBCCreateInstance' symbol that will allow us to get
     * the API function pointers.
     */
    NvFBCCreateInstance_ptr =
        (PNVFBCCREATEINSTANCE) dlsym(libNVFBC, "NvFBCCreateInstance");
    if (NvFBCCreateInstance_ptr == NULL) {
        fprintf(stderr, "Unable to resolve symbol 'NvFBCCreateInstance'\n");
        return EXIT_FAILURE;
    }

    /*
     * Create an NvFBC instance.
     *
     * API function pointers are accessible through pFn.
     */
    memset(&pFn, 0, sizeof(pFn));

    pFn.dwVersion = NVFBC_VERSION;

    fbcStatus = NvFBCCreateInstance_ptr(&pFn);
    if (fbcStatus != NVFBC_SUCCESS) {
        fprintf(stderr, "Unable to create NvFBC instance (status: %d)\n",
                fbcStatus);
        return EXIT_FAILURE;
    }

    /*
     * Create a session handle that is used to identify the client.
     */
    memset(&createHandleParams, 0, sizeof(createHandleParams));

    createHandleParams.dwVersion = NVFBC_CREATE_HANDLE_PARAMS_VER;

    fbcStatus = pFn.nvFBCCreateHandle(&fbcHandle, &createHandleParams);
    if (fbcStatus != NVFBC_SUCCESS) {
        fprintf(stderr, "%s\n", pFn.nvFBCGetLastErrorStr(fbcHandle));
        return EXIT_FAILURE;
    }

    /*
     * Get information about the state of the display driver.
     *
     * This call is optional but helps the application decide what it should
     * do.
     */
    memset(&statusParams, 0, sizeof(statusParams));

    statusParams.dwVersion = NVFBC_GET_STATUS_PARAMS_VER;

    fbcStatus = pFn.nvFBCGetStatus(fbcHandle, &statusParams);
    if (fbcStatus != NVFBC_SUCCESS) {
        fprintf(stderr, "%s\n", pFn.nvFBCGetLastErrorStr(fbcHandle));
        return EXIT_FAILURE;
    }

    if (statusParams.bCanCreateNow == NVFBC_FALSE) {
        fprintf(stderr, "It is not possible to create a capture session "
                        "on this system.\n");
        return EXIT_FAILURE;
    }

    /*
     * Create a capture session.
     */
    memset(&createCaptureParams, 0, sizeof(createCaptureParams));

    createCaptureParams.dwVersion     = NVFBC_CREATE_CAPTURE_SESSION_PARAMS_VER;
    createCaptureParams.eCaptureType  = NVFBC_CAPTURE_SHARED_CUDA;
    createCaptureParams.bWithCursor   = NVFBC_TRUE;
    createCaptureParams.frameSize     = frameSize; // scale
    createCaptureParams.captureBox    = captureBox; // crop
    createCaptureParams.eTrackingType = trackingType;

    if (trackingType == NVFBC_TRACKING_OUTPUT) {
        createCaptureParams.dwOutputId = outputId;
    }

    fbcStatus = pFn.nvFBCCreateCaptureSession(fbcHandle, &createCaptureParams);
    if (fbcStatus != NVFBC_SUCCESS) {
        fprintf(stderr, "%s\n", pFn.nvFBCGetLastErrorStr(fbcHandle));
        return EXIT_FAILURE;
    }

    /*
     * Set up the capture session.
     */
    memset(&setupParams, 0, sizeof(setupParams));

    setupParams.dwVersion     = NVFBC_TOCUDA_SETUP_PARAMS_VER;
    setupParams.eBufferFormat = bufferFormat;

    fbcStatus = pFn.nvFBCToCudaSetUp(fbcHandle, &setupParams);
    if (fbcStatus != NVFBC_SUCCESS) {
        fprintf(stderr, "%s\n", pFn.nvFBCGetLastErrorStr(fbcHandle));
        return EXIT_FAILURE;
    }

    pubSocket = PubSocket::create();
    pubSocket->connect(Context::create(), "roadCameraState");

    // struct timespec delay = {0, 50*1000000};
    /*
     * We are now ready to start grabbing frames.
     */
    static unsigned char *frame = NULL;
    for (i = 0; ; i++) {
        static CUdeviceptr cuDevicePtr;
        static uint32_t lastByteSize = 0;

        uint64_t t1, t2, t1_total, t2_total, t_delta, wait_time_ms;

        CUresult cuRes;

        NVFBC_TOCUDA_GRAB_FRAME_PARAMS grabParams;

        NVFBC_FRAME_GRAB_INFO frameInfo;

        t1 = NvFBCUtilsGetTimeInMillis();
        t1_total = t1;

        memset(&grabParams, 0, sizeof(grabParams));
        memset(&frameInfo, 0, sizeof(frameInfo));

        grabParams.dwVersion = NVFBC_TOCUDA_GRAB_FRAME_PARAMS_VER;

        /*
         * Use asynchronous calls.
         *
         * The application will not wait for a new frame to be ready.  It will
         * capture a frame that is already available.  This might result in
         * capturing several times the same frame.  This can be detected by
         * checking the frameInfo.bIsNewFrame structure member.
         */
        grabParams.dwFlags = NVFBC_TOCUDA_GRAB_FLAGS_NOWAIT;

        /*
         * This structure will contain information about the captured frame.
         */
        grabParams.pFrameGrabInfo = &frameInfo;

        /*
         * The frame will be mapped in video memory through this CUDA
         * device pointer.
         */
        grabParams.pCUDADeviceBuffer = &cuDevicePtr;

        /*
         * Capture a frame.
         */
        fbcStatus = pFn.nvFBCToCudaGrabFrame(fbcHandle, &grabParams);
        if (fbcStatus != NVFBC_SUCCESS) {
            fprintf(stderr, "%s\n", pFn.nvFBCGetLastErrorStr(fbcHandle));
            return EXIT_FAILURE;
        }

        t2 = NvFBCUtilsGetTimeInMillis();

        /*
         * Allocate or re-allocate the destination buffer in system memory
         * when necessary.
         *
         * This is to handle change of resolution.
         */
        if (lastByteSize < frameInfo.dwByteSize) {
            frame = (unsigned char *)realloc(frame, frameInfo.dwByteSize);
            if (frame == NULL) {
                fprintf(stderr, "Unable to allocate system memory\n");
                return EXIT_FAILURE;
            }

            printf("Reallocated %u KB of system memory\n",
                   frameInfo.dwByteSize / 1024);

            lastByteSize = frameInfo.dwByteSize;
        }

        // printf("%s id %u grabbed in %llu ms",
        //        (frameInfo.bIsNewFrame ? "Frame" : "frame"),
        //        frameInfo.dwCurrentFrame,
        //        (unsigned long long) (t2 - t1));

        /*
         * Download frame from video memory to system memory.
         */
        t1 = NvFBCUtilsGetTimeInMillis();

        cuRes = cuMemcpyDtoH_v2_ptr((void *) frame, cuDevicePtr,
                                    frameInfo.dwByteSize);
        if (cuRes != CUDA_SUCCESS) {
            fprintf(stderr, "CUDA memcpy failure (result: %d)\n", cuRes);
            return EXIT_FAILURE;
        }

        t2 = NvFBCUtilsGetTimeInMillis();

        // printf(", downloaded in %llu ms", (unsigned long long) (t2 - t1));

        send_frame(frame, frameInfo.dwCurrentFrame, frameInfo.dwWidth, frameInfo.dwHeight);

        /*
         * Convert RGB frame to BMP and save it on the disk.
         *
         * This operation can be quite slow.
         */
        // t1 = NvFBCUtilsGetTimeInMillis();

        // sprintf(filename, "frame%u.bmp", frameInfo.dwCurrentFrame);

        // res = NvFBCUtilsSaveFrame(bufferFormat, filename, frame,
        //                           frameInfo.dwWidth, frameInfo.dwHeight);
        // if (res > 0) {
        //     fprintf(stderr, "Unable to save frame\n");
        //     return EXIT_FAILURE;
        // }

        // t2 = NvFBCUtilsGetTimeInMillis();

        // printf(", saved in %llu ms\n", (unsigned long long) (t2 - t1));

        t2_total = t2;

        /*
         * Compute how much time to sleep before capturing the next frame.
         */
        t_delta = t2_total - t1_total;
        wait_time_ms = t_delta < 50 ? 50 - t_delta : 0;
        // printf(", now sleeping for %llu ms\n",
        //        (unsigned long long) wait_time_ms);
        if(wait_time_ms > 0){
            usleep(wait_time_ms * 1000);
        }
    }

    /*
     * Destroy capture session, tear down resources.
     */
    memset(&destroyCaptureParams, 0, sizeof(destroyCaptureParams));

    destroyCaptureParams.dwVersion = NVFBC_DESTROY_CAPTURE_SESSION_PARAMS_VER;

    fbcStatus = pFn.nvFBCDestroyCaptureSession(fbcHandle, &destroyCaptureParams);
    if (fbcStatus != NVFBC_SUCCESS) {
        fprintf(stderr, "%s\n", pFn.nvFBCGetLastErrorStr(fbcHandle));
        return EXIT_FAILURE;
    }

    /*
     * Destroy session handle, tear down more resources.
     */
    memset(&destroyHandleParams, 0, sizeof(destroyHandleParams));

    destroyHandleParams.dwVersion = NVFBC_DESTROY_HANDLE_PARAMS_VER;

    fbcStatus = pFn.nvFBCDestroyHandle(fbcHandle, &destroyHandleParams);
    if (fbcStatus != NVFBC_SUCCESS) {
        fprintf(stderr, "%s\n", pFn.nvFBCGetLastErrorStr(fbcHandle));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
