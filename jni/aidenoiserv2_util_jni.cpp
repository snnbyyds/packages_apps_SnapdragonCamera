/*
Copyright (c) 2021, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <jni.h>
#include <assert.h>
#include <stdlib.h>
#include <dlfcn.h>
#include "aidenoiserv2/aidenoiserenginev2.h"

#ifdef __ANDROID__
#include "android/log.h"
#define printf(...) __android_log_print( ANDROID_LOG_ERROR, "aidev2jni", __VA_ARGS__ )
#endif

#ifdef __cplusplus
extern "C" {
#endif
JNIEXPORT jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineCreateV2(
        JNIEnv* env, jobject thiz, jintArray pInputFrameDim, jintArray pDsInputFrameDim, jintArray pOutputFrameDim,jint imageformat, jint mode);
JNIEXPORT jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineProcessFrameV2(
        JNIEnv *env, jobject thiz, jobjectArray inputY,jobjectArray inputC, jobjectArray dsinputY, jobjectArray dsinputC,jbyteArray output,
        jlong expTimeInNs, jint iso, jfloat denoiseStrength, jfloat adrcGain, jint rGain, jint bGain, jint gGain, jintArray roi);
JNIEXPORT jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineAbortV2(
        JNIEnv* env, jobject thiz);
JNIEXPORT jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineDestroyV2(
        JNIEnv* env, jobject thiz);
#ifdef __cplusplus
}
#endif

AIDE_Handle handle;
uint32_t width;
uint32_t height;
uint32_t stride;
uint32_t dsWidth;
uint32_t dsHeight;
uint32_t dsStride;

jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineCreateV2(
        JNIEnv* env, jobject thiz, jintArray pInputFrameDim, jintArray pDsInputFrameDim, jintArray pOutputFrameDim,jint imageformat, jint mode)
{
    jint *inputFrameDim = env->GetIntArrayElements(pInputFrameDim, 0);
    AIDE_FrameDim _pInputFrameDim;
    _pInputFrameDim.width = (uint32_t)inputFrameDim[0];
    _pInputFrameDim.height = (uint32_t)inputFrameDim[1];
    _pInputFrameDim.heightStride = (uint32_t)inputFrameDim[2];
    _pInputFrameDim.widthStride = (uint32_t)inputFrameDim[3];
    jint *dsinputFrameDim = env->GetIntArrayElements(pDsInputFrameDim, 0);
    AIDE_FrameDim _pDsInputFrameDim;
    _pDsInputFrameDim.width = (uint32_t)dsinputFrameDim[0];
    _pDsInputFrameDim.height = (uint32_t)dsinputFrameDim[1];
    _pDsInputFrameDim.heightStride = (uint32_t)dsinputFrameDim[2];
    _pDsInputFrameDim.widthStride = (uint32_t)dsinputFrameDim[3];
    width = (uint32_t)inputFrameDim[0];
    height = (uint32_t)inputFrameDim[1];
    stride = (uint32_t)inputFrameDim[2];
    dsWidth = (uint32_t)dsinputFrameDim[0];
    dsHeight = (uint32_t)dsinputFrameDim[1];
    dsStride = (uint32_t)dsinputFrameDim[2];
    jint *outputFrameDim = env->GetIntArrayElements(pOutputFrameDim, 0);
    AIDE_FrameDim _outputFrameDim;
    _outputFrameDim.width = (uint32_t)outputFrameDim[0];
    _outputFrameDim.height = (uint32_t)outputFrameDim[1];
    _outputFrameDim.heightStride = (uint32_t)outputFrameDim[2];
    _outputFrameDim.widthStride = (uint32_t)inputFrameDim[3];

    AIDE_ImgFormat format_input = AIDE_NV21;
    if((uint32_t)imageformat == 0){
        format_input = AIDE_NV12;
    }
    AIDE_Mode mode_input = AIDE1;
    if((uint32_t)mode == 1){
        mode_input = AIDE2;
    }else if ((uint32_t)mode == 2){
        mode_input = AIDE2_SAT;
    }
    printf("aide create,mode_input=%d,format_input=%d", mode_input, format_input);
    int createResult = AIDenoiserEngine_Create(&_pInputFrameDim, &_pDsInputFrameDim, &_outputFrameDim, format_input, mode_input, &handle);
    printf("aide create,createResult=%d,handle=%d", createResult, handle);

    //set out put
    uint32_t* pOutputFrame = new uint32_t[4];
    pOutputFrame[0] = _outputFrameDim.width;
    pOutputFrame[1] = _outputFrameDim.height;
    pOutputFrame[2] = _outputFrameDim.heightStride;
    pOutputFrame[3] = _outputFrameDim.widthStride;

    printf("aide create,width=%d,height=%d,heightStride=%d,widthStride=%d", _outputFrameDim.width,_outputFrameDim.height,_outputFrameDim.heightStride,_outputFrameDim.widthStride);

    env->SetIntArrayRegion(pOutputFrameDim, 0, 4, (jint *)pOutputFrame);
    env->ReleaseIntArrayElements(pInputFrameDim, inputFrameDim, 0);
    env->ReleaseIntArrayElements(pOutputFrameDim, outputFrameDim, 0);
    delete [] pOutputFrame;
    return 0;
}

void WriteData(FILE *fp, unsigned char *pStart, int width, int height, int stride)
{
    for (int i = 0; i < height; i++)
    {
        fwrite(pStart, stride, 1, fp);
        pStart += stride;
    }
}

jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineProcessFrameV2(
        JNIEnv *env, jobject thiz, jobjectArray inputY,jobjectArray inputC, jobjectArray dsinputY, jobjectArray dsinputC, jbyteArray output,
        jlong expTimeInNs, jint iso, jfloat denoiseStrength, jfloat adrcGain, jint rGain, jint bGain, jint gGain, jintArray roi)
{
    AIDE_ProcessFrameArgs args;
    args.rGain = (uint32_t)rGain;
    args.gGain = (uint32_t)gGain;
    args.bGain = (uint32_t)bGain;
    jint *croi = env->GetIntArrayElements(roi, NULL);
    args.roi.x = (uint32_t)croi[0];
    args.roi.y = (uint32_t)croi[1];
    args.roi.width = (uint32_t)croi[2];
    args.roi.height = (uint32_t)croi[3];
    args.denoiseStrength = (float)denoiseStrength;
    args.adrcGain = (float)adrcGain;
    args.iso = (uint32_t)iso;
    args.expTimeInNs = (uint64_t)expTimeInNs;

    uint8_t *cinputY = (uint8_t *)env->GetDirectBufferAddress(inputY);
    uint8_t *cinputVU = (uint8_t *)env->GetDirectBufferAddress(inputC);
    uint8_t *cdsinputY = (uint8_t *)env->GetDirectBufferAddress(dsinputY);
    uint8_t *cdsinputVU = (uint8_t *)env->GetDirectBufferAddress(dsinputC);
    jbyte* outputArray = env->GetByteArrayElements(output, NULL);
    uint8_t *coutput = (uint8_t*)outputArray;
    uint8_t* coutputY = (uint8_t*)&(coutput[0]);
    uint8_t* coutputVU = (uint8_t*)&(coutput[stride*height]);
    FILE *inputFile = fopen("/data/data/org.codeaurora.snapcam/files/aidefullinput.yuv", "wb+");
    if ((inputFile != NULL)){
        WriteData(inputFile, cinputY, width, height, stride);
        WriteData(inputFile, cinputVU, width, height/2, stride);
        fclose(inputFile);
    } else {
        printf( "aidefullinput is NULL");
    }
    FILE *inputDsFile = fopen("/data/data/org.codeaurora.snapcam/files/AideDsInput.yuv", "wb+");
    if ((inputDsFile != NULL)){
        WriteData(inputDsFile, cdsinputY, dsWidth, dsHeight, dsStride);
        WriteData(inputDsFile, cdsinputVU, dsWidth, dsHeight/2, dsStride);
        fclose(inputDsFile);
    } else {
        printf( "AideDsInput is NULL");
    }
    args.pInputLuma = cinputY;
    args.pInputChroma = cinputVU;
    args.pDsInputLuma = cdsinputY;
    args.pDsInputChroma = cdsinputVU;
    args.pOutputLuma = coutputY;
    args.pOutputChroma = coutputVU;
    int result = AIDenoiserEngine_ProcessFrame(handle, &args, NULL);
    FILE *outputFile = fopen("/data/data/org.codeaurora.snapcam/files/aideoutput.yuv", "wb+");
    if ((outputFile != NULL)){
        WriteData(outputFile, coutputY, width, height, stride);
        WriteData(outputFile, coutputVU, width, height/2, stride);
        fclose(outputFile);
    } else {
        printf( "aideoutput is NULL");
    }
    //set out put
    env->ReleaseIntArrayElements(roi, croi, 0);
    return result;
}

jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineAbortV2(
        JNIEnv* env, jobject thiz)
{
    return AIDenoiserEngine_Abort(handle);
}

jint JNICALL Java_com_android_camera_aide_AideUtil_nativeAIDenoiserEngineDestroyV2(
        JNIEnv* env, jobject thiz)
{
    return AIDenoiserEngine_Destroy(handle);
}