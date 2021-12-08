/*
Copyright (c) 2020 The Linux Foundation. All rights reserved.

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

package com.android.camera.aide;

import java.nio.ByteBuffer;
import android.util.Log;
import android.os.SystemClock;
import com.android.camera.CameraActivity;
import com.android.camera.CaptureModule;
import android.util.Size;
import android.hardware.camera2.TotalCaptureResult;
import android.graphics.Rect;

public class AideUtil {
    private static final String TAG = "SnapCam_AideUtil";
    private static boolean mIsSupported = false;
    private static boolean mIsSupported2 = false;

    public AideUtil() {}

    public native int nativeAIDenoiserEngineCreate(int[] pInputFrameDim, int[] pOutputFrameDim);

    public native int nativeAIDenoiserEngineProcessFrame(byte[] input, byte[] output,
        long expTimeInNs, int iso, float denoiseStrength, int rGain, int bGain, int gGain, int[] roi);

    public native int nativeAIDenoiserEngineAbort();

    public native int nativeAIDenoiserEngineDestroy();

    public native int nativeAIDenoiserEngineCreateV2(int[] pInputFrameDim, int[] pDsInputFrameDim, int[] pOutputFrameDim, int imageformat, int mode);

    public native int nativeAIDenoiserEngineProcessFrameV2(ByteBuffer inputY, ByteBuffer inputC, ByteBuffer dsinputY, ByteBuffer dsinput, byte[] output,
        long expTimeInNs, int iso, float denoiseStrength, float adrcGain, int rGain, int bGain, int gGain, int[] roi);

    public native int nativeAIDenoiserEngineAbortV2();

    public native int nativeAIDenoiserEngineDestroyV2();

    public native int nativeCvtYuvToRgb(byte[] input, byte[] output, int width, int height, int stride, int type);

    static {
        try {
            System.loadLibrary("jni_aidenoiserutil");
            Log.i(TAG, "load jni_aidenoiserutil successfully");
            mIsSupported = true;
        } catch (UnsatisfiedLinkError e) {
            mIsSupported = false;
            Log.d(TAG, e.toString());
        }
        try {
            System.loadLibrary("jni_aidenoiserutilv2");
            Log.i(TAG, "load libjni_aidenoiserutilv2 successfully");
            mIsSupported2 = true;
        } catch (UnsatisfiedLinkError e) {
            Log.d(TAG, e.toString());
        }
    }

    public static boolean isAideSupported(){
        return mIsSupported;
    }

    public static boolean isAide2Supported(){
        return mIsSupported2;
    }

    public static class AIDEFrameDim {
        int width;
        int height;
        int stride;
    }

    public static class AIDEROI {
        int x;
        int y;
        int width;
        int height;
    }

    public static class AIDEProcessFrameArgs {
        byte[] pInputLuma; // Image input luma plane pointer
        byte[] pInputChroma; // Image input chroma plane pointer
        byte[] pOutputLuma; // Image output luma plane pointer
        byte[] pOutputChroma; // Image output chroma plane pointer
        long expTimeInNs; // Exposure Time in nanoseconds - range: 1/100 - 1s
        int iso; // ISO = (real_gain * 100) / iso_100_gain
        float denoiseStrength; // Denoising strength: 0.0-1.0 (default = 0.5) tuning param
        int rGain; // R gain from 3A
        int bGain; // B gain from 3A
        int gGain; // G gain from 3A
        int[] reservedData; // Placeholder for future parameters
        AIDEROI roi; // Crop roi data

        public AIDEProcessFrameArgs(byte[] pInputLuma, byte[] pInputChroma, long expTimeInNs, int iso, float denoiseStrength, int rGain, int bGain, int gGain) {
            this.pInputLuma = pInputLuma;
            this.pInputChroma = pInputChroma;
            this.expTimeInNs = expTimeInNs;
            this.iso = iso;
            this.denoiseStrength = denoiseStrength;
            this.rGain = rGain;
            this.bGain = bGain;
            this.gGain = gGain;
        }
    }

    public static class AIDEV2ProcessFrameArgs {
        int[] inputFrameDim;
        int[] downFrameDim;
        ByteBuffer srcInputY;
        ByteBuffer srcInputUV;
        ByteBuffer srcDsInputY;
        ByteBuffer srcDsInputUV;
        String title;
        Rect cropRegion;
        TotalCaptureResult captureResult;
        Size pictureSize;
        float denoiseStrengthParam;
        float adrcGain;
        int rGain;
        int bGain;
        int gGain;
        int orientation;
        int quality;

        public AIDEV2ProcessFrameArgs(int[] inputFrameDim, int[] downFrameDim, ByteBuffer srcInputY, ByteBuffer srcInputUV, ByteBuffer srcDsInputY, ByteBuffer srcDsInputUV,
                String title, Rect cropRegion, TotalCaptureResult captureResult, Size pictureSize, float denoiseStrengthParam, float adrcGain, int rGain, int bGain, int gGain, int orientation, int quality) {
            this.inputFrameDim = inputFrameDim;
            this.downFrameDim = downFrameDim;
            this.srcInputY = srcInputY;
            this.srcInputUV = srcInputUV;
            this.srcDsInputY = srcDsInputY;
            this.srcDsInputUV = srcDsInputUV;
            this.title = title;
            this.cropRegion = cropRegion;
            this.captureResult = captureResult;
            this.pictureSize = pictureSize;
            this.denoiseStrengthParam = denoiseStrengthParam;
            this.adrcGain = adrcGain;
            this.rGain = rGain;
            this.bGain = bGain;
            this.gGain = gGain;
            this.orientation = orientation;
            this.quality = quality;
        }

        public int[] getInputFrameDim(){
            return inputFrameDim;
        }

        public int[] getdownFrameDim(){
            return downFrameDim;
        }

        public ByteBuffer getsrcInputY(){
            return srcInputY;
        }

        public ByteBuffer getsrcInputUV(){
            return srcInputUV;
        }

        public ByteBuffer getsrcDsInputY(){
            return srcDsInputY;
        }

        public ByteBuffer getsrcDsInputUV(){
            return srcDsInputUV;
        }

        public String gettitle(){
            return title;
        }

        public Rect getcropRegion(){
            return cropRegion;
        }

        public TotalCaptureResult getcaptureResult(){
            return captureResult;
        }

        public Size getpictureSize(){
            return pictureSize;
        }

        public float getdenoiseStrengthParam(){
            return denoiseStrengthParam;
        }

        public float getadrcGain(){
            return adrcGain;
        }

        public int getrGain(){
            return rGain;
        }
        public int getbGain(){
            return bGain;
        }

        public int getgGain(){
            return gGain;
        }

        public int getorientation(){
            return orientation;
        }

        public int getquality(){
            return quality;
        }
    }
}