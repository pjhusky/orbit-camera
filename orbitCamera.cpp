#include "orbitCamera.h"

#include "../math/linAlg.h" // remove this dependency eventually

#ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <numbers>

namespace {
    constexpr static float practicallyZero = 0.00001f;
    constexpr static float mouseSensitivityInit = 0.23f;

    static void loadIdentityMatrix( OrbitCamera::rowMajorMat3x4_t& mat ) {
        mat[0] = OrbitCamera::rowVec4_t{ 1.0f, 0.0f, 0.0f, 0.0f };
        mat[1] = OrbitCamera::rowVec4_t{ 0.0f, 1.0f, 0.0f, 0.0f };
        mat[2] = OrbitCamera::rowVec4_t{ 0.0f, 0.0f, 1.0f, 0.0f };
    }
    
     //template<uint32_t num_T>
     //static OrbitCamera::vec_t<num_T>& operator*( OrbitCamera::vec_t<num_T>& vec, const float scalar ) {
     //    for ( auto& entry : vec ) {
     //        entry *= scalar;
     //    }
     //    return vec;
     //}
    template<std::size_t num_T>
    static inline OrbitCamera::vec_t<num_T> operator*( const OrbitCamera::vec_t<num_T>& vec, const float scalar ) {
        OrbitCamera::vec_t<num_T> result = vec;
        for ( auto& entry : result ) {
            entry *= scalar;
        }
        return result;
    }

    template<std::size_t num_T>
    static inline OrbitCamera::vec_t<num_T> operator+( const OrbitCamera::vec_t<num_T>& vecL, const OrbitCamera::vec_t<num_T>& vecR ) {
        OrbitCamera::vec_t<num_T> result = vecL;
        for ( uint32_t i = 0; i < num_T; i++ ) {
            result[i] += vecR[i];
        }
        return result;
    }

    template<std::size_t num_T>
    static inline OrbitCamera::vec_t<num_T> operator-( const OrbitCamera::vec_t<num_T>& vecL, const OrbitCamera::vec_t<num_T>& vecR ) {
        OrbitCamera::vec_t<num_T> result = vecL;
        for ( uint32_t i = 0; i < num_T; i++ ) {
            result[i] -= vecR[i];
        }
        return result;
    }

    template<std::size_t num_T>
    static inline float dot( const OrbitCamera::vec_t<num_T>& vecL, const OrbitCamera::vec_t<num_T>& vecR ) {
        float result = 0.0f;
        for ( uint32_t i = 0; i < num_T; i++ ) {
            result += vecL[i] * vecR[i];
        }
        return result;
    }
    
}

OrbitCamera::OrbitCamera() 
    : mLMBdown( false )
    , mRMBdown( false )
    , mIsActive( true ) {

    resetTrafos();
    //mPanVector = { 0.0f, 0.0f, 0.0f };

    setControlConfig( ControlConfig{ .invertY=1u } );

    setMouseSensitivity( mouseSensitivityInit );

    mCurrMouseX = 0.0f;
    mCurrMouseY = 0.0f;
    mPrevMouseX = mCurrMouseX;
    mPrevMouseY = mCurrMouseY;
}

OrbitCamera::Status_t OrbitCamera::update( 
    const float timeDelta,
    const float mouseX, const float mouseY, 
    const int32_t screenW, const int32_t screenH, 
    const bool LMBpressed, const bool RMBpressed, 
    const rowVec3_t& translationDelta
    ) {

    if (!mIsActive) { return Status_t::OK; }

    mCurrMouseX = mouseX;
    mCurrMouseY = mouseY;
    const float mouse_dx = (mCurrMouseX - mPrevMouseX) * mMouseSensitivity;
    const float mouse_dy = (mCurrMouseY - mPrevMouseY) * mMouseSensitivity;

    if (!mLMBdown && LMBpressed) {
        printf( "LMB pressed\n" );
        mLMBdown = true;
    }
    if (mLMBdown && !LMBpressed) {
        printf( "LMB released\n" );
        mLMBdown = false;
    }

    if (!mRMBdown && RMBpressed) {
        printf( "RMB pressed\n" );
        mRMBdown = true;
    }
    if (mRMBdown && !RMBpressed) {
        printf( "RMB released\n" );
        mRMBdown = false;
    }

    float damping = 0.7f;
    //mOrbitPivotWS = mOrbitPivotWS * damping + mTargetOrbitPivotWS * ( 1.0f - damping );
    mOrbitPivotWS = { 0.0f, 0.0f, 0.0f };

    mOrbitDist += translationDelta[2];


    rowVec3_t *const xAxis = reinterpret_cast< rowVec3_t *const >( &mViewOrbitRotMat[0] );
    rowVec3_t *const yAxis = reinterpret_cast< rowVec3_t *const >( &mViewOrbitRotMat[1] );
    rowVec3_t *const zAxis = reinterpret_cast< rowVec3_t *const >( &mViewOrbitRotMat[2] );

    auto camPosWS = mOrbitPivotWS + *zAxis * mOrbitDist;
    auto startCamPosWS = camPosWS;

    auto strafeAmount_LR = translationDelta[0];
    auto strafeAmount_UD = translationDelta[1];
    if ( mLMBdown ) {

        //{
        //    const float angleRad = static_cast<float>(std::numbers::pi) * mouse_dx / screenW;
        //    const float cosAngle = cosf( angleRad );
        //    const float sinAngle = sinf( angleRad );
        //    *xAxis = (currXAxis *  cosAngle) + (currZAxis * sinAngle);
        //    *zAxis = (currXAxis * -sinAngle) + (currZAxis * cosAngle);
        //    currXAxis = *xAxis;
        //    currZAxis = *zAxis;
        //}

        //{
        //    const float angleRad = static_cast<float>(std::numbers::pi) * mouse_dy / screenH;
        //    const float cosAngle = cosf( angleRad );
        //    const float sinAngle = sinf( angleRad );
        //    *yAxis = (currYAxis *  cosAngle) + (currZAxis * sinAngle);
        //    *zAxis = (currYAxis * -sinAngle) + (currZAxis * cosAngle);
        //    currYAxis = *yAxis;
        //    currZAxis = *zAxis;
        //}

        //camPosWS = camPosWS + currXAxis * ( mouse_dx + translationDelta[0] );
        //camPosWS = camPosWS + currYAxis * ( mouse_dy + translationDelta[1] );

        strafeAmount_LR += mouse_dx;
        strafeAmount_UD -= mouse_dy;

    } 

    camPosWS = camPosWS + *xAxis * strafeAmount_LR;
    camPosWS = camPosWS + *yAxis * strafeAmount_UD;

    if ( mRMBdown ) {

        const float angleRad = static_cast<float>( std::numbers::pi ) * -mouse_dx / screenW;
        const float cosAngle = cosf( angleRad );
        const float sinAngle = sinf( angleRad );
        *xAxis = ( *xAxis *  cosAngle ) + ( *yAxis * sinAngle );
        *yAxis = ( *xAxis * -sinAngle ) + ( *yAxis * cosAngle );
    }


    // now map camera back to orbit
    *zAxis = linAlg::normalizeRet( camPosWS - mOrbitPivotWS );
    //if (linAlg::len( camPosWS - startCamPosWS ) > 0.01f) {
    linAlg::cross( *xAxis, *yAxis, *zAxis );
    linAlg::normalize( *xAxis );
    linAlg::cross( *yAxis, *zAxis, *xAxis );
    linAlg::normalize( *yAxis );
    //}

    //mPosWS = mPosWS + ( currXAxis * translationDelta[0] ) + ( currYAxis * translationDelta[1] ) + ( currZAxis * translationDelta[2] );

    camPosWS = mOrbitPivotWS + *zAxis * mOrbitDist;

    mViewOrbitRotMat[0][3] = -dot( *xAxis, camPosWS ); 
    mViewOrbitRotMat[1][3] = -dot( *yAxis, camPosWS );
    mViewOrbitRotMat[2][3] = -dot( *zAxis, camPosWS );
    //mViewOrbitRotMat[0][3] = 0.0f; 
    //mViewOrbitRotMat[1][3] = 0.0f;
    //mViewOrbitRotMat[2][3] = 0.0f;

    linAlg::loadTranslationMatrix( mViewPivotOffsetMat, mTargetOrbitPivotWS * -1.0f );
    mViewMat = mViewOrbitRotMat * mViewPivotOffsetMat;
    
    linAlg::loadTranslationMatrix( mViewPivotOffsetMat, mTargetOrbitPivotWS ); // jumps, but has the offset
    //linAlg::loadTranslationMatrix( mViewPivotOffsetMat, { dot( *xAxis, mTargetOrbitPivotWS ), dot( *yAxis, mTargetOrbitPivotWS ), dot( *zAxis, mTargetOrbitPivotWS ) } ); // removes pivot offset
    mViewMat = mViewPivotOffsetMat * mViewMat;

    //linAlg::loadTranslationMatrix( mViewPivotOffsetMat, { -dot( *xAxis, camPosWS ), -dot( *yAxis, camPosWS ), -dot( *zAxis, camPosWS ) } ); // removes pivot offset
    //mViewMat = mViewPivotOffsetMat * mViewMat;

    //mViewMat[0][3] = -dot( *xAxis, camPosWS ); 
    //mViewMat[1][3] = -dot( *yAxis, camPosWS );
    //mViewMat[2][3] = -dot( *zAxis, camPosWS );

    linAlg::mat3x4_t pivotOffsetCompensationMatrix;
    linAlg::loadTranslationMatrix( pivotOffsetCompensationMatrix, mPanVector );

    mViewMat = pivotOffsetCompensationMatrix * mViewMat;


    mPrevMouseX = mCurrMouseX;
    mPrevMouseY = mCurrMouseY;

    return Status_t::OK;
}

void OrbitCamera::setViewMatrix( const rowMajorMat3x4_t& viewMatrix ) {
    resetTrafos();

    mViewMat = viewMatrix;

    linAlg::mat4_t viewMat4;
    linAlg::castMatrix( viewMat4, viewMatrix );
    linAlg::mat4_t invViewMat4;
    linAlg::inverse( invViewMat4, viewMat4 );
    rowVec3_t camPosWS = {invViewMat4[0][3], invViewMat4[1][3], invViewMat4[2][3] };

    // remove translational part
    //mViewOrbitRotMat[0][3] = 0.0f;
    //mViewOrbitRotMat[1][3] = 0.0f;
    //mViewOrbitRotMat[2][3] = 0.0f;
    mViewOrbitRotMat = mViewMat;


    //linAlg::loadTranslationMatrix( mViewPivotOffsetMat, camPosWS * -1.0f );
    mTargetOrbitPivotWS = { 0.0f, 0.0f, 0.0f };

    auto diffVec = mOrbitPivotWS - camPosWS;
    mOrbitDist = sqrtf( dot( diffVec, diffVec ) );
}

void OrbitCamera::setOrbitPivotWS( const rowVec3_t& orbitPivotWS ) {
    mTargetOrbitPivotWS = orbitPivotWS;// *-1.0f;
}

void OrbitCamera::addPanDelta( const rowVec3_t& delta ) { 
    mPanVector = mPanVector + delta; 
}

//void OrbitCamera::setPosition( const rowVec3_t& pos ) {
//    mPosWS = pos;
//
//    rowVec3_t *const xAxis = reinterpret_cast< rowVec3_t *const >( &mViewOrbitRotMat[0] );
//    rowVec3_t *const yAxis = reinterpret_cast< rowVec3_t *const >( &mViewOrbitRotMat[1] );
//    rowVec3_t *const zAxis = reinterpret_cast< rowVec3_t *const >( &mViewOrbitRotMat[2] );
//
//    rowVec3_t currXAxis = *xAxis;
//    rowVec3_t currYAxis = *yAxis;
//    rowVec3_t currZAxis = *zAxis;
//
//    mViewOrbitRotMat[0][3] = -dot( currXAxis, mPosWS ); 
//    mViewOrbitRotMat[1][3] = -dot( currYAxis, mPosWS );
//    mViewOrbitRotMat[2][3] = -dot( currZAxis, mPosWS );
//}

void OrbitCamera::resetTrafos() {

    loadIdentityMatrix( mViewMat );
    loadIdentityMatrix( mViewOrbitRotMat );
    loadIdentityMatrix( mViewPivotOffsetMat );

    //mPosWS = { 0.0f, 0.0f, 0.0f };
    mTargetOrbitPivotWS = { 0.0f, 0.0f, 0.0f };
    mOrbitDist = 10.0f;

    mPanVector = { 0.0f, 0.0f, 0.0f };

    mCurrMouseX = 0;
    mCurrMouseY = 0;
    mPrevMouseX = 0;
    mPrevMouseY = 0;
    
}
