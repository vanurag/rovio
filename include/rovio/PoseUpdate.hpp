/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_POSEUPDATE_HPP_
#define ROVIO_POSEUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "gnuplot-iostream/gnuplot-iostream.h"

namespace rovio {

class PoseInnovation: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseInnovation(){
    static_assert(_att+1==E_,"Error with indices");
  };
  virtual ~PoseInnovation(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
  inline QPD& att(){
    return this->template get<_att>();
  }
  inline const QPD& att() const{
    return this->template get<_att>();
  }
};
class PoseUpdateMeas: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateMeas(){
    static_assert(_att+1==E_,"Error with indices");
  };
  virtual ~PoseUpdateMeas(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
  inline QPD& att(){
    return this->template get<_att>();
  }
  inline const QPD& att() const{
    return this->template get<_att>();
  }
};
class PoseUpdateNoise: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateNoise(){
    static_assert(_att+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_att>() = "att";
  };
  virtual ~PoseUpdateNoise(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
  inline V3D& att(){
    return this->template get<_att>();
  }
  inline const V3D& att() const{
    return this->template get<_att>();
  }
};
class PoseOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<PoseInnovation::template getId<PoseInnovation::_pos>(),6>>{
 public:
  virtual ~PoseOutlierDetection(){};
};

/** \brief Class for adding poseUpdates to the filter.
 *
 * Coordinate frames overview:
 * I: Inertial frame of measured pose
 * V: Body frame of measured pose
 * W: Inertial frame of ROVIO
 * M: IMU-coordinate frame
 *
 *  @tparam FILTERSTATE         - FilterState
 *  @tparam inertialPoseIndex   - Index where the estimated relative pose between inertial frames is stored. Set -1 if this should not be estimated.
 *  @tparam bodyPoseIndex       - Index where the estimated relative pose between body frames is stored. Set -1 if this should not be estimated.
 */
template<typename FILTERSTATE, int inertialPoseIndex, int bodyPoseIndex>
class PoseUpdate: public LWF::Update<PoseInnovation,FILTERSTATE,PoseUpdateMeas,PoseUpdateNoise,PoseOutlierDetection,false>{
 public:
  typedef LWF::Update<PoseInnovation,FILTERSTATE,PoseUpdateMeas,PoseUpdateNoise,PoseOutlierDetection,false> Base;
  using Base::eval;
  using Base::boolRegister_;
  using Base::intRegister_;
  using Base::doubleRegister_;
  using Base::meas_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  static constexpr int inertialPoseIndex_ = inertialPoseIndex;
  static constexpr int bodyPoseIndex_ = bodyPoseIndex;
  QPD qVM_;
  V3D MrMV_;
  QPD qWI_;
  V3D IrIW_;
  double timeOffset_;
  bool enablePosition_;
  bool enableAttitude_;
  bool noFeedbackToRovio_;
  bool doInertialAlignmentAtStart_;
  bool didAlignment_;
  Gnuplot gp;//("gnuplot -persist");
  std::vector<boost::tuple<double,double,double>> gp_pts_vio, gp_pts_mocap;
  std::vector<std::vector<boost::tuple<double, double, double>>> gp_segments;
  PoseUpdate() {
    static_assert(mtState::nPose_>inertialPoseIndex_,"Please add enough poses to the filter state (templated).");
    static_assert(mtState::nPose_>bodyPoseIndex_,"Please add enough poses to the filter state (templated).");
    qVM_.setIdentity();
    MrMV_.setZero();
    qWI_.setIdentity();
    IrIW_.setZero();
    timeOffset_ = 0.0;
    enablePosition_ = true;
    enableAttitude_ = true;
    noFeedbackToRovio_ = true;
    doInertialAlignmentAtStart_ = true;
    didAlignment_ = false;
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
    doubleRegister_.registerVector("IrIW",IrIW_);
    doubleRegister_.registerQuaternion("qWI",qWI_);
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
    doubleRegister_.registerScalar("timeOffset",timeOffset_);
    boolRegister_.registerScalar("enablePosition",enablePosition_);
    boolRegister_.registerScalar("enableAttitude",enableAttitude_);
    boolRegister_.registerScalar("noFeedbackToRovio",noFeedbackToRovio_);
    boolRegister_.registerScalar("doInertialAlignmentAtStart",doInertialAlignmentAtStart_);

    // plotting
    gp << "set xrange [-2:2]\n";
    gp << "set yrange [-2:2]\n";
    gp << "set zrange [0:3]\n";
    gp << "set hidden3d nooffset\n";
//    gp << "set multiplot\n";

    gp << "splot ";
//    std::vector<std::vector<boost::tuple<double,double,double> > > pts(2);
//    pts[0].resize(1);
//    pts[1].resize(1);
//    pts[0][0] = boost::make_tuple(1.0, 1.0, 1.0);
//    pts[1][0] = boost::make_tuple(-1.0, -1.0, 1.0);
//    gp << gp.binFile2d(pts, "record") << "with lines title 'vec of vec of boost::tuple'";
//    gp << std::endl;
  }
  virtual ~PoseUpdate(){}
  const V3D& get_IrIW(const mtState& state) const{
    if(inertialPoseIndex_ >= 0){
      return state.poseLin(inertialPoseIndex_);
    } else {
      return IrIW_;
    }
  }
  const QPD& get_qWI(const mtState& state) const{
    if(inertialPoseIndex_ >= 0){
      return state.poseRot(inertialPoseIndex_);
    } else {
      return qWI_;
    }
  }
  const V3D& get_MrMV(const mtState& state) const{
    if(bodyPoseIndex_ >= 0){
      return state.poseLin(bodyPoseIndex_);
    } else {
      return MrMV_;
    }
  }
  const QPD& get_qVM(const mtState& state) const{
    if(bodyPoseIndex_ >= 0){
      return state.poseRot(bodyPoseIndex_);
    } else {
      return qVM_;
    }
  }
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    // IrIV = IrIW + qWI^T*(WrWM + qWM*MrMV)
    // qVI = qVM*qWM^T*qWI
    if(enablePosition_){
      y.pos() = get_IrIW(state) + get_qWI(state).inverseRotate(V3D(state.WrWM()+state.qWM().rotate(get_MrMV(state)))) - meas_.pos() + noise.pos();
    } else {
      y.pos() = noise.pos();
    }
    if(enableAttitude_){
      QPD attNoise = attNoise.exponentialMap(noise.att());
      y.att() = attNoise*get_qVM(state)*state.qWM().inverted()*get_qWI(state)*meas_.att().inverted();
    } else {
      QPD attNoise = attNoise.exponentialMap(noise.att());
      y.att() = attNoise;
    }
  }
  void jacState(MXD& F, const mtState& state) const{
    F.setZero();
    if(enablePosition_){
      if(!noFeedbackToRovio_){
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) =
            MPD(get_qWI(state).inverted()).matrix();
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_att>()) =
            MPD(get_qWI(state).inverted()).matrix()*gSM(state.qWM().rotate(get_MrMV(state)));
      }
      if(inertialPoseIndex_ >= 0){
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pop>(inertialPoseIndex_)) =
            M3D::Identity();
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_poa>(inertialPoseIndex_)) =
            -gSM(get_qWI(state).inverseRotate(V3D(state.WrWM()+state.qWM().rotate(get_MrMV(state)))))*MPD(get_qWI(state).inverted()).matrix();
      }
      if(bodyPoseIndex_ >= 0){
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pop>(bodyPoseIndex_)) =
            MPD(get_qWI(state).inverted()*state.qWM()).matrix();
      }
    }
    if(enableAttitude_){
      if(!noFeedbackToRovio_){
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_att>()) =
            -MPD(get_qVM(state)*state.qWM().inverted()).matrix();
      }
      if(inertialPoseIndex_ >= 0){
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_poa>(inertialPoseIndex_)) =
            MPD(get_qVM(state)*state.qWM().inverted()).matrix();
      }
      if(bodyPoseIndex_ >= 0){
        F.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_poa>(bodyPoseIndex_)) =
            M3D::Identity();
      }
    }
  }
  void jacNoise(MXD& G, const mtState& state) const{
    G.setZero();
    G.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity();
    G.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtNoise::template getId<mtNoise::_att>()) = M3D::Identity();
  }
  void preProcess(mtFilterState& filterstate, const mtMeas& meas, bool& isFinished){
    mtState& state = filterstate.state_;
    isFinished = false;
    if(!didAlignment_ && doInertialAlignmentAtStart_){
      // qWI = qWM*qVM^T*qVI;
      std::cout << "check:" << std::endl;
      std::cout << MPD(state.qWM()).matrix() << std::endl;
      std::cout << MPD(get_qVM(state)).matrix() << std::endl;
      std::cout << MPD(meas.att()).matrix() << std::endl;
      std::cout << "flag1" << std::endl;
      qWI_ = state.qWM()*get_qVM(state).inverted()*meas.att();
      std::cout << "flag2" << std::endl;
      std::cout << MPD(qWI_).matrix() << std::endl;
      if(inertialPoseIndex_ >= 0){
        state.poseRot(inertialPoseIndex_) = qWI_;
      }
      // IrIW = IrIV - qWI^T*(WrWM + qWM*MrMV);
      std::cout << "check2:" << std::endl;
      std::cout << meas.pos() << std::endl;
      std::cout << state.WrWM() << std::endl;
      std::cout << get_MrMV(state) << std::endl;
      IrIW_ = meas.pos() - qWI_.inverseRotate(V3D(state.WrWM() + state.qWM().rotate(get_MrMV(state))));
      std::cout << IrIW_ << std::endl;
      if(inertialPoseIndex_ >= 0){
        state.poseLin(inertialPoseIndex_) = IrIW_;
      }
      didAlignment_ = true;
    }
  }
  void postProcess(mtFilterState& filterstate, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    mtState& state = filterstate.state_;
    isFinished = true;
    // WrWC = qWI*(IrIV - qWI^T*qWM*MrMV -IrIW) +qWM*MrMC
    state.aux().poseMeasLin_ = get_qWI(state).rotate(V3D(meas.pos()-(get_qWI(state).inverted()*state.qWM()).rotate(get_MrMV(state))-get_IrIW(state)))+state.template get<mtState::_att>().rotate(state.MrMC(0));
    // qCW = qCM*qVM^T*qVI*qWI^T;
    state.aux().poseMeasRot_ = state.qCM(0)*get_qVM(state).inverted()*meas.att()*get_qWI(state).inverted();

    // plotting
    V3D tIC_vio = get_qWI(state).inverted().rotate(state.aux().poseMeasLin_) + get_IrIW(state);
    QPD qIC_vio = get_qWI(state).inverted()*state.aux().poseMeasRot_.inverted();
    V3D tIC_mocap = meas.att().inverted().rotate(get_qVM(state).rotate(V3D(state.MrMC(0) - get_MrMV(state)))) + meas.pos();
    QPD qIC_mocap = meas.att().inverted()*get_qVM(state)*state.qCM(0).inverted();
    gp_pts_vio.push_back(boost::make_tuple(tIC_vio.x(), tIC_vio.y(), tIC_vio.z()));
    gp_pts_mocap.push_back(boost::make_tuple(tIC_mocap.x(), tIC_mocap.y(), tIC_mocap.z()));
    std::vector<boost::tuple<double, double, double>> segment;
    segment.push_back(boost::make_tuple(tIC_vio.x(), tIC_vio.y(), tIC_vio.z()));
    segment.push_back(boost::make_tuple(tIC_mocap.x(), tIC_mocap.y(), tIC_mocap.z()));
    gp_segments.push_back(segment);

    gp << "splot" << gp.file2d(gp_segments) << "with lines linecolor rgb '#000000' title 'error',"
                  << gp.file1d(gp_pts_vio) << "with points linecolor rgb '#ff0000' pointtype 5 pointsize 0.2 title 'ROVIO',"
                  << gp.file1d(gp_pts_mocap) << "with points linecolor rgb '#0000ff' pointtype 7 pointsize 0.2 title 'MOCAP'" << std::endl;
//    gp << "replot\n";
  }
};

}


#endif /* ROVIO_POSEUPDATE_HPP_ */
