#include "lidar_viewer/object_tracker.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace lidar_viewer;
namespace {
void Check(bool value,const char * message) {if (!value) throw std::runtime_error(message);}
algorithms::Detection D(float x,float y=0,int cls=3,float yaw=0,float score=.8f) {
  const int model_class=cls==3 ? 4 : cls==4 ? 3 : cls==2 ? 1 : cls==1 ? 2 : 0;
  const float box[]{x,y,.9f,cls==3 ? .7f : 4.f,cls==3 ? .6f : 2.f,1.8f,yaw,score,static_cast<float>(model_class)};
  return algorithms::DecodeCenterPointBox(box,0);
}
TrackingFrame F(std::uint64_t sequence,double time=-1,std::uint64_t session=1,bool live=false) {
  return {session,sequence,static_cast<std::int64_t>((time<0 ? 1+sequence*.1 : time)*1e9),live};
}
const ObjectTrack & Id(const std::vector<ObjectTrack> & tracks,std::uint64_t id) {
  const auto found=std::find_if(tracks.begin(),tracks.end(),[&](const auto & t){return t.id==id;});
  if (found==tracks.end()) throw std::runtime_error("Expected track ID is missing");return *found;
}
}
int main() {
  try {
    TrackingParams params;ObjectTracker tracker;
    const auto initial=tracker.Update({D(0)},F(0),params,.5f).front().id;
    Check(!tracker.Tracks().front().confirmed,"One observation was declared confirmed");
    for (std::uint64_t i=1;i<20;++i) tracker.Update({D(static_cast<float>(i)*.1f)},F(i),params,.5f);
    const auto moving=Id(tracker.Tracks(),initial);
    Check(moving.confirmed && std::abs(moving.velocity.x-1)<.15f && std::abs(moving.velocity.y)<.01f,
      "Constant-velocity track changed ID or used a wrong time interval");
    const auto history=moving.trail.size();
    for (int i=0;i<50;++i) tracker.Update({D(100)},F(19),params,.5f);
    Check(Id(tracker.Tracks(),initial).trail.size()==history && Id(tracker.Tracks(),initial).detection.oriented_bounds.center.x<2,
      "Repeated rendering aged, moved or duplicated a track");
    tracker.Update({},F(20),params,.5f);const auto predicted=Id(tracker.Tracks(),initial);
    Check(predicted.predicted && predicted.detection.oriented_bounds.center.x>moving.detection.oriented_bounds.center.x,
      "Missing detection did not coast with the same ID");
    tracker.Update({},F(21),params,.5f);tracker.Update({D(2.2f)},F(22),params,.5f);
    Check(Id(tracker.Tracks(),initial).confirmed && !Id(tracker.Tracks(),initial).predicted,
      "Short occlusion created a new object ID");
    tracker.Update({D(3.1f)},F(23,4.1),params,.5f);
    Check(tracker.Tracks().size()==1 && tracker.Tracks().front().id!=initial,"Expired track was revived");

    ObjectTracker crossing;auto objects=crossing.Update({D(-3),D(3)},F(0),params,.5f);
    const auto rightward=objects[0].id,leftward=objects[1].id;
    for (std::uint64_t i=1;i<=40;++i) {
      const float position=-3+static_cast<float>(i)*.15f;
      std::vector<algorithms::Detection> detections{D(position),D(-position)};
      if (i%2) std::reverse(detections.begin(),detections.end());
      crossing.Update(detections,F(i),params,.5f);
    }
    Check(Id(crossing.Tracks(),rightward).velocity.x>1.2f && Id(crossing.Tracks(),rightward).detection.oriented_bounds.center.x>2.5f &&
      Id(crossing.Tracks(),leftward).velocity.x< -1.2f,"Crossing or reordered detections swapped IDs");
    Check(Id(crossing.Tracks(),rightward).trail.size()<=ObjectTracker::kTrailLength,"Trajectory history grew without bound");

    // Longitudinal box-center jitter makes the other car's center closer.
    // Their rotated footprints still occupy separate lanes.
    ObjectTracker adjacent_lanes;
    const auto car_box=[](float x,float y,float yaw) {
      constexpr float angle=.3f;
      const float box[]{48.f+std::cos(angle)*x-std::sin(angle)*y,std::sin(angle)*x+std::cos(angle)*y,
        .8f,5.2f,1.2f,1.6f,yaw+angle,.8f,0};
      return algorithms::DecodeCenterPointBox(box,0);
    };
    for (std::uint64_t i=0;i<8;++i)
      adjacent_lanes.Update({car_box(0,0,0),car_box(2,1,0)},F(i),params,.5f);
    adjacent_lanes.Update({car_box(.7f,1,-.05f),car_box(1.3f,0,.05f)},F(8),params,.5f);
    Check(adjacent_lanes.Tracks().size()==2 && Id(adjacent_lanes.Tracks(),1).confirmed &&
      Id(adjacent_lanes.Tracks(),2).confirmed && Id(adjacent_lanes.Tracks(),1).measurement_index==1 &&
      Id(adjacent_lanes.Tracks(),2).measurement_index==0,
      "Longitudinal detection jitter swapped adjacent oriented vehicle tracks");

    ObjectTracker typed;const auto person=typed.Update({D(0,0,3)},F(0),params,.5f).front().id;
    typed.Update({D(0,0,0)},F(1),params,.5f);
    Check(typed.Tracks().size()==1 && typed.Tracks().front().id!=person && typed.Tracks().front().detection.class_id==0,
      "Different semantic classes shared an ID");
    ObjectTracker brief;brief.Update({D(0)},F(0),params,.5f);brief.Update({},F(1),params,.5f);
    Check(brief.Tracks().empty(),"A single-frame false detection was displayed as a prediction");

    ObjectTracker interrupted;
    const auto interrupted_id=interrupted.Update({D(0)},F(0),params,.5f).front().id;
    for (std::uint64_t i=1;i<=4;++i) {
      interrupted.Update({D(static_cast<float>(i)*.1f,0,3,0,i%2 ? .42f : .85f)},F(i),params,.5f);
      if (i%2) Check(interrupted.Tracks().empty(),"A weak observation confirmed or displayed a tentative track");
    }
    Check(Id(interrupted.Tracks(),interrupted_id).confirmed,"An isolated missing scan erased probation evidence");

    ObjectTracker weak;
    const auto weak_id=weak.Update({D(0)},F(0),params,.5f).front().id;
    weak.Update({D(.1f)},F(1),params,.5f);
    weak.Update({D(.2f,0,3,0,.85f)},F(2),params,.5f);
    weak.Update({D(20,0,3,0,.48f),D(.3f,0,3,0,.40f)},F(3),params,.5f);
    const auto & recovered=Id(weak.Tracks(),weak_id);
    Check(weak.Tracks().size()==1 && recovered.confirmed && !recovered.predicted && recovered.low_confidence_update &&
      recovered.measurement_index==1 && recovered.last_strong_confidence==.85f && recovered.detection.confidence==.40f,
      "Weak detections were promoted, duplicated, or failed to update an existing ID");
    weak.Update({D(.4f,0,3,0,.4f),D(.45f,0,3,0,.82f)},F(4),params,.5f);
    const auto & refreshed=Id(weak.Tracks(),weak_id);
    Check(!refreshed.low_confidence_update && refreshed.measurement_index==1 && refreshed.last_strong_confidence==.82f,
      "A weak detection displaced a valid strong measurement");
    weak.Update({D(.5f,0,3,0,.20f)},F(5),params,.5f);
    Check(Id(weak.Tracks(),weak_id).predicted,"A candidate below the weak floor steered a track");
    for (std::uint64_t i=6;i<=13;++i) weak.Update({D(static_cast<float>(i)*.1f,0,3,0,.4f)},F(i),params,.5f);
    Check(weak.Tracks().empty(),"Weak observations extended a track indefinitely");

    ObjectTracker partial_shape;
    for (std::uint64_t i=0;i<8;++i) partial_shape.Update({D(0,0,0)},F(i),params,.5f);
    const float partial[]{.25f,.1f,.9f,2.5f,1.3f,1.4f,.72f,.4f,0};
    partial_shape.Update({algorithms::DecodeCenterPointBox(partial,0)},F(8),params,.5f);
    const auto & partial_track=Id(partial_shape.Tracks(),1);
    const auto & partial_box=partial_track.detection.oriented_bounds;
    Check(partial_track.low_confidence_update && partial_box.center.x>.01f &&
      std::abs(partial_box.half_length-2.f)<1e-5f && std::abs(partial_box.half_width-1.f)<1e-5f &&
      std::abs(partial_box.axis_x.y)<1e-5f,
      "A weak partial detection altered the tracked dimensions or heading while recovering position");

    ObjectTracker young;
    const auto young_id=young.Update({D(0)},F(0),params,.5f).front().id;
    young.Update({D(.1f)},F(1),params,.5f);young.Update({D(.2f)},F(2),params,.5f);
    for (std::uint64_t i=3;i<=6;++i) young.Update({},F(i),params,.5f);
    Check(young.Tracks().empty(),"A briefly established track displayed an extended forecast");
    young.Update({D(.7f)},F(7),params,.5f);
    Check(Id(young.Tracks(),young_id).confirmed && !Id(young.Tracks(),young_id).predicted,
      "Hiding a low-evidence forecast prevented reacquiring its ID within the hold time");

    ObjectTracker overlap;
    auto close=overlap.Update({D(0),D(.12f)},F(0),params,.5f);
    const auto covered_id=close[1].id;
    for (std::uint64_t i=1;i<8;++i) overlap.Update({D(0),D(.12f)},F(i),params,.5f);
    overlap.Update({D(0)},F(8),params,.5f);
    Check(overlap.Tracks().size()==1 && !overlap.Tracks().front().predicted,
      "A covered forecast stacked a second box on the measured object");
    overlap.Update({D(0),D(.12f)},F(9),params,.5f);
    Check(Id(overlap.Tracks(),covered_id).confirmed,"Hiding a covered forecast discarded its original ID");

    ObjectTracker stale;
    auto both=stale.Update({D(0),D(1.4f)},F(0),params,.5f);
    const auto older=both[0].id,recent=both[1].id;
    for (std::uint64_t i=1;i<30;++i) stale.Update({D(0),D(1.4f)},F(i),params,.5f);
    for (std::uint64_t i=30;i<37;++i) stale.Update({D(1.4f)},F(i),params,.5f);
    stale.Update({D(.95f)},F(37),params,.5f);
    Check(Id(stale.Tracks(),older).predicted && Id(stale.Tracks(),recent).measurement_index==0,
      "An old, less likely track stole a nearby object's detection");

    ObjectTracker mature;
    const auto mature_id=mature.Update({D(0)},F(0),params,.5f).front().id;
    mature.Update({D(0)},F(1),params,.5f);
    mature.Update({D(0)},F(2),params,.5f);
    mature.Update({D(0),D(.6f)},F(3),params,.5f);
    mature.Update({D(.5f)},F(4),params,.5f);
    Check(Id(mature.Tracks(),mature_id).measurement_index==0,
      "A tentative track stole the only viable detection from a confirmed track");

    ObjectTracker turn;const auto car=turn.Update({D(0,0,0,3.13f)},F(0),params,.5f).front().id;
    for (std::uint64_t i=1;i<20;++i) turn.Update({D(.1f*static_cast<float>(i),0,0,i%2 ? -3.13f : .01f)},F(i),params,.5f);
    const auto & oriented=Id(turn.Tracks(),car).detection.oriented_bounds;
    Check(std::abs(oriented.axis_x.y)<.05f && std::isfinite(oriented.center.x),"Yaw wrap or pi ambiguity rotated a stable box");

    ObjectTracker vehicle;
    const auto vehicle_id=vehicle.Update({D(0,0,0)},F(0),params,.5f).front().id;
    vehicle.Update({D(0,0,0)},F(1),params,.5f);
    vehicle.Update({D(0,0,0)},F(2),params,.5f);
    const float swapped[]{0,0,.9f,2.f,4.f,1.8f,1.5707963f,.8f,0};
    for (std::uint64_t i=3;i<=12;++i) {
      vehicle.Update({i%2 ? D(0,0,2) : algorithms::DecodeCenterPointBox(swapped,0)},F(i),params,.5f);
      const auto & track=Id(vehicle.Tracks(),vehicle_id);
      const auto & box=track.detection.oriented_bounds;
      Check(vehicle.Tracks().size()==1 && !track.predicted && track.detection.class_id==0 &&
        std::abs(box.axis_x.y)<.02f && std::abs(box.half_length-2.f)<.02f && std::abs(box.half_width-1.f)<.02f,
        "An equivalent rectangle or momentary car/truck disagreement flipped the box or ID");
    }
    for (std::uint64_t i=13;i<=15;++i) vehicle.Update({D(0,0,2)},F(i),params,.5f);
    Check(Id(vehicle.Tracks(),vehicle_id).detection.class_id==2,"Repeated vehicle class evidence did not update the label");

    ObjectTracker discontinuity;auto id=discontinuity.Update({D(0)},F(10),params,.5f).front().id;
    auto next=discontinuity.Update({D(0)},F(5),params,.5f).front().id;
    Check(next>id && discontinuity.Stats().reset_reason=="seek or loop","Backward seek reused old IDs");id=next;
    next=discontinuity.Update({D(0)},F(20),params,.5f).front().id;
    Check(next>id,"Forward seek reused old IDs");id=next;
    next=discontinuity.Update({D(0)},F(0,-1,2),params,.5f).front().id;
    Check(next>id,"Implicit source reset recycled an ID");id=next;
    discontinuity.Reset();next=discontinuity.Update({D(0)},F(0),params,.5f).front().id;
    Check(next>id,"Explicit reset recycled an ID");

    ObjectTracker recording;
    for (std::uint64_t frame=0;frame<7;++frame) recording.Update({D(0),D(8)},F(frame),params,.5f);
    recording.Update({D(0)},F(7),params,.5f);
    Check(Id(recording.Tracks(),1).confirmed && Id(recording.Tracks(),2).predicted,
      "Recording reset fixture lacks active and coasting tracks");
    recording.ResetSequence();
    Check(recording.Tracks().empty() && recording.Stats().confirmed==0 && recording.Stats().coasting==0,
      "Starting a recording retained previous tracks or statistics");
    recording.Update({D(20),D(28)},F(7),params,.5f);
    Check(recording.Tracks().size()==2 &&
      Id(recording.Tracks(),1).detection.oriented_bounds.center.x==20 &&
      Id(recording.Tracks(),2).detection.oriented_bounds.center.x==28 &&
      !Id(recording.Tracks(),1).confirmed && !Id(recording.Tracks(),2).predicted,
      "A new recording did not restart IDs with fresh observations");

    ObjectTracker live;id=live.Update({D(0)},F(1,1,1,true),params,.5f).front().id;
    live.Update({D(.2f)},F(4,1.2,1,true),params,.5f);
    Check(live.Tracks().front().id==id,"Latest-frame live subscription reset on a skipped sequence");
    live.Update({D(.4f)},F(7,1.1,1,true),params,.5f);
    Check(live.Tracks().front().id!=id,"Reversed sensor time retained old velocity");

    ObjectTracker slow_live;
    const auto slow_id=slow_live.Update({D(0)},F(1,1,1,true),params,.5f).front().id;
    slow_live.Update({D(.2f)},F(11,2,1,true),params,.5f);
    slow_live.Update({D(.4f)},F(21,3,1,true),params,.5f);
    Check(Id(slow_live.Tracks(),slow_id).confirmed,"Slow successful live inference reset the ID");
    slow_live.Update({},F(31,4,1,true),params,.5f);
    Check(slow_live.Tracks().empty(),"Slow live inference kept a prediction past the configured timeout");

    ObjectTracker noise;double raw_error=0,filtered_error=0;
    for (std::uint64_t i=0;i<150;++i) {
      const float truth=static_cast<float>(i)*.05f,measurement=truth+.22f*std::sin(static_cast<float>(i)*2.1f);
      noise.Update({D(measurement)},F(i),params,.5f);
      if (i>20) {raw_error+=(measurement-truth)*(measurement-truth);const auto e=noise.Tracks().front().detection.oriented_bounds.center.x-truth;filtered_error+=e*e;}
    }
    Check(filtered_error<raw_error*.8,"Kalman update did not reduce measurement jitter");
    ObjectTracker burst;
    const auto burst_id=burst.Update({D(-.25f)},F(0,1),params,.5f).front().id;
    for (std::uint64_t i=1;i<=5;++i)
      burst.Update({D(-.25f+static_cast<float>(i)*.10f)},F(i,1+i*.002),params,.5f);
    const float before_gap=Id(burst.Tracks(),burst_id).detection.oriented_bounds.center.x;
    burst.Update({},F(6,1.21),params,.5f);
    const auto & after_gap=Id(burst.Tracks(),burst_id);
    Check(after_gap.predicted && std::abs(after_gap.detection.oriented_bounds.center.x-before_gap)<.35f,
      "A burst of millisecond-spaced jitter caused a large phantom displacement on the next miss");
    ObjectTracker bounded;std::vector<algorithms::Detection> many;
    for (int i=0;i<700;++i) many.push_back(D(static_cast<float>(i)*3));
    bounded.Update(many,F(0),params,.5f);
    Check(bounded.Tracks().size()==ObjectTracker::kMaxTracks && bounded.Stats().capacity_dropped==700-ObjectTracker::kMaxTracks,
      "Tracking capacity was not bounded and reported");
    ObjectTracker sparse;
    std::vector<algorithms::Detection> separated;
    for (int i=0;i<64;++i) {
      const float x=8.f*static_cast<float>(i)+(i%2 ? -7.2f : 0.f);
      separated.push_back(D(x));
    }
    sparse.Update(separated,F(0),params,.5f);
    std::reverse(separated.begin(),separated.end());
    sparse.Update(separated,F(1),params,.5f);
    for (std::size_t i=0;i<separated.size();++i)
      Check(Id(sparse.Tracks(),i+1).measurement_index==separated.size()-1-i,
        "Solving independent groups associated a track outside its original group");
    auto invalid=D(0);invalid.oriented_bounds.center.x=std::numeric_limits<float>::quiet_NaN();
    ObjectTracker robust;robust.Update({invalid,D(0,0,3,0,.2f)},F(0),params,.5f);
    Check(robust.Tracks().empty(),"Invalid or below-threshold boxes created tracks");
    robust.Update({D(0,0,3,0,.99f),D(10,0,3,0,1.f)},F(1),params,1.f);
    Check(robust.Tracks().size()==1 && robust.Tracks().front().detection.confidence==1.f,
      "The maximum UI confidence threshold did not filter tracks correctly");
    params.max_missed_s=std::numeric_limits<float>::infinity();bool rejected=false;
    try {robust.Update({},F(1),params,.5f);} catch (const std::invalid_argument &) {rejected=true;}
    Check(rejected,"Invalid tracking parameters were accepted");
    std::cout<<"Tracking: identity, crossing, class gates, sensor time, occlusion, yaw, reset, smoothing and bounds passed\n";
    return 0;
  } catch (const std::exception & error) {std::cerr<<error.what()<<'\n';return 1;}
}
