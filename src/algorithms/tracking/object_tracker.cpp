#include "lidar_viewer/object_tracker.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>

namespace lidar_viewer
{
namespace
{
using State=Eigen::Matrix<double,10,1>;
using Measurement=Eigen::Matrix<double,7,1>;
constexpr double pi=3.14159265358979323846, forbidden=1e6, unmatched=1.1;
constexpr double strong_gate=16.27, weak_gate=7.81;
constexpr float weak_score_fraction=.6f, max_weak_age_s=.5f;
constexpr int max_confirmation_hits=20;

bool Valid(const algorithms::Detection & d)
{
  const auto & b=d.oriented_bounds;
  if (!b.valid || d.class_id<0 || d.class_id>4 || !std::isfinite(d.confidence) ||
      d.confidence<0 || d.confidence>1) return false;
  for (float v:{b.center.x,b.center.y,b.center.z,b.axis_x.x,b.axis_x.y,b.half_length,b.half_width,b.min_z,b.max_z})
    if (!std::isfinite(v)) return false;
  return b.half_length>0 && b.half_length<=50 && b.half_width>0 && b.half_width<=50 &&
    b.max_z>b.min_z && b.max_z-b.min_z<=30 && std::abs(b.center.x)<1e6 &&
    std::abs(b.center.y)<1e6 && std::abs(b.center.z)<1e6 && std::hypot(b.axis_x.x,b.axis_x.y)>.5;
}

Measurement Measure(const algorithms::Detection & d)
{
  const auto & b=d.oriented_bounds;
  Measurement z;z<<b.center.x,b.center.y,b.center.z,std::atan2(b.axis_x.y,b.axis_x.x),
    b.half_length*2,b.half_width*2,b.max_z-b.min_z;
  if (d.class_id!=algorithms::DetectionClassPerson && z[4]<z[5]) {
    // Equivalent rectangles can switch which axis the detector calls length.
    std::swap(z[4],z[5]);
    z[3]+=pi*.5;
  }
  return z;
}

Measurement MeasurementVariance(const State & x)
{
  const double sigma=.20+.004*std::hypot(x[0],x[1]);
  Measurement variance;
  variance<<sigma*sigma,sigma*sigma,.09,.09,.16,.16,.16;
  return variance;
}

algorithms::Detection Box(const State & x,int class_id,float score)
{
  algorithms::Detection result;result.class_id=class_id;result.confidence=score;
  auto & b=result.oriented_bounds;
  b.center={static_cast<float>(x[0]),static_cast<float>(x[1]),static_cast<float>(x[2])};
  b.axis_x={static_cast<float>(std::cos(x[3])),static_cast<float>(std::sin(x[3])),0};
  b.axis_y={-b.axis_x.y,b.axis_x.x,0};
  b.half_length=static_cast<float>(std::max(.01,x[4])*.5);
  b.half_width=static_cast<float>(std::max(.01,x[5])*.5);
  b.min_z=b.center.z-static_cast<float>(std::max(.01,x[6])*.5);
  b.max_z=b.center.z+static_cast<float>(std::max(.01,x[6])*.5);b.valid=true;
  const float ex=std::abs(b.axis_x.x)*b.half_length+std::abs(b.axis_y.x)*b.half_width;
  const float ey=std::abs(b.axis_x.y)*b.half_length+std::abs(b.axis_y.y)*b.half_width;
  result.bounds={{b.center.x-ex,b.center.y-ey,b.min_z},{b.center.x+ex,b.center.y+ey,b.max_z}};
  return result;
}

// Rectangular shortest-augmenting-path Hungarian assignment. Dummy columns
// allow each track to remain unmatched instead of forcing an invalid link.
std::vector<int> AssignDense(const std::vector<double> & costs,std::size_t rows,std::size_t real_columns)
{
  if (!rows || !real_columns) return std::vector<int>(rows,-1);
  const auto columns=real_columns+rows;
  std::vector<double> u(rows+1),v(columns+1),minimum(columns+1);
  std::vector<int> p(columns+1),way(columns+1),result(rows,-1);
  std::vector<unsigned char> used(columns+1);
  for (std::size_t i=1;i<=rows;++i) {
    p[0]=static_cast<int>(i);std::fill(minimum.begin(),minimum.end(),forbidden*2);
    std::fill(used.begin(),used.end(),0);std::size_t column=0;
    do {
      used[column]=1;const auto row=static_cast<std::size_t>(p[column]);double delta=forbidden*2;std::size_t next=0;
      for (std::size_t j=1;j<=columns;++j) if (!used[j]) {
        const double cost=(j<=real_columns ? costs[(row-1)*real_columns+j-1] : unmatched)-u[row]-v[j];
        if (cost<minimum[j]) {minimum[j]=cost;way[j]=static_cast<int>(column);}
        if (minimum[j]<delta) {delta=minimum[j];next=j;}
      }
      for (std::size_t j=0;j<=columns;++j) {
        if (used[j]) {u[static_cast<std::size_t>(p[j])]+=delta;v[j]-=delta;}
        else minimum[j]-=delta;
      }
      column=next;
    } while (p[column]!=0);
    do {const auto previous=static_cast<std::size_t>(way[column]);p[column]=p[previous];column=previous;} while(column);
  }
  for (std::size_t j=1;j<=real_columns;++j) if (p[j]>0 &&
      costs[(static_cast<std::size_t>(p[j])-1)*real_columns+j-1]<unmatched)
    result[static_cast<std::size_t>(p[j])-1]=static_cast<int>(j-1);
  return result;
}

// Rejected pairs do not couple otherwise independent assignment problems.
// Solve each connected component with its own unmatched columns rather than
// paying for every isolated or out-of-range target in the dense solver.
std::vector<int> Assign(const std::vector<double> & costs,std::size_t rows,std::size_t columns)
{
  if (rows*columns<=256) return AssignDense(costs,rows,columns);
  std::vector<std::size_t> parent(rows+columns);
  std::iota(parent.begin(),parent.end(),0);
  std::vector<unsigned char> linked(rows+columns,0);
  const auto root=[&](std::size_t node) {
    while (parent[node]!=node) {
      parent[node]=parent[parent[node]];
      node=parent[node];
    }
    return node;
  };
  for (std::size_t row=0;row<rows;++row) for (std::size_t col=0;col<columns;++col) {
    if (costs[row*columns+col]>=unmatched) continue;
    parent[root(rows+col)]=root(row);
    linked[row]=linked[rows+col]=1;
  }
  struct Component {std::vector<std::size_t> rows,columns;};
  std::vector<Component> components(rows);
  for (std::size_t row=0;row<rows;++row) if (linked[row]) components[root(row)].rows.push_back(row);
  for (std::size_t col=0;col<columns;++col) if (linked[rows+col])
    components[root(rows+col)].columns.push_back(col);

  std::vector<int> result(rows,-1);
  std::vector<double> local;
  for (const auto & component:components) {
    const auto & selected_rows=component.rows;
    const auto & selected_columns=component.columns;
    if (selected_rows.empty()) continue;
    if (selected_rows.size()==rows && selected_columns.size()==columns)
      return AssignDense(costs,rows,columns);
    if (selected_rows.size()==1 && selected_columns.size()==1) {
      result[selected_rows.front()]=static_cast<int>(selected_columns.front());
      continue;
    }
    local.clear();
    local.reserve(selected_rows.size()*selected_columns.size());
    for (const auto row:selected_rows) for (const auto col:selected_columns) local.push_back(costs[row*columns+col]);
    const auto assignments=AssignDense(local,selected_rows.size(),selected_columns.size());
    for (std::size_t i=0;i<assignments.size();++i) if (assignments[i]>=0)
      result[selected_rows[i]]=static_cast<int>(selected_columns[static_cast<std::size_t>(assignments[i])]);
  }
  return result;
}

struct TrackState
{
  State x=State::Zero();
  // Independent position/velocity pairs and independent yaw/dimension
  // states. Only each position's covariance with its own velocity is nonzero
  // away from the diagonal.
  State variance=State::Ones();
  Eigen::Vector3d position_velocity_covariance=Eigen::Vector3d::Zero();
  std::uint64_t id{0};int class_id{-1},streak{1},strong_hits{1};bool confirmed{false};
  int alternate_class{-1},alternate_hits{0};
  float score{0},strong_score{0};
  std::int64_t last_measurement{0},last_strong{0};
  bool weak_update{false},missed_previous{false};
  std::optional<std::size_t> measurement;
  std::vector<Vec3> trail;
};


struct Observation
{
  std::size_t index;
  Measurement box;
  bool strong;
};

bool IsVehicle(int class_id)
{
  return class_id==algorithms::DetectionClassCar || class_id==algorithms::DetectionClassBus ||
    class_id==algorithms::DetectionClassTruck;
}

bool ForecastCoveredBy(const TrackState & forecast,const Measurement & box,int observed_class)
{
  if (forecast.class_id!=observed_class && !(IsVehicle(forecast.class_id) && IsVehicle(observed_class))) return false;
  const Eigen::Vector3d offset=forecast.x.head<3>()-box.head<3>();
  const Eigen::Vector3d smaller=forecast.x.segment<3>(4).cwiseMin(box.segment<3>(4));
  const Eigen::Vector3d larger=forecast.x.segment<3>(4).cwiseMax(box.segment<3>(4));
  if (offset.head<2>().squaredNorm()>larger.head<2>().squaredNorm()*.0625 ||
      std::abs(offset.z())>smaller.z()*.25 || (smaller.array()<larger.array()*.7).any()) return false;
  const double c=std::cos(box[3]),s=std::sin(box[3]);
  return std::abs(c*offset.x()+s*offset.y())<smaller.x()*.25 &&
    std::abs(-s*offset.x()+c*offset.y())<smaller.y()*.25;
}

struct FootprintPoint {double x,y;};

double OrientedVolumeIoU(const State & track, const Measurement & observation)
{
  const double h1=std::max(.01,track[6])*.5, h2=observation[6]*.5;
  const double dz=observation[2]-track[2];
  const double height=std::min(h1,dz+h2)-std::max(-h1,dz-h2);
  if (height<=0) return 0;

  // Work in the predicted box's local coordinates. The observation is a
  // rotated rectangle, so clipping against the four axis-aligned sides is
  // enough and each intermediate polygon has at most eight vertices.
  const double dx=observation[0]-track[0], dy=observation[1]-track[1];
  const double c=std::cos(track[3]), s=std::sin(track[3]);
  const double cx=c*dx+s*dy, cy=-s*dx+c*dy;
  const double l1=std::max(.01,track[4])*.5, w1=std::max(.01,track[5])*.5;
  const double l2=observation[4]*.5, w2=observation[5]*.5;
  const double yaw=observation[3]-track[3], cos_yaw=std::cos(yaw), sin_yaw=std::sin(yaw);
  const double lx=l2*cos_yaw, ly=l2*sin_yaw, wx=-w2*sin_yaw, wy=w2*cos_yaw;
  if (std::abs(cx)>=l1+std::abs(lx)+std::abs(wx) || std::abs(cy)>=w1+std::abs(ly)+std::abs(wy))
    return 0;

  std::array<FootprintPoint,8> a{{
    {cx-lx-wx,cy-ly-wy}, {cx+lx-wx,cy+ly-wy}, {cx+lx+wx,cy+ly+wy}, {cx-lx+wx,cy-ly+wy}
  }}, b;
  auto * polygon=&a;
  auto * clipped=&b;
  std::size_t count=4;
  for (int side=0;side<4;++side) {
    const auto distance=[&](const FootprintPoint & p) {
      return side<2 ? l1+(side==0 ? p.x : -p.x) : w1+(side==2 ? p.y : -p.y);
    };
    auto previous=(*polygon)[count-1];
    double previous_distance=distance(previous);
    std::size_t kept=0;
    for (std::size_t i=0;i<count;++i) {
      const auto current=(*polygon)[i];
      const double current_distance=distance(current);
      if ((previous_distance>0 && current_distance<0) || (previous_distance<0 && current_distance>0)) {
        const double t=previous_distance/(previous_distance-current_distance);
        (*clipped)[kept++]={previous.x+t*(current.x-previous.x),previous.y+t*(current.y-previous.y)};
      }
      if (current_distance>=0) (*clipped)[kept++]=current;
      previous=current;
      previous_distance=current_distance;
    }
    if (kept<3) return 0;
    std::swap(polygon,clipped);
    count=kept;
  }
  double twice_area=0;
  for (std::size_t i=0;i<count;++i) {
    const auto p=(*polygon)[i], q=(*polygon)[(i+1)%count];
    twice_area+=p.x*q.y-p.y*q.x;
  }
  const double volume1=8*l1*w1*h1, volume2=8*l2*w2*h2;
  const double overlap=std::min(std::abs(twice_area)*.5*height,std::min(volume1,volume2));
  return overlap/(volume1+volume2-overlap);
}

void Predict(TrackState & track,double dt)
{
  const double dt2=dt*dt;
  for (int position=0;position<3;++position) {
    const int velocity=position+7;
    const double acceleration_density=position==2 ? .4 : 1.6;
    double & covariance=track.position_velocity_covariance[position];
    // Continuous acceleration noise accumulates consistently across skipped scans.
    track.x[position]+=dt*track.x[velocity];
    track.variance[position]+=2.*dt*covariance+dt2*track.variance[velocity]+acceleration_density*dt2*dt/3.;
    covariance+=dt*track.variance[velocity]+acceleration_density*dt2*.5;
    track.variance[velocity]+=acceleration_density*dt;
  }
  track.variance[3]+=.09*dt;
  track.variance.segment<3>(4).array()+=.01*dt;
  track.measurement.reset();
  track.weak_update=false;
}

double AssociationCost(const TrackState & track,const algorithms::Detection & detection,
  const Measurement & z,const Eigen::Vector3d & inverse,bool weak,double unobserved_s)
{
  const bool changed_class=track.class_id!=detection.class_id;
  if (changed_class && (weak || !track.confirmed || !IsVehicle(track.class_id) || !IsVehicle(detection.class_id)))
    return forbidden;
  const bool strict=weak || changed_class;
  const double gate=strict ? weak_gate : strong_gate;
  const bool person=track.class_id==algorithms::DetectionClassPerson;
  const double max_distance=strict ? std::clamp(.4*track.x[4]+.4,.75,2.5) : person ? 4. : 12.;
  const Eigen::Vector3d delta=z.head<3>()-track.x.head<3>();
  const Eigen::Vector3d squared=delta.cwiseProduct(delta);
  const double mahal=squared.dot(inverse), distance_squared=squared.sum();
  if (!std::isfinite(mahal) || mahal>gate || distance_squared>max_distance*max_distance) return forbidden;
  double size_cost=0;
  for (int axis=4;axis<7;++axis) {
    const double ratio=z[axis]/std::max(.01,track.x[axis]);
    if (ratio<(strict ? .55 : .25) || ratio>(strict ? 1.8 : 4.)) return forbidden;
    size_cost+=std::abs(std::log(ratio));
  }
  const double heading=person ? 0. : std::abs(std::remainder(z[3]-track.x[3],pi))/(pi*.5);
  // A large covariance can keep a stale track eligible, but should not give it
  // an advantage over a closer, recently measured track.
  const double distance_scale=std::clamp(.5*track.x[4]+.5,1.,3.);
  const double distance_cost=std::min(1.,std::sqrt(distance_squared)/distance_scale);
  return .45*mahal/gate+.30*distance_cost+.10*std::min(1.,size_cost/3)+.05*heading+
    .025*(1-detection.confidence)+.12*std::clamp(unobserved_s/.6,0.,1.)+(changed_class ? .15 : 0.);
}

void CorrectMeasurement(TrackState & track,const Measurement & z,const Measurement & noise,bool strong)
{
  const int dimensions=strong ? 7 : 3;
  for (int i=0;i<dimensions;++i) {
    const double variance=track.variance[i], innovation_variance=variance+noise[i];
    const double innovation=z[i]-track.x[i];
    const double gain=variance/innovation_variance, residual=1.-gain;
    track.x[i]+=gain*innovation;
    // Joseph-form covariance correction, specialized for a scalar observation.
    track.variance[i]=residual*residual*variance+gain*gain*noise[i];
    if (i<3) {
      const int velocity=i+7;
      double & covariance=track.position_velocity_covariance[i];
      const double velocity_gain=covariance/innovation_variance;
      track.x[velocity]+=velocity_gain*innovation;
      track.variance[velocity]+=velocity_gain*(velocity_gain*innovation_variance-2.*covariance);
      covariance=residual*(covariance-velocity_gain*variance)+gain*velocity_gain*noise[i];
    }
  }
}

void Correct(TrackState & track,const algorithms::Detection & detection,
  const Observation & observation,std::int64_t stamp,double dt)
{
  auto z=observation.box;
  // A 3D box's heading is ambiguous by pi. Correct the nearest equivalent
  // orientation, including wraparound at +/-pi, before the linear update.
  z[3]=track.x[3]+std::remainder(z[3]-track.x[3],pi);
  // Closely spaced detections share noise. Limit their influence per unit of
  // sensor time. Weak observations can recover position through occlusion,
  // where shape and heading often describe only the visible part of an object.
  const double temporal_scale=std::max(1.,.03/std::max(dt,.001));
  const double noise_scale=observation.strong ? temporal_scale : 4.*temporal_scale;
  CorrectMeasurement(track,z,MeasurementVariance(track.x)*noise_scale,observation.strong);
  track.x[3]=std::remainder(track.x[3],2*pi);
  track.score=detection.confidence;track.last_measurement=stamp;track.measurement=observation.index;
  track.weak_update=!observation.strong;track.missed_previous=false;
  if (!observation.strong) {track.alternate_hits=0;return;}
  track.strong_score=detection.confidence;track.last_strong=stamp;
  if (track.strong_hits<2*max_confirmation_hits) ++track.strong_hits;
  if (!track.confirmed) ++track.streak;
  if (track.class_id==detection.class_id) {track.alternate_hits=0;return;}
  if (!track.alternate_hits || track.alternate_class!=detection.class_id) {
    track.alternate_class=detection.class_id;track.alternate_hits=0;
  }
  if (++track.alternate_hits==3) {
    track.class_id=track.alternate_class;track.alternate_hits=0;
  }
}
}  // namespace

struct ObjectTracker::Impl
{
  std::vector<TrackState> states;
  std::vector<ObjectTrack> output;
  std::optional<TrackingFrame> last;
  std::uint64_t next_id{1};
  TrackingStats stats;
};

ObjectTracker::ObjectTracker():impl_(std::make_unique<Impl>()) {}
ObjectTracker::~ObjectTracker()=default;
void ObjectTracker::Reset() {impl_->states.clear();impl_->output.clear();impl_->last.reset();impl_->stats={};}
void ObjectTracker::ResetSequence()
{
  Reset();
  impl_->next_id=1;
}
const std::vector<ObjectTrack> & ObjectTracker::Tracks() const {return impl_->output;}
const TrackingStats & ObjectTracker::Stats() const {return impl_->stats;}

const std::vector<ObjectTrack> & ObjectTracker::Update(const std::vector<algorithms::Detection> & detections,
  const TrackingFrame & frame,const TrackingParams & params,float min_confidence)
{
  if (frame.timestamp_ns<0 || !std::isfinite(min_confidence) || min_confidence<.1f || min_confidence>1.f ||
      !std::isfinite(params.max_missed_s) || params.max_missed_s<.05f || params.max_missed_s>10 ||
      params.confirmation_hits<1 || params.confirmation_hits>max_confirmation_hits)
    throw std::invalid_argument("Invalid tracking frame or settings");
  auto & core=*impl_;
  if (core.last && core.last->session==frame.session && core.last->sequence==frame.sequence) return core.output;
  const auto started=std::chrono::steady_clock::now();
  double dt=0;std::string reason;
  if (core.last) {
    dt=static_cast<double>((static_cast<long double>(frame.timestamp_ns)-core.last->timestamp_ns)*1e-9L);
    if (frame.session!=core.last->session) reason="source changed";
    else if (frame.sequence<core.last->sequence || (!frame.allow_skipped_frames && frame.sequence-core.last->sequence!=1)) reason="seek or loop";
    else if (dt<=0 || dt>std::max(2.,static_cast<double>(params.max_missed_s)*2)) reason="timestamp discontinuity";
  }
  if (!reason.empty()) {Reset();dt=0;}
  core.stats={};core.stats.reset_reason=reason;core.output.clear();
  const auto age=[&](std::int64_t stamp) {return static_cast<double>((static_cast<long double>(frame.timestamp_ns)-stamp)*1e-9L);};
  const auto expired = [&](const TrackState & t) {
    return age(t.last_strong)>(t.confirmed ? params.max_missed_s : std::min(.25f,params.max_missed_s));
  };
  core.states.erase(std::remove_if(core.states.begin(),core.states.end(),[&](const auto & t) {
    // Live inference can skip scans. A track measured on the last processed
    // scan gets one chance to match this result before using its miss timeout.
    return expired(t) && !(frame.allow_skipped_frames && t.measurement && !t.weak_update);
  }),core.states.end());
  for (auto & track:core.states) Predict(track,dt);

  // Keep strong candidates at the front of a bounded set. Break score ties by
  // the original input order so duplicates cannot introduce replay variation.
  const auto better=[&](std::size_t a,std::size_t b) {
    return detections[a].confidence!=detections[b].confidence ? detections[a].confidence>detections[b].confidence : a<b;
  };
  const float weak_floor=std::max(.1f,min_confidence*weak_score_fraction);
  const double weak_age_limit=std::min(max_weak_age_s,params.max_missed_s);
  const bool can_recover=std::any_of(core.states.begin(),core.states.end(),[&](const TrackState & track) {
    return track.confirmed && age(track.last_strong)<=weak_age_limit;
  });
  const float candidate_floor=can_recover ? weak_floor : min_confidence;
  std::priority_queue<std::size_t,std::vector<std::size_t>,decltype(better)> strongest(better);
  std::size_t strong_count=0;
  for (std::size_t i=0;i<detections.size();++i) if (detections[i].confidence>=candidate_floor && Valid(detections[i])) {
    if (detections[i].confidence>=min_confidence) ++strong_count;
    if (strongest.size()<kMaxMeasurements) strongest.push(i);
    else if (better(i,strongest.top())) {strongest.pop();strongest.push(i);}
  }
  core.stats.capacity_dropped=strong_count-std::min(strong_count,kMaxMeasurements);
  std::vector<Observation> observations;observations.reserve(strongest.size());
  while (!strongest.empty()) {
    const auto index=strongest.top();strongest.pop();
    observations.push_back({index,Measure(detections[index]),detections[index].confidence>=min_confidence});
  }
  std::reverse(observations.begin(),observations.end());

  std::vector<unsigned char> matched(observations.size());
  std::vector<std::size_t> rows,columns;
  std::vector<double> costs;
  const auto associate=[&](bool confirmed, bool strong) {
    rows.clear();columns.clear();
    for (std::size_t j=0;j<observations.size();++j)
      if (!matched[j] && observations[j].strong==strong) columns.push_back(j);
    if (columns.empty()) return;
    for (std::size_t i=0;i<core.states.size();++i) {
      const auto & track=core.states[i];
      if (!track.measurement && track.confirmed==confirmed && (strong || age(track.last_strong)<=weak_age_limit))
        rows.push_back(i);
    }
    if (rows.empty()) return;

    costs.resize(rows.size()*columns.size());
    std::array<std::size_t,kMaxTracks> row_candidates{};
    std::array<std::size_t,kMaxMeasurements> column_candidates{};
    const bool check_overlap=confirmed && strong && (rows.size()>1 || columns.size()>1);
    bool ambiguous=false;
    for (std::size_t i=0;i<rows.size();++i) {
      const auto & track=core.states[rows[i]];
      const Eigen::Vector3d innovation=track.variance.head<3>()+MeasurementVariance(track.x).head<3>();
      const Eigen::Vector3d inverse=innovation.cwiseInverse();
      const double unobserved=std::max(0.,age(track.last_measurement)-dt);
      for (std::size_t j=0;j<columns.size();++j) {
        const auto & observed=observations[columns[j]];
        auto & cost=costs[i*columns.size()+j];
        cost=AssociationCost(track,detections[observed.index],observed.box,inverse,!strong,unobserved);
        if (check_overlap && cost<unmatched && IsVehicle(track.class_id)) {
          const auto row_count=++row_candidates[i], column_count=++column_candidates[j];
          ambiguous|=row_count>1 || column_count>1;
        }
      }
    }
    // Compare trusted vehicle shapes where multiple valid links compete.
    // SimpleTrack uses 3D generalized IoU (arXiv:2111.09621). A bounded
    // overlap bonus here leaves the motion gates and recovery policy intact.
    if (ambiguous) for (std::size_t i=0;i<rows.size();++i) {
      if (!row_candidates[i]) continue;
      const auto & track=core.states[rows[i]];
      for (std::size_t j=0;j<columns.size();++j) {
        auto & cost=costs[i*columns.size()+j];
        if (cost<unmatched && (row_candidates[i]>1 || column_candidates[j]>1))
          cost-=.20*OrientedVolumeIoU(track.x,observations[columns[j]].box);
      }
    }
    const auto assignments=Assign(costs,rows.size(),columns.size());
    for (std::size_t i=0;i<rows.size();++i) {
      if (assignments[i]<0) continue;
      const auto j=columns[static_cast<std::size_t>(assignments[i])];
      const auto & observed=observations[j];
      auto & track=core.states[rows[i]];
      Correct(track,detections[observed.index],observed,frame.timestamp_ns,dt);
      matched[j]=1;
      if (strong && track.streak>=params.confirmation_hits) track.confirmed=true;
    }
  };
  associate(true,true);
  associate(false,true);
  if (can_recover) associate(true,false);
  for (auto & track:core.states) if (!track.measurement) {
    // One missing scan does not wipe probation evidence. Only strong hits can
    // increase it; a longer break still requires fresh confirmation.
    if (track.missed_previous || age(track.last_strong)>.20) track.streak=0;
    track.missed_previous=true;
    track.alternate_hits=0;
  }
  if (frame.allow_skipped_frames)
    core.states.erase(std::remove_if(core.states.begin(),core.states.end(),expired),core.states.end());
  for (std::size_t j=0;j<observations.size() && observations[j].strong;++j) if (!matched[j]) {
    if (core.states.size()>=kMaxTracks) {++core.stats.capacity_dropped;continue;}
    if (core.next_id==std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Tracking ID exhausted");
    const auto & observed=observations[j];const auto & detection=detections[observed.index];TrackState track;
    track.x.head<7>()=observed.box;
    track.variance.head<7>()=MeasurementVariance(track.x);
    track.variance.tail<3>().setConstant(100);
    track.id=core.next_id++;track.class_id=detection.class_id;track.score=track.strong_score=detection.confidence;
    track.last_measurement=track.last_strong=frame.timestamp_ns;track.measurement=observed.index;
    track.confirmed=params.confirmation_hits==1;
    track.trail.reserve(kTrailLength);core.states.push_back(std::move(track));
  }
  core.states.erase(std::remove_if(core.states.begin(),core.states.end(),[](const auto & track) {
    return !track.x.allFinite() || !track.variance.allFinite() || !track.position_velocity_covariance.allFinite();
  }),core.states.end());
  for (auto & track:core.states) {
    if (!track.confirmed && !track.measurement) continue;
    // Briefly established tracks can be reacquired for the entire hold window,
    // but display a shorter forecast until there is more motion evidence.
    if (!track.measurement && params.confirmation_hits>1 && track.strong_hits<2*params.confirmation_hits &&
        age(track.last_measurement)>.35) continue;
    // A trusted observation already covers this footprint. Keep the predicted
    // candidate for later association without stacking two boxes in the view.
    if (!track.measurement && std::any_of(observations.begin(),observations.end(),[&](const Observation & observed) {
        return observed.strong && ForecastCoveredBy(track,observed.box,detections[observed.index].class_id);
      })) continue;
    const Vec3 center{static_cast<float>(track.x[0]),static_cast<float>(track.x[1]),static_cast<float>(track.x[2])};
    if (track.trail.size()==kTrailLength) track.trail.erase(track.trail.begin());
    track.trail.push_back(center);
    ObjectTrack output;output.detection=Box(track.x,track.class_id,track.score);output.id=track.id;
    output.velocity={static_cast<float>(track.x[7]),static_cast<float>(track.x[8]),static_cast<float>(track.x[9])};
    output.confirmed=track.confirmed;output.predicted=!track.measurement;
    output.low_confidence_update=track.weak_update;output.last_strong_confidence=track.strong_score;
    output.missed_s=static_cast<float>(age(track.last_measurement));
    output.measurement_index=track.measurement;output.trail=track.trail;core.output.push_back(std::move(output));
    if (track.confirmed) ++core.stats.confirmed;else ++core.stats.tentative;
    if (!track.measurement) ++core.stats.coasting;
  }
  core.last=frame;
  core.stats.processing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return core.output;
}
}  // namespace lidar_viewer
