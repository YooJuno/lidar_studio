#include "lidar_viewer/algorithms/clustering_cuda.hpp"
#include "cuda_workspace.cuh"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace lidar_viewer::algorithms
{
namespace
{
struct DevicePoint
{
  float x;
  float y;
  float z;
  float intensity;
};

__device__ std::uint64_t DeviceGridKey(int x, int y)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
    static_cast<std::uint32_t>(y);
}

__global__ void BuildAssignmentsKernel(
  const DevicePoint * points,
  CudaCellAssignment * assignments,
  std::size_t count,
  float cell_size)
{
  const std::size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }

  const int cell_x = static_cast<int>(floorf(points[index].x / cell_size));
  const int cell_y = static_cast<int>(floorf(points[index].y / cell_size));
  assignments[index].key = DeviceGridKey(cell_x, cell_y);
  assignments[index].point_index = index;
}

bool HasCudaDevice()
{
  int device_count = 0;
  const cudaError_t error = cudaGetDeviceCount(&device_count);
  return error == cudaSuccess && device_count > 0;
}
}  // namespace

bool BuildCellAssignmentsCuda(
  const std::vector<KittiPoint> & input, std::size_t point_count,
  const ClusteringParams & params,
  std::vector<CudaCellAssignment> & assignments)
{
  assignments.clear();
  if (point_count == 0 || point_count > input.size() || !HasCudaDevice()) {
    return false;
  }

  const float cell_size = std::max(params.grid_cell_m, 0.05f);
  thread_local CudaBuffer<DevicePoint> points_buffer;
  thread_local CudaBuffer<CudaCellAssignment> assignments_buffer;
  const std::size_t points_bytes = point_count * sizeof(DevicePoint);
  const std::size_t assignments_bytes = point_count * sizeof(CudaCellAssignment);

  if (!points_buffer.Reserve(point_count) || !assignments_buffer.Reserve(point_count)) {
    return false;
  }
  auto * device_points = points_buffer.Get();
  auto * device_assignments = assignments_buffer.Get();
  cudaError_t error = cudaMemcpy(device_points, input.data(), points_bytes, cudaMemcpyHostToDevice);
  if (error != cudaSuccess) {
    return false;
  }

  constexpr int kThreadsPerBlock = 256;
  const int blocks = static_cast<int>((point_count + kThreadsPerBlock - 1) / kThreadsPerBlock);
  BuildAssignmentsKernel<<<blocks, kThreadsPerBlock>>>(
    device_points,
    device_assignments,
    point_count,
    cell_size);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return false;
  }

  assignments.resize(point_count);
  error = cudaMemcpy(assignments.data(), device_assignments, assignments_bytes, cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) {
    assignments.clear();
    return false;
  }

  return true;
}
}  // namespace lidar_viewer::algorithms
